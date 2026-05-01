/****************************************************************************
 * apps/btsensor/btsensor_main.c
 *
 * btsensor daemon entry point (Issue #56 Commit B).
 *
 * Replaces the foreground `btsensor &` model with a long-lived daemon
 * spawned via `btsensor start [batch]`.  The daemon owns the BTstack
 * run loop and the IMU sampler; `btsensor stop` posts a teardown_kick
 * onto the run loop, which walks an event-driven FSM (GAP off ->
 * RFCOMM disconnect -> sampler deinit -> HCI off -> trigger_exit) so
 * BTstack's asynchronous shutdown completes cleanly before the daemon
 * returns.
 *
 * Lifecycle invariants:
 *   - g_daemon_pid > 0  iff a daemon task is running (or about to enter
 *     the run loop); cleared by the daemon itself just before exit.
 *   - g_lifecycle_lock guards g_daemon_pid, g_running, g_ts_state.
 *   - Default Bluetooth posture is "HCI up, advertising off" — the BT
 *     button handler in Commit C toggles advertising at runtime.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <arch/board/board_btuart.h>
#include <arch/board/board_lsm6dsl.h>

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_h4.h"
#include "btstack_chipset_cc256x.h"
#include "classic/btstack_link_key_db_memory.h"
#include "classic/rfcomm.h"
#include "gap.h"

#include "btstack_run_loop_nuttx.h"
#include "btstack_uart_nuttx.h"
#include "btsensor_button.h"
#include "btsensor_cmd.h"
#include "btsensor_led.h"
#include "btsensor_spp.h"
#include "btsensor_tx.h"
#include "bundle_emitter.h"
#include "imu_sampler.h"
#include "sensor_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_INIT_BAUDRATE   115200
#define BT_MAIN_BAUDRATE   3000000

#define BTSENSOR_DAEMON_NAME       "btsensor_d"
#define BTSENSOR_DAEMON_PRIORITY   100
#define BTSENSOR_DAEMON_STACKSIZE  4096

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum teardown_state
{
  TS_RUNNING = 0,         /* normal operation */
  TS_GAP_OFF_REQ,         /* GAP discoverable/connectable disabled */
  TS_RFCOMM_CLOSING,      /* waiting for RFCOMM_EVENT_CHANNEL_CLOSED */
  TS_SAMPLER_DEINIT,      /* sampler/tx down, HCI_POWER_OFF issued */
  TS_HCI_OFF_PENDING,     /* waiting for HCI_STATE_OFF */
  TS_DONE,                /* run loop exit triggered */
};

/* BT visibility state machine driven by the BT button + pairing events.
 * Default startup state is BT_OFF (HCI on but advertising / connectable
 * both off — the BT button toggles them).
 */

enum bt_state
{
  BT_OFF = 0,
  BT_ADVERTISING,
  BT_FAIL_BLINK,
  BT_PAIRED,
  BT_CONNECTABLE,   /* connectable=1 / discoverable=0 — paired peer can
                     * reconnect by BD_ADDR but inquiry is silent. Used as
                     * the post-RFCOMM-drop landing state instead of
                     * BT_ADVERTISING (Issue #68). */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const hci_transport_config_uart_t g_hci_transport_config =
{
  HCI_TRANSPORT_CONFIG_UART,
  BT_INIT_BAUDRATE,
  BT_MAIN_BAUDRATE,
  0,                          /* flowcontrol is already on in HW */
  NULL,                       /* device_name — ignored by our UART impl */
  BTSTACK_UART_PARITY_OFF,
};

static btstack_packet_callback_registration_t g_hci_event_reg;
static btstack_context_callback_registration_t g_teardown_kick_reg;
static btstack_timer_source_t                  g_teardown_timer;

#define BTSENSOR_TEARDOWN_TIMEOUT_MS  3000

static pthread_mutex_t g_lifecycle_lock = PTHREAD_MUTEX_INITIALIZER;

static volatile bool   g_running;
static int             g_daemon_pid = -1;
static enum teardown_state g_ts_state = TS_RUNNING;
static enum bt_state       g_bt_state = BT_OFF;

/* Mirror of the active RFCOMM channel id, updated by btsensor_spp_*
 * callbacks.  The teardown FSM consults it to decide whether to issue
 * rfcomm_disconnect() or jump straight to SAMPLER_DEINIT.
 */

static uint16_t g_rfcomm_cid;

/****************************************************************************
 * Private Functions — teardown FSM
 ****************************************************************************/

static void teardown_check_rfcomm(void);
static void teardown_rfcomm_closed(void);
static void teardown_sampler_off(void);
static void teardown_finish(void);

static void teardown_arm_timer(uint32_t ms);
static void teardown_cancel_timer(void);

static void teardown_timer_handler(btstack_timer_source_t *ts)
{
  (void)ts;
  syslog(LOG_WARNING, "btsensor: teardown timeout in state %d\n",
         (int)g_ts_state);

  switch (g_ts_state)
    {
      case TS_RFCOMM_CLOSING:
        g_rfcomm_cid = 0;
        teardown_rfcomm_closed();
        break;
      case TS_HCI_OFF_PENDING:
        teardown_finish();
        break;
      default:
        break;
    }
}

static void teardown_arm_timer(uint32_t ms)
{
  btstack_run_loop_remove_timer(&g_teardown_timer);
  btstack_run_loop_set_timer_handler(&g_teardown_timer,
                                     teardown_timer_handler);
  btstack_run_loop_set_timer(&g_teardown_timer, ms);
  btstack_run_loop_add_timer(&g_teardown_timer);
}

static void teardown_cancel_timer(void)
{
  btstack_run_loop_remove_timer(&g_teardown_timer);
}

static void on_teardown_kick(void *context)
{
  (void)context;

  if (g_ts_state != TS_RUNNING)
    {
      return;          /* idempotent — multiple stop requests collapse */
    }

  g_ts_state = TS_GAP_OFF_REQ;
  gap_discoverable_control(0);
  gap_connectable_control(0);
  teardown_check_rfcomm();
}

static void teardown_check_rfcomm(void)
{
  if (g_rfcomm_cid != 0)
    {
      syslog(LOG_INFO, "btsensor: teardown rfcomm disconnect cid=%u\n",
             g_rfcomm_cid);
      rfcomm_disconnect(g_rfcomm_cid);
      g_ts_state = TS_RFCOMM_CLOSING;
      teardown_arm_timer(BTSENSOR_TEARDOWN_TIMEOUT_MS);
      return;
    }

  /* No RFCOMM session — skip the closing wait. */

  teardown_sampler_off();
}

static void teardown_rfcomm_closed(void)
{
  teardown_cancel_timer();
  teardown_sampler_off();
}

static void teardown_sampler_off(void)
{
  g_ts_state = TS_SAMPLER_DEINIT;
  btsensor_button_deinit();
  btsensor_led_deinit();

  /* Strict order: stop the BUNDLE timer first so no tick can race the
   * sampler / CLAIM teardown that follows.  Then drop SENSOR (CLAIM
   * release lives here in Issue C) before closing IMU / TX state.
   */

  bundle_emitter_deinit();
  sensor_sampler_deinit();
  imu_sampler_deinit();
  btsensor_tx_deinit();
  syslog(LOG_INFO, "btsensor: teardown HCI off\n");
  hci_power_control(HCI_POWER_OFF);
  g_ts_state = TS_HCI_OFF_PENDING;
  teardown_arm_timer(BTSENSOR_TEARDOWN_TIMEOUT_MS);
}

static void teardown_finish(void)
{
  teardown_cancel_timer();
  g_ts_state = TS_DONE;
  btstack_run_loop_trigger_exit();
}

/****************************************************************************
 * Private Functions — BT state machine
 ****************************************************************************/

#ifndef CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS
#  define CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS  1000
#endif
#ifndef CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS
#  define CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS      3
#endif

static const char *bt_state_name(enum bt_state s)
{
  switch (s)
    {
      case BT_OFF:         return "off";
      case BT_ADVERTISING: return "advertising";
      case BT_FAIL_BLINK:  return "fail_blink";
      case BT_PAIRED:      return "paired";
      case BT_CONNECTABLE: return "connectable";
      default:             return "?";
    }
}

static void bt_enter_off(void)
{
  syslog(LOG_INFO, "btsensor: BT off (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_OFF;
  gap_discoverable_control(0);
  gap_connectable_control(0);
  btsensor_led_off();
}

static void bt_enter_advertising(void)
{
  syslog(LOG_INFO, "btsensor: BT advertising (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_ADVERTISING;
  gap_connectable_control(1);
  gap_discoverable_control(1);
  btsensor_led_blink_blue(CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS);
}

static void bt_enter_paired(void)
{
  syslog(LOG_INFO, "btsensor: BT paired (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_PAIRED;

  /* Stop being discoverable once we have a session — pybricks does the
   * same.  Keep connectable on so a brief drop-and-reconnect just
   * resumes streaming without the user having to press the button.
   */

  gap_discoverable_control(0);
  btsensor_led_solid_blue();
}

static void bt_enter_connectable(void)
{
  /* Post-RFCOMM-drop landing state (Issue #68): paired peer can still
   * reconnect by BD_ADDR (link key remains in btstack's in-RAM DB), but
   * inquiry is silent so a stranger cannot opportunistically pair just
   * because the previous PC went away.  Visually shares
   * BT_ADVERTISING's blink so users see "Hub is waiting for the PC" —
   * the discoverability difference is observable through `btsensor
   * status`.  bt_visibility_request() promotes back to BT_ADVERTISING
   * if the user explicitly toggles via button or NSH.
   */

  syslog(LOG_INFO, "btsensor: BT connectable (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_CONNECTABLE;
  gap_connectable_control(1);
  gap_discoverable_control(0);

  /* Issue #73: use the double-blink rhythm here so the LED is visually
   * distinct from BT_ADVERTISING's symmetric blink — both states are
   * "blue blinking", but ADVERTISING is 50% duty while CONNECTABLE is
   * "ti-tick . . . . . . ti-tick", clearly different at a glance.
   */

  btsensor_led_double_blink_blue(CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS);
}

static void bt_enter_fail_blink(void)
{
  syslog(LOG_WARNING, "btsensor: BT pairing failed (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_FAIL_BLINK;
  gap_discoverable_control(0);
  gap_connectable_control(0);
  btsensor_led_fail_blink(CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS);
}

/* Returns true iff btstack's link key DB has at least one stored entry
 * (= we have previously paired with a peer this session).  Used by
 * bt_visibility_request() to choose whether `bt on` should advertise
 * (DB empty — need to pair) or stay connectable-only (DB non-empty —
 * just wait for the known peer).
 */

static bool bt_has_stored_link_key(void)
{
  btstack_link_key_iterator_t it;
  if (!gap_link_key_iterator_init(&it))
    {
      /* No link key DB attached or iterator_init not provided; treat
       * as empty so we fall back to BT_ADVERTISING.
       */

      return false;
    }

  bd_addr_t addr;
  link_key_t link_key;
  link_key_type_t type;
  bool any = gap_link_key_iterator_get_next(&it, addr, link_key, &type) != 0;
  gap_link_key_iterator_done(&it);
  return any;
}

/* BT visibility request — usable both from the button handlers and the
 * `btsensor bt on/off` NSH dispatch.  `on=true` is "be reachable":
 * link-key-aware, going to BT_CONNECTABLE if the DB already has a paired
 * peer (silent reconnect, no inquiry exposure — Issue #71) or
 * BT_ADVERTISING otherwise (fresh pair).  No-op if already in any of
 * the reachable states (CONNECTABLE / ADVERTISING / PAIRED).  `on=false`
 * is "go silent" (RFCOMM disconnect when paired, no-op if already OFF /
 * FAIL_BLINK).  The escape hatch for forcing BT_ADVERTISING despite
 * stored keys is the long-button-press path (on_button_long), which
 * calls bt_enter_advertising() directly.
 *
 * Must be called on the BTstack main thread.  Returns 0 on success or
 * -ENXIO if the daemon is in teardown.
 */

static int bt_visibility_request(bool on)
{
  if (g_ts_state != TS_RUNNING)
    {
      return -ENXIO;
    }

  if (on)
    {
      switch (g_bt_state)
        {
          case BT_OFF:
          case BT_FAIL_BLINK:
            if (bt_has_stored_link_key())
              {
                bt_enter_connectable();
              }
            else
              {
                bt_enter_advertising();
              }
            return 0;
          case BT_CONNECTABLE:
          case BT_ADVERTISING:
          case BT_PAIRED:
            return 0;       /* already visible / paired */
        }
    }
  else
    {
      switch (g_bt_state)
        {
          case BT_ADVERTISING:
          case BT_CONNECTABLE:
            bt_enter_off();
            return 0;
          case BT_PAIRED:
            if (g_rfcomm_cid != 0)
              {
                rfcomm_disconnect(g_rfcomm_cid);
              }
            bt_enter_off();
            return 0;
          case BT_OFF:
          case BT_FAIL_BLINK:
            return 0;       /* already silent */
        }
    }

  return 0;
}

/* Button events.  Short press = "turn BT on" from any off-ish state.
 * Long press = "toggle" (on if currently off, off if currently on).
 */

static void on_button_short(void)
{
  if (g_bt_state == BT_OFF || g_bt_state == BT_FAIL_BLINK)
    {
      bt_visibility_request(true);
    }
}

static void on_button_long(void)
{
  if (g_bt_state == BT_OFF || g_bt_state == BT_FAIL_BLINK)
    {
      /* Long-press from OFF / FAIL_BLINK is the "force discoverable"
       * escape hatch: bt_visibility_request(true) is link-key-aware
       * and would land in BT_CONNECTABLE when a previously-paired peer
       * is in the DB, making it impossible to add a new PC without a
       * reboot.  Calling bt_enter_advertising() directly forces
       * discoverable=1 regardless (Issue #71).
       */

      bt_enter_advertising();
    }
  else
    {
      bt_visibility_request(false);
    }
}

/****************************************************************************
 * Public Functions — pairing / link bridge
 ****************************************************************************/

void btsensor_on_pairing_complete(uint8_t status)
{
  if (g_ts_state != TS_RUNNING)
    {
      return;
    }

  if (status != 0)
    {
      bt_enter_fail_blink();
      return;
    }

  /* Issue #56 spec: "ペアリング成功で BT LED 点灯".  Flip to BT_PAIRED
   * on SSP completion so the LED goes solid blue immediately, even
   * before the PC opens the RFCOMM channel.  If RFCOMM is opened
   * later, btsensor_set_rfcomm_cid() observes us already in BT_PAIRED
   * and is a no-op for the LED state machine.  BT_CONNECTABLE is
   * included so a peer that re-pairs (e.g. after PC-side link key
   * removal) while we were waiting in connectable mode still drives
   * the LED solid on SSP success rather than only on RFCOMM open.
   */

  if (g_bt_state == BT_ADVERTISING || g_bt_state == BT_CONNECTABLE)
    {
      bt_enter_paired();
    }
}

/****************************************************************************
 * Private Functions — packet handler
 ****************************************************************************/

static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size)
{
  (void)channel;
  (void)size;

  if (packet_type != HCI_EVENT_PACKET)
    {
      return;
    }

  switch (hci_event_packet_get_type(packet))
    {
      case BTSTACK_EVENT_STATE:
        {
          uint8_t state = btstack_event_state_get_state(packet);
          if (state == HCI_STATE_WORKING)
            {
              bd_addr_t addr;
              gap_local_bd_addr(addr);
              syslog(LOG_INFO,
                     "btsensor: HCI working, BD_ADDR %s — adv off "
                     "(\"%s\" hidden until BT button)\n",
                     bd_addr_to_str(addr), BTSENSOR_SPP_LOCAL_NAME);
            }
          else if (state == HCI_STATE_OFF &&
                   g_ts_state == TS_HCI_OFF_PENDING)
            {
              teardown_finish();
            }
        }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Private Functions — daemon entry
 ****************************************************************************/

static int btsensor_daemon(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  syslog(LOG_INFO, "btsensor: bringing up btstack on /dev/ttyBT\n");

  g_ts_state    = TS_RUNNING;
  g_rfcomm_cid  = 0;

  /* 0. Hard-reset the CC2564C via nSHUTD.  Without this the second
   * `btsensor start` after a `stop` silently fails to reach
   * HCI_STATE_WORKING because the chip retains its post-init-script
   * state from the previous session.  Harmless on the very first
   * start since the cold-boot pulse has already settled.
   * BUILD_PROTECTED keeps board code out of the user image, so we
   * reach the chip-reset GPIO sequence through a board-private ioctl.
   */

  {
    int btfd = open(BOARD_BTUART_DEVPATH, O_RDWR);
    if (btfd < 0)
      {
        syslog(LOG_WARNING, "btsensor: chip reset open(%s) errno %d\n",
               BOARD_BTUART_DEVPATH, errno);
      }
    else
      {
        if (ioctl(btfd, BTUART_IOC_CHIPRESET, 0) < 0)
          {
            syslog(LOG_WARNING, "btsensor: BTUART_IOC_CHIPRESET errno %d\n",
                   errno);
          }
        close(btfd);
      }
  }

  /* 1. Run loop. */

  btstack_memory_init();
  btstack_run_loop_init(btstack_run_loop_nuttx_get_instance());

  /* 2. HCI H4 transport on top of our NuttX UART wrapper. */

  const hci_transport_t *transport =
      hci_transport_h4_instance_for_uart(btstack_uart_nuttx_instance());

  hci_init(transport, &g_hci_transport_config);

  /* In-memory link key DB. */

  hci_set_link_key_db(btstack_link_key_db_memory_instance());

  /* 3. CC2564C chipset helper. */

  hci_set_chipset(btstack_chipset_cc256x_instance());

  /* 4. Hook BTSTACK_EVENT_STATE for the WORKING banner and HCI_STATE_OFF
   * during teardown.
   */

  g_hci_event_reg.callback = &packet_handler;
  hci_add_event_handler(&g_hci_event_reg);

  /* 5. SPP stack — registers SDP record + RFCOMM service handler.  Does
   * NOT enable advertising (the BT button in Commit C will).
   */

  spp_server_init();

  /* 6. TX arbiter + samplers + bundle emitter.  Order matters: the
   * emitter expects the samplers to be initialised before it is.  All
   * three start in OFF state — `IMU ON` / `SENSOR ON` (NSH or RFCOMM)
   * arms the 100 Hz timer.
   */

  btsensor_tx_init();
  btsensor_cmd_init();

  if (imu_sampler_init() != 0)
    {
      syslog(LOG_ERR, "btsensor: imu sampler init failed, "
                      "streaming disabled\n");
    }

  if (sensor_sampler_init() != 0)
    {
      syslog(LOG_ERR, "btsensor: sensor sampler init failed, "
                      "LEGO streaming disabled\n");
    }

  if (bundle_emitter_init() != 0)
    {
      syslog(LOG_ERR, "btsensor: bundle emitter init failed\n");
    }

  /* 7. BT button + LED.  Wire callbacks before init so the IRQ does
   * not arrive ahead of the dispatch table.
   */

  btsensor_button_set_callbacks(on_button_short, on_button_long);
  if (btsensor_button_init() != 0)
    {
      syslog(LOG_WARNING, "btsensor: button init failed — BT will only "
                          "be controllable from NSH\n");
    }

  btsensor_led_init();
  g_bt_state = BT_OFF;
  btsensor_led_off();

  /* 7. Power HCI on (radio active, but adv/connectable still off) and
   * enter the run loop.  cmd_stop() posts teardown_kick which walks
   * the FSM until btstack_run_loop_trigger_exit() returns us here.
   */

  hci_power_control(HCI_POWER_ON);
  btstack_run_loop_execute();

  /* Run loop exited.  Tear down the HCI layer so the next `btsensor
   * start` (which calls hci_init() again on the same global state) gets
   * a clean slate; without this, restart sometimes silently fails to
   * reach HCI_STATE_WORKING because leftover handlers / timers from
   * the previous session are still on btstack's linked lists.
   */

  hci_close();

  /* Mark daemon stopped under the lifecycle lock so a concurrent
   * `btsensor start` sees the cleared state.
   */

  pthread_mutex_lock(&g_lifecycle_lock);
  g_running     = false;
  g_daemon_pid  = -1;
  g_ts_state    = TS_RUNNING;
  pthread_mutex_unlock(&g_lifecycle_lock);

  syslog(LOG_INFO, "btsensor: daemon stopped\n");
  return 0;
}

/****************************************************************************
 * Public Functions — RFCOMM bridge for spp_packet_handler
 ****************************************************************************/

void btsensor_set_rfcomm_cid(uint16_t cid)
{
  uint16_t prev = g_rfcomm_cid;
  g_rfcomm_cid = cid;

  /* Drive the BT state machine on link up / down (only while the
   * teardown FSM is idle — once stop is requested the LED + GAP are
   * already being torn down by the daemon).
   */

  if (g_ts_state == TS_RUNNING)
    {
      if (cid != 0 &&
          (g_bt_state == BT_ADVERTISING || g_bt_state == BT_CONNECTABLE))
        {
          /* Either fresh-pair-then-RFCOMM-open (BT_ADVERTISING) or a
           * silent reconnect by a peer that still has the stored link
           * key (BT_CONNECTABLE — Issue #68).  Either way, RFCOMM
           * coming up means the session is live; flip to BT_PAIRED so
           * the LED goes solid blue and `btsensor status` reflects
           * the active stream.
           */

          bt_enter_paired();
        }
      else if (cid == 0 && g_bt_state == BT_PAIRED)
        {
          /* Link drop after a successful session: stay connectable so
           * the same paired peer can come back via its stored BD_ADDR
           * + link key, but stop advertising in inquiry — strangers
           * should not be able to opportunistically discover the Hub
           * just because the previous PC went away.  The LED resumes
           * blinking to communicate "waiting for reconnect".  Promotion
           * back to fully discoverable BT_ADVERTISING is opt-in via
           * the button / `btsensor bt on` (Issue #68).
           */

          bt_enter_connectable();
        }
    }

  /* If teardown is waiting for the RFCOMM channel to close, the cid=0
   * notification is our cue to advance to the next state.
   */

  if (cid == 0 && g_ts_state == TS_RFCOMM_CLOSING)
    {
      teardown_rfcomm_closed();
    }

  (void)prev;
}

/****************************************************************************
 * Private Functions — NSH -> main-thread action dispatch
 ****************************************************************************/

/* Cross-thread action dispatch.  NSH-side commands (bt, imu, set) post
 * a `btsensor_action_s` to the BTstack main thread via
 * execute_on_main_thread() so the BT state machine and the IMU
 * sampler stay single-threaded.  The NSH caller blocks on the
 * action's semaphore (with a 3 s timeout to match the teardown
 * watchdog) and reports the action's return value.
 */

enum btsensor_action_kind
{
  ACTION_BT_ON = 0,
  ACTION_BT_OFF,
  ACTION_IMU_ON,
  ACTION_IMU_OFF,
  ACTION_SENSOR_ON,
  ACTION_SENSOR_OFF,
  ACTION_SET_ODR,
  ACTION_SET_ACCEL_FSR,
  ACTION_SET_GYRO_FSR,
  ACTION_UNPAIR_ALL,
};

/* Heap-allocated action struct with explicit ownership transfer
 * between the waiter (NSH context) and the runner (BTstack main
 * thread).  The struct outlives the dispatch_action() stack frame so
 * a timed-out waiter cannot leave the runner with a dangling pointer
 * — whichever side discovers the other has already finished frees
 * the struct.
 */

struct btsensor_action_s
{
  enum btsensor_action_kind kind;
  uint32_t                  arg;
  int                       rc;
  sem_t                     done;
  pthread_mutex_t           lock;
  bool                      runner_done;
  bool                      waiter_gave_up;
  btstack_context_callback_registration_t reg;
};

static void action_free(struct btsensor_action_s *a)
{
  sem_destroy(&a->done);
  pthread_mutex_destroy(&a->lock);
  free(a);
}

static void action_runner(void *ctx)
{
  struct btsensor_action_s *a = (struct btsensor_action_s *)ctx;

  switch (a->kind)
    {
      case ACTION_BT_ON:
        a->rc = bt_visibility_request(true);
        break;
      case ACTION_BT_OFF:
        a->rc = bt_visibility_request(false);
        break;
      case ACTION_IMU_ON:
        a->rc = bundle_emitter_set_imu_enabled(true);
        break;
      case ACTION_IMU_OFF:
        a->rc = bundle_emitter_set_imu_enabled(false);
        break;
      case ACTION_SENSOR_ON:
        a->rc = bundle_emitter_set_sensor_enabled(true);
        break;
      case ACTION_SENSOR_OFF:
        a->rc = bundle_emitter_set_sensor_enabled(false);
        break;
      case ACTION_SET_ODR:
        a->rc = imu_sampler_set_odr_hz(a->arg);
        break;
      case ACTION_SET_ACCEL_FSR:
        a->rc = imu_sampler_set_accel_fsr(a->arg);
        break;
      case ACTION_SET_GYRO_FSR:
        a->rc = imu_sampler_set_gyro_fsr(a->arg);
        break;
      case ACTION_UNPAIR_ALL:
        /* gap_delete_all_link_keys() walks the link-key DB and drops
         * every entry via gap_drop_link_key_for_bd_addr().  No HCI
         * round-trip is required for the in-RAM DB, but it must run on
         * the BTstack main thread for consistency with other DB
         * mutations and to keep iterator state coherent if the daemon
         * later switches to a flash-backed DB (Issue #72).
         */

        gap_delete_all_link_keys();
        a->rc = 0;
        break;
      default:
        a->rc = -ENOSYS;
        break;
    }

  pthread_mutex_lock(&a->lock);
  a->runner_done    = true;
  bool waiter_gone  = a->waiter_gave_up;
  pthread_mutex_unlock(&a->lock);

  if (waiter_gone)
    {
      action_free(a);                  /* waiter timed out — clean up */
    }
  else
    {
      sem_post(&a->done);              /* hand result back to waiter */
    }
}

static int dispatch_action(enum btsensor_action_kind kind, uint32_t arg)
{
  pthread_mutex_lock(&g_lifecycle_lock);
  bool running = g_daemon_pid > 0;
  pthread_mutex_unlock(&g_lifecycle_lock);
  if (!running)
    {
      return -ENOTCONN;
    }

  struct btsensor_action_s *a = calloc(1, sizeof(*a));
  if (a == NULL)
    {
      return -ENOMEM;
    }

  a->kind = kind;
  a->arg  = arg;
  a->rc   = -ETIMEDOUT;
  pthread_mutex_init(&a->lock, NULL);
  sem_init(&a->done, 0, 0);
  a->reg.callback = action_runner;
  a->reg.context  = a;

  btstack_run_loop_execute_on_main_thread(&a->reg);

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 3;
  int wait_rc = sem_timedwait(&a->done, &ts);

  pthread_mutex_lock(&a->lock);
  bool runner_done = a->runner_done;
  if (!runner_done)
    {
      a->waiter_gave_up = true;
    }

  int result = (wait_rc < 0) ? -ETIMEDOUT : a->rc;
  pthread_mutex_unlock(&a->lock);

  if (runner_done)
    {
      action_free(a);                  /* runner finished — we own it */
    }
  /* else: runner will free when it finishes (may be after we return) */

  return result;
}

static void print_action_result(int rc)
{
  if (rc == 0)
    {
      printf("OK\n");
    }
  else if (rc == -EBUSY)
    {
      printf("ERR busy\n");
    }
  else if (rc == -EINVAL)
    {
      printf("ERR invalid value\n");
    }
  else if (rc == -ENOTCONN)
    {
      printf("btsensor: not running\n");
    }
  else if (rc == -ENXIO)
    {
      printf("btsensor: daemon is stopping\n");
    }
  else if (rc == -ETIMEDOUT)
    {
      printf("btsensor: action timed out\n");
    }
  else
    {
      printf("btsensor: rc=%d\n", rc);
    }
}

/****************************************************************************
 * Private Functions — NSH builtins
 ****************************************************************************/

static int cmd_start(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  pthread_mutex_lock(&g_lifecycle_lock);
  if (g_daemon_pid > 0)
    {
      pthread_mutex_unlock(&g_lifecycle_lock);
      printf("btsensor: already running (pid %d)\n", g_daemon_pid);
      return 1;
    }

  g_running = true;
  int pid = task_create(BTSENSOR_DAEMON_NAME, BTSENSOR_DAEMON_PRIORITY,
                        BTSENSOR_DAEMON_STACKSIZE, btsensor_daemon, NULL);
  if (pid < 0)
    {
      g_running = false;
      pthread_mutex_unlock(&g_lifecycle_lock);
      printf("btsensor: task_create failed\n");
      return 1;
    }

  g_daemon_pid = pid;
  pthread_mutex_unlock(&g_lifecycle_lock);
  printf("btsensor: started (pid %d)\n", pid);
  return 0;
}

static int cmd_stop(void)
{
  pthread_mutex_lock(&g_lifecycle_lock);
  if (g_daemon_pid <= 0)
    {
      pthread_mutex_unlock(&g_lifecycle_lock);
      printf("btsensor: not running\n");
      return 1;
    }

  pthread_mutex_unlock(&g_lifecycle_lock);

  /* Schedule the teardown kick on the run loop thread.  The static
   * registration struct is reused on every stop; safe because the run
   * loop drains the callback list each iteration.
   */

  g_teardown_kick_reg.callback = on_teardown_kick;
  g_teardown_kick_reg.context  = NULL;
  btstack_run_loop_execute_on_main_thread(&g_teardown_kick_reg);

  printf("btsensor: stop requested\n");
  return 0;
}

static int cmd_status(void)
{
  pthread_mutex_lock(&g_lifecycle_lock);
  bool running = g_daemon_pid > 0;
  int  pid     = g_daemon_pid;
  enum teardown_state ts = g_ts_state;
  pthread_mutex_unlock(&g_lifecycle_lock);

  uint16_t cid = g_rfcomm_cid;
  uint32_t sent;
  uint32_t dropped_oldest;
  uint32_t dropped_full;
  btsensor_tx_get_stats(&sent, &dropped_oldest, &dropped_full);

  printf("running:    %s%s\n", running ? "yes" : "no",
         running && ts != TS_RUNNING ? " (stopping)" : "");
  if (running)
    {
      printf("pid:        %d\n", pid);
      printf("bt:         %s\n", bt_state_name(g_bt_state));
      printf("imu:        %s\n",
             bundle_emitter_is_imu_enabled() ? "on" : "off");
      printf("sensor:     %s\n",
             bundle_emitter_is_sensor_enabled() ? "on" : "off");
      printf("config:     odr=%uHz accel_fsr=%ug gyro_fsr=%udps\n",
             (unsigned)imu_sampler_get_odr_hz(),
             (unsigned)imu_sampler_get_accel_fsr_g(),
             (unsigned)imu_sampler_get_gyro_fsr_dps());
    }

  printf("rfcomm cid: %u\n", (unsigned)cid);
  printf("frames:     sent=%u dropped_oldest=%u dropped_full=%u\n",
         (unsigned)sent, (unsigned)dropped_oldest,
         (unsigned)dropped_full);
  return 0;
}

static int cmd_bt(int argc, char **argv)
{
  if (argc < 3)
    {
      printf("Usage: btsensor bt <on|off>\n");
      return 1;
    }

  enum btsensor_action_kind kind;
  if (strcmp(argv[2], "on") == 0)
    {
      kind = ACTION_BT_ON;
    }
  else if (strcmp(argv[2], "off") == 0)
    {
      kind = ACTION_BT_OFF;
    }
  else
    {
      printf("btsensor: invalid bt arg '%s' (expected on|off)\n", argv[2]);
      return 1;
    }

  print_action_result(dispatch_action(kind, 0));
  return 0;
}

static int cmd_imu(int argc, char **argv)
{
  if (argc < 3)
    {
      printf("Usage: btsensor imu <on|off>\n");
      return 1;
    }

  enum btsensor_action_kind kind;
  if (strcmp(argv[2], "on") == 0)
    {
      kind = ACTION_IMU_ON;
    }
  else if (strcmp(argv[2], "off") == 0)
    {
      kind = ACTION_IMU_OFF;
    }
  else
    {
      printf("btsensor: invalid imu arg '%s' (expected on|off)\n",
             argv[2]);
      return 1;
    }

  print_action_result(dispatch_action(kind, 0));
  return 0;
}

static int cmd_sensor(int argc, char **argv)
{
  if (argc < 3)
    {
      printf("Usage: btsensor sensor <on|off>\n");
      return 1;
    }

  enum btsensor_action_kind kind;
  if (strcmp(argv[2], "on") == 0)
    {
      kind = ACTION_SENSOR_ON;
    }
  else if (strcmp(argv[2], "off") == 0)
    {
      kind = ACTION_SENSOR_OFF;
    }
  else
    {
      printf("btsensor: invalid sensor arg '%s' (expected on|off)\n",
             argv[2]);
      return 1;
    }

  print_action_result(dispatch_action(kind, 0));
  return 0;
}

static int cmd_dump(int argc, char **argv)
{
  /* Default 1 s if no duration given.  Independent of `btsensor start`
   * — opens the uORB topic directly so the kernel driver
   * auto-activates if no other subscriber is around (and stays active
   * if one is).  We never write to RFCOMM here; this is the local
   * "no PC" path the user asked for.
   */

  uint32_t duration_ms = 1000;
  if (argc >= 3)
    {
      long v = strtol(argv[2], NULL, 10);
      if (v < 0 || v > 60000)
        {
          printf("btsensor: invalid duration_ms (allowed 0..60000)\n");
          return 1;
        }

      duration_ms = (uint32_t)v;
    }

  int fd = open("/dev/uorb/sensor_imu0", O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("btsensor: open sensor_imu0 errno=%d\n", errno);
      return 1;
    }

  printf("# ts_us ax ay az gx gy gz (raw int16, Hub body frame)\n");

  struct timespec t0;
  clock_gettime(CLOCK_BOOTTIME, &t0);
  uint64_t deadline_us = (uint64_t)t0.tv_sec * 1000000ULL +
                         (uint64_t)t0.tv_nsec / 1000ULL +
                         (uint64_t)duration_ms * 1000ULL;

  uint32_t printed = 0;
  while (1)
    {
      struct timespec now;
      clock_gettime(CLOCK_BOOTTIME, &now);
      uint64_t now_us = (uint64_t)now.tv_sec * 1000000ULL +
                        (uint64_t)now.tv_nsec / 1000ULL;
      if (now_us >= deadline_us)
        {
          break;
        }

      int ms_left = (int)((deadline_us - now_us) / 1000ULL);
      if (ms_left <= 0)
        {
          break;
        }

      struct pollfd pfd = { .fd = fd, .events = POLLIN };
      int pret = poll(&pfd, 1, ms_left);
      if (pret <= 0)
        {
          continue;
        }

      struct sensor_imu s;
      while (read(fd, &s, sizeof(s)) == sizeof(s))
        {
          printf("%u %d %d %d %d %d %d\n",
                 (unsigned)s.timestamp,
                 s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
          printed++;
        }
    }

  close(fd);
  printf("# %u sample(s) over %u ms\n",
         (unsigned)printed, (unsigned)duration_ms);
  return 0;
}

static int cmd_unpair(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  print_action_result(dispatch_action(ACTION_UNPAIR_ALL, 0));
  return 0;
}

static int cmd_set(int argc, char **argv)
{
  if (argc < 4)
    {
      printf("Usage: btsensor set <odr|accel_fsr|gyro_fsr> <value>\n");
      return 1;
    }

  enum btsensor_action_kind kind;
  if (strcmp(argv[2], "odr") == 0)
    {
      kind = ACTION_SET_ODR;
    }
  else if (strcmp(argv[2], "accel_fsr") == 0)
    {
      kind = ACTION_SET_ACCEL_FSR;
    }
  else if (strcmp(argv[2], "gyro_fsr") == 0)
    {
      kind = ACTION_SET_GYRO_FSR;
    }
  else
    {
      printf("btsensor: invalid set field '%s'\n", argv[2]);
      return 1;
    }

  long val = strtol(argv[3], NULL, 10);
  if (val < 0)
    {
      printf("btsensor: invalid value\n");
      return 1;
    }

  print_action_result(dispatch_action(kind, (uint32_t)val));
  return 0;
}

static void print_usage(void)
{
  printf("Usage: btsensor <command>\n");
  printf("Commands:\n");
  printf("  start                       launch daemon (BT/IMU/SENSOR all off)\n");
  printf("  stop                        request teardown\n");
  printf("  status                      print full state + current config\n");
  printf("  bt     <on|off>             start/stop BT advertising (default off)\n");
  printf("  imu    <on|off>             start/stop IMU streaming (default off)\n");
  printf("  sensor <on|off>             start/stop LEGO sensor streaming (default off)\n");
  printf("  unpair                      forget all stored Bluetooth pairings\n");
  printf("  dump  [ms]                  dump raw IMU samples for ms (default 1000)\n");
  printf("  set   odr        <hz>       ODR  13|26|52|104|208|416|833 (default 833, capped)\n");
  printf("  set   accel_fsr  <g>        accel FSR 2|4|8|16 (default 8)\n");
  printf("  set   gyro_fsr   <dps>      gyro FSR  125|250|500|1000|2000 (default 2000)\n");
  printf("Note: bt/imu/sensor/dump/set require `start` first (dump can\n");
  printf("      auto-activate the driver standalone).  set * are\n");
  printf("      rejected with `ERR busy` while imu is on.\n");
  printf("      ODR > 833 is rejected to keep the BUNDLE 100 Hz tick in budget.\n");
}

/****************************************************************************
 * Entry Point
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      print_usage();
      return 1;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      return cmd_start(argc, argv);
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      return cmd_stop();
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return cmd_status();
    }

  if (strcmp(argv[1], "bt") == 0)
    {
      return cmd_bt(argc, argv);
    }

  if (strcmp(argv[1], "imu") == 0)
    {
      return cmd_imu(argc, argv);
    }

  if (strcmp(argv[1], "sensor") == 0)
    {
      return cmd_sensor(argc, argv);
    }

  if (strcmp(argv[1], "set") == 0)
    {
      return cmd_set(argc, argv);
    }

  if (strcmp(argv[1], "unpair") == 0)
    {
      return cmd_unpair(argc, argv);
    }

  if (strcmp(argv[1], "dump") == 0)
    {
      return cmd_dump(argc, argv);
    }

  print_usage();
  return 1;
}
