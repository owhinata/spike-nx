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

#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

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
#include "btsensor_led.h"
#include "btsensor_spp.h"
#include "btsensor_tx.h"
#include "imu_sampler.h"

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

/* Initial argv[1] (batch override) is consumed by the daemon entry. */

static uint8_t g_pending_batch;

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

static void bt_enter_fail_blink(void)
{
  syslog(LOG_WARNING, "btsensor: BT pairing failed (was %s)\n",
         bt_state_name(g_bt_state));
  g_bt_state = BT_FAIL_BLINK;
  gap_discoverable_control(0);
  gap_connectable_control(0);
  btsensor_led_fail_blink(CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS);
}

/* Button events: short = "turn BT on" from any off-ish state.  long =
 * "turn BT off" from any visible state, or also "turn BT on" from off
 * (treated identically to short for OFF/FAIL_BLINK so the user does
 * not have to discover the press-duration semantics for the on path).
 */

static void on_button_short(void)
{
  if (g_ts_state != TS_RUNNING)
    {
      return;
    }

  switch (g_bt_state)
    {
      case BT_OFF:
      case BT_FAIL_BLINK:
        bt_enter_advertising();
        break;
      default:
        break;        /* short press is a no-op while ADV / PAIRED */
    }
}

static void on_button_long(void)
{
  if (g_ts_state != TS_RUNNING)
    {
      return;
    }

  switch (g_bt_state)
    {
      case BT_OFF:
      case BT_FAIL_BLINK:
        bt_enter_advertising();
        break;
      case BT_ADVERTISING:
        bt_enter_off();
        break;
      case BT_PAIRED:
        if (g_rfcomm_cid != 0)
          {
            rfcomm_disconnect(g_rfcomm_cid);
          }
        bt_enter_off();
        break;
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
    }

  /* Success is reflected when RFCOMM_EVENT_CHANNEL_OPENED arrives via
   * btsensor_set_rfcomm_cid(); SSP completion alone doesn't mean the
   * SPP service is up yet.
   */
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

  if (g_pending_batch > 0)
    {
      imu_sampler_configure(g_pending_batch);
      syslog(LOG_INFO, "btsensor: configure batch=%u\n",
             (unsigned)g_pending_batch);
    }

  g_ts_state    = TS_RUNNING;
  g_rfcomm_cid  = 0;

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

  /* 6. TX arbiter + IMU sampler.  imu_sampler_init() opens
   * /dev/uorb/sensor_imu0 and registers it as a btstack data source;
   * frames flow into btsensor_tx whenever a sample lands.
   */

  btsensor_tx_init();

  if (imu_sampler_init() != 0)
    {
      syslog(LOG_ERR, "btsensor: imu sampler init failed, "
                      "streaming disabled\n");
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
      if (cid != 0 && g_bt_state == BT_ADVERTISING)
        {
          bt_enter_paired();
        }
      else if (cid == 0 && g_bt_state == BT_PAIRED)
        {
          /* Link drop while we still want to be reachable — go back to
           * advertising (LED resumes blinking).
           */

          bt_enter_advertising();
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
 * Private Functions — NSH builtins
 ****************************************************************************/

static int cmd_start(int argc, char **argv)
{
  pthread_mutex_lock(&g_lifecycle_lock);
  if (g_daemon_pid > 0)
    {
      pthread_mutex_unlock(&g_lifecycle_lock);
      printf("btsensor: already running (pid %d)\n", g_daemon_pid);
      return 1;
    }

  g_pending_batch = 0;
  if (argc >= 3)
    {
      int batch = atoi(argv[2]);
      if (batch < 1 || batch > 80)
        {
          pthread_mutex_unlock(&g_lifecycle_lock);
          printf("btsensor: invalid batch=%d (allowed 1..80)\n", batch);
          return 1;
        }

      g_pending_batch = (uint8_t)batch;
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
  uint32_t dropped;
  btsensor_tx_get_stats(&sent, &dropped);

  printf("running:    %s%s\n", running ? "yes" : "no",
         running && ts != TS_RUNNING ? " (stopping)" : "");
  if (running)
    {
      printf("pid:        %d\n", pid);
      printf("bt:         %s\n", bt_state_name(g_bt_state));
    }

  printf("rfcomm cid: %u\n", (unsigned)cid);
  printf("frames:     sent=%u dropped=%u\n",
         (unsigned)sent, (unsigned)dropped);
  return 0;
}

static void print_usage(void)
{
  printf("Usage: btsensor <command>\n");
  printf("Commands:\n");
  printf("  start [batch]  - launch daemon (default BT adv off)\n");
  printf("  stop           - request teardown\n");
  printf("  status         - print state\n");
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

  print_usage();
  return 1;
}
