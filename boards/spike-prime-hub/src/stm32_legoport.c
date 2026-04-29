/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport.c
 *
 * I/O port device-connection-manager (DCM) for SPIKE Prime Hub.
 *
 * Each of the 6 external ports (A-F) carries 5 signals (gpio1, gpio2,
 * uart_tx, uart_rx, uart_buf) used in two roles:
 *
 *   - GPIO mode (this file): the resistor-divider passive-detection scheme
 *     ported from pybricks `lib/pbio/drv/legodev/legodev_pup.c`, run from
 *     HPWORK at 2 ms cadence per yield boundary.
 *   - UART mode (Issue #43 LUMP engine): once a port is confirmed
 *     `UNKNOWN_UART (14)`, ownership is transferred to a registered
 *     handoff callback that switches the pins to UART AF.
 *
 * Detection is debounced over 20 consecutive identical scans (~400 ms
 * worst case for the longest scan path) before `confirmed_type` is updated
 * and `state_change_sem` posted to wake any pending WAIT_CONNECT /
 * WAIT_DISCONNECT ioctl.
 *
 * The DCM state machine is a faithful port of pybricks `poll_dcm()` —
 * each of the 11 `PT_YIELD(pt)` calls in pybricks corresponds to a real
 * yield boundary here, where the worker function returns and is
 * rescheduled in 2 ms.  Two `_WAIT` states (`S4_ID1_SETTLE_WAIT`,
 * `S8_PRE_FINAL_WAIT`) are no-op ticks that exist solely to honour the
 * unconditional yields at pybricks lines 199 and 278; they MUST NOT be
 * optimised away or the timing diverges from the reference.
 *
 * See `docs/{ja,en}/drivers/port-detection.md` for the full design.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board_legoport.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_LEGO_PORT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Tick cadence — pybricks uses 2 ms between protothread yields, we mirror. */

#define LEGOPORT_TICK_MS              2

/* Debounce: 20 consecutive identical scans confirms a type change.
 * Verbatim from pybricks `AFFIRMATIVE_MATCH_COUNT`.
 */

#define LEGOPORT_AFFIRMATIVE_MATCH    20

/* UNOWNED gated re-scan: how long to idle between full scans, and how
 * many consecutive non-UNKNOWN_UART confirmations are needed to release
 * the latch.  See `DCM_LATCHED_UART_UNOWNED_IDLE` handler.
 */

#define LEGOPORT_UNOWNED_PROBE_MS     100
#define LEGOPORT_UNOWNED_DISCONNECT_K 3

/* dev_id_group classifications (pybricks `dev_id_group_t`) */

#define DEV_ID_GROUP_GND        0
#define DEV_ID_GROUP_VCC        1
#define DEV_ID_GROUP_PULL_DOWN  2
#define DEV_ID_GROUP_OPEN       3

/* Per-port GPIO pin descriptor builder for the table.  All states share
 * the same port/pin and mode flavour (no pull, push-pull); only the
 * mode/level bits differ.  Speed is 50MHz for normal pins and 2MHz for
 * the backup-domain Port D pins (PC14, PC15, PC13, PC11) per RM0430.
 *
 * GPIO_INPUT must be combined with GPIO_FLOAT (PUPDR=00) so the resistor
 * divider sees the bare pin without the internal ~40k pull biasing it.
 */

#define LP_INPUT(speed, port, pin)                                         \
  (GPIO_INPUT | GPIO_FLOAT | (speed) | (port) | (pin))

#define LP_OUT_HI(speed, port, pin)                                        \
  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | (speed) | (port) | (pin))

#define LP_OUT_LO(speed, port, pin)                                        \
  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR | (speed) | (port) | (pin))

#define LP_AF(af, speed, port, pin)                                        \
  (GPIO_ALT | GPIO_PUSHPULL | (af) | (speed) | (port) | (pin))

/* Convenience for backup-domain pins (PC13/14/15) — must use 2MHz drive. */

#define LP_INPUT_BD(port, pin)  LP_INPUT(GPIO_SPEED_2MHz, port, pin)
#define LP_OUT_HI_BD(port, pin) LP_OUT_HI(GPIO_SPEED_2MHz, port, pin)
#define LP_OUT_LO_BD(port, pin) LP_OUT_LO(GPIO_SPEED_2MHz, port, pin)

/* Standard speed pins */

#define LP_INPUT_S(port, pin)   LP_INPUT(GPIO_SPEED_50MHz, port, pin)
#define LP_OUT_HI_S(port, pin)  LP_OUT_HI(GPIO_SPEED_50MHz, port, pin)
#define LP_OUT_LO_S(port, pin)  LP_OUT_LO(GPIO_SPEED_50MHz, port, pin)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DCM state machine — each enumerator either yields (return + reschedule)
 * or falls through (continue) to the next state.  See file-level comment
 * for the pybricks line correspondence.
 */

enum dcm_state_e
{
  /* Active scan states (pybricks `poll_dcm()`) */

  DCM_S0_INIT = 0,            /* yield: pybricks 130 */
  DCM_S1_READ_ID2_A,          /* yield: pybricks 138 */
  DCM_S2_READ_ID2_B,          /* fallthrough: branches to S2A / S8_PRE / S3 */
  DCM_S2A_TOUCH_SETTLE,       /* yield: pybricks 152 */
  DCM_S2B_TOUCH_SAMPLE,       /* fallthrough: → S8_PRE */
  DCM_S3_READ_ID1_A,          /* yield: pybricks 168 */
  DCM_S3_READ_ID1_B,          /* fallthrough: branches to S4 or S4_WAIT */
  DCM_S4_OPEN_PD,             /* yield: pybricks 187 (only on else branch) */
  DCM_S4_ID1_SETTLE_WAIT,     /* yield: pybricks 199 (UNCONDITIONAL) */
  DCM_S5_DRIVE_ID2_HI,        /* yield: pybricks 208 */
  DCM_S6_READ_ID1_C,          /* yield: pybricks 216 */
  DCM_S6_READ_ID1_D,          /* fallthrough: branches to S8_PRE / S7 */
  DCM_S7_PROBE_RX,            /* yield: pybricks 241 */
  DCM_S7_READ_RX_HI,          /* yield: pybricks 255 (only on rx==1 branch) */
  DCM_S7B_READ_RX_LO,         /* fallthrough: → S8_PRE */
  DCM_S8_PRE_FINAL_WAIT,      /* yield: pybricks 278 (UNCONDITIONAL) */
  DCM_S8_FINALIZE,            /* fallthrough: → S0 (or LATCHED_*) */

  /* Latched states (DCM paused for the port) */

  DCM_LATCHED_UART_OWNED,            /* #43 owns the pins; we do nothing */
  DCM_LATCHED_UART_UNOWNED_IDLE,     /* gated re-scan; transitions to S0 on timer */
};

struct legoport_state_s
{
  /* DCM scratch + result */

  uint8_t  state;                /* enum dcm_state_e */
  uint8_t  type_id;              /* candidate this scan */
  uint8_t  prev_type_id;         /* candidate previous scan */
  uint8_t  confirmed_type;       /* confirmed (debounced) — published value */
  uint8_t  id1_group;            /* DEV_ID_GROUP_* during scan */
  uint8_t  match_count;          /* 0..AFFIRMATIVE_MATCH */
  uint8_t  gpio_value;           /* sample scratch */
  uint8_t  prev_gpio_value;      /* sample scratch */

  /* UNOWNED re-scan state */

  uint8_t  unowned_disc_streak;  /* consecutive NONE confirmations */
  uint8_t  unowned_other_streak; /* consecutive non-UNKNOWN_UART, non-NONE */
  uint8_t  unowned_other_type;   /* type seen across the streak */
  bool     in_unowned_rescan;    /* set when entering S0 from UNOWNED idle */
  clock_t  unowned_idle_until;   /* ticks: gate expiry */

  /* Handoff registry (per-port).  Layered protections:
   *  - handoff_generation: stale-result guard
   *  - handoff_inflight + handoff_quiescent: priv lifetime guard
   *  - handoff_waiters: optimization to skip stale posts (Codex polish #3)
   */

  legoport_uart_handoff_cb_t handoff_cb;
  void                      *handoff_priv;
  uint32_t                   handoff_generation;
  uint32_t                   handoff_inflight;
  uint32_t                   handoff_waiters;
  sem_t                      handoff_quiescent;

  /* Edge notification for WAIT_CONNECT / WAIT_DISCONNECT / poll().  The
   * counter increments whenever `confirmed_type` changes; clients use the
   * delta against a snapshot to avoid missed-edge races.
   */

  uint32_t event_counter;
  sem_t    state_change_sem;

  mutex_t  lock;                 /* serializes ioctl vs HPWORK update */

  /* Flags published in `legoport_info_s::flags` */

  uint8_t  flags;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 3x3 device-type lookup — verbatim port of `legodev_pup_type_id_lookup`
 * (pybricks legodev_pup.c:83-99).  Indexed by [id1_group][id2_group] where
 * group ∈ {GND=0, VCC=1, PULL_DOWN=2}.  ID1=OPEN is handled as the
 * UNKNOWN_UART fallback in S7_READ_RX_HI.
 */

static const uint8_t g_type_lookup[3][3] =
{
  /* ID1 = GND */
  [DEV_ID_GROUP_GND] = {
    [DEV_ID_GROUP_GND]       = LEGOPORT_TYPE_LPF2_POWER,
    [DEV_ID_GROUP_VCC]       = LEGOPORT_TYPE_LPF2_TURN,
    [DEV_ID_GROUP_PULL_DOWN] = LEGOPORT_TYPE_LPF2_LIGHT2,
  },
  /* ID1 = VCC */
  [DEV_ID_GROUP_VCC] = {
    [DEV_ID_GROUP_GND]       = LEGOPORT_TYPE_LPF2_TRAIN,
    [DEV_ID_GROUP_VCC]       = LEGOPORT_TYPE_LPF2_LMOTOR,
    [DEV_ID_GROUP_PULL_DOWN] = LEGOPORT_TYPE_LPF2_LIGHT1,
  },
  /* ID1 = PULL_DOWN */
  [DEV_ID_GROUP_PULL_DOWN] = {
    [DEV_ID_GROUP_GND]       = LEGOPORT_TYPE_LPF2_MMOTOR,
    [DEV_ID_GROUP_VCC]       = LEGOPORT_TYPE_LPF2_XMOTOR,
    [DEV_ID_GROUP_PULL_DOWN] = LEGOPORT_TYPE_LPF2_LIGHT,
  },
};

/* Per-port pin table — one entry per A..F.  See
 * `docs/ja/hardware/pin-mapping.md` for the canonical mapping.
 *
 * Pin caveats encoded here:
 *   - Port D gpio1=PC15, gpio2=PC14: backup-domain I/O, GPIO_SPEED_2MHz only.
 *   - Port E gpio1=PC13:             backup-domain I/O, GPIO_SPEED_2MHz only.
 *   - Port F gpio1=PC11:             backup-domain I/O, GPIO_SPEED_2MHz only.
 *     RM0430 §6.2.4 ("PC13/PC14/PC15" wired to backup domain even with
 *     LSEON=0, but usable as GPIO when LSE is off — current path).
 *   - Port D uart_buf=PB2: also STM32 BOOT1 strap.  HW board must hold the
 *     correct level during reset; software cannot fix that.
 *   - Port A uart_buf=PA10: also STM32 OTG_FS_ID.  Defconfig sets
 *     CONFIG_OTG_ID_GPIO_DISABLE=y so NuttX upstream OTG init does not
 *     stm32_configgpio(GPIO_OTGFS_ID) and overwrite our setting.
 *
 * UART AFs (TX/RX): Port A/C/D = AF8 (UART7/UART8/UART5),
 *                   Port B/E/F = AF11 (UART4/UART10/UART9).
 * UART9 and UART10 do not currently have a NuttX serial driver — that is
 * #43's problem; #42 only stores the AF descriptors so #43 can install
 * them when it takes ownership.
 */

static const struct legoport_pin_s g_legoport_pins[BOARD_LEGOPORT_COUNT] =
{
  /* ----- Port A (UART7) ----- */
  [0] = {
    .gpio1_in     = LP_INPUT_S (GPIO_PORTD, GPIO_PIN7),    /* PD7  */
    .gpio2_in     = LP_INPUT_S (GPIO_PORTD, GPIO_PIN8),    /* PD8  */
    .gpio2_lo     = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN8),
    .gpio2_hi     = LP_OUT_HI_S(GPIO_PORTD, GPIO_PIN8),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN8),    /* PE8 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN8),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN8),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN7),    /* PE7 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN7),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTA, GPIO_PIN10),   /* PA10 */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTA, GPIO_PIN10),
    .uart_tx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN8),
    .uart_rx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN7),
    .port_index   = 0,
  },
  /* ----- Port B (UART4) ----- */
  [1] = {
    .gpio1_in     = LP_INPUT_S (GPIO_PORTD, GPIO_PIN9),    /* PD9  */
    .gpio2_in     = LP_INPUT_S (GPIO_PORTD, GPIO_PIN10),   /* PD10 */
    .gpio2_lo     = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN10),
    .gpio2_hi     = LP_OUT_HI_S(GPIO_PORTD, GPIO_PIN10),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTD, GPIO_PIN1),    /* PD1 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTD, GPIO_PIN1),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN1),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTD, GPIO_PIN0),    /* PD0 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN0),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTA, GPIO_PIN8),    /* PA8  */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTA, GPIO_PIN8),
    .uart_tx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTD, GPIO_PIN1),
    .uart_rx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTD, GPIO_PIN0),
    .port_index   = 1,
  },
  /* ----- Port C (UART8) ----- */
  [2] = {
    .gpio1_in     = LP_INPUT_S (GPIO_PORTD, GPIO_PIN11),   /* PD11 */
    .gpio2_in     = LP_INPUT_S (GPIO_PORTE, GPIO_PIN4),    /* PE4  */
    .gpio2_lo     = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN4),
    .gpio2_hi     = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN4),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN1),    /* PE1 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN1),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN1),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN0),    /* PE0 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN0),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN5),    /* PE5  */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN5),
    .uart_tx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN1),
    .uart_rx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN0),
    .port_index   = 2,
  },
  /* ----- Port D (UART5) — backup-domain pins for gpio1/gpio2 ----- */
  [3] = {
    .gpio1_in     = LP_INPUT_BD(GPIO_PORTC, GPIO_PIN15),   /* PC15 (BD) */
    .gpio2_in     = LP_INPUT_BD(GPIO_PORTC, GPIO_PIN14),   /* PC14 (BD) */
    .gpio2_lo     = LP_OUT_LO_BD(GPIO_PORTC, GPIO_PIN14),
    .gpio2_hi     = LP_OUT_HI_BD(GPIO_PORTC, GPIO_PIN14),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTC, GPIO_PIN12),   /* PC12 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTC, GPIO_PIN12),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTC, GPIO_PIN12),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTD, GPIO_PIN2),    /* PD2 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN2),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN2),    /* PB2 (BOOT1) */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN2),
    .uart_tx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTC, GPIO_PIN12),
    .uart_rx_af   = LP_AF(GPIO_AF8, GPIO_SPEED_50MHz, GPIO_PORTD, GPIO_PIN2),
    .port_index   = 3,
  },
  /* ----- Port E (UART10) — backup-domain pin for gpio1 ----- */
  [4] = {
    .gpio1_in     = LP_INPUT_BD(GPIO_PORTC, GPIO_PIN13),   /* PC13 (BD) */
    .gpio2_in     = LP_INPUT_S (GPIO_PORTE, GPIO_PIN12),   /* PE12 */
    .gpio2_lo     = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN12),
    .gpio2_hi     = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN12),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN3),    /* PE3 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN3),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN3),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTE, GPIO_PIN2),    /* PE2 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN2),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN5),    /* PB5  */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN5),
    .uart_tx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN3),
    .uart_rx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTE, GPIO_PIN2),
    .port_index   = 4,
  },
  /* ----- Port F (UART9) — backup-domain pin for gpio1 ----- */
  [5] = {
    .gpio1_in     = LP_INPUT_BD(GPIO_PORTC, GPIO_PIN11),   /* PC11 (BD) */
    .gpio2_in     = LP_INPUT_S (GPIO_PORTE, GPIO_PIN6),    /* PE6  */
    .gpio2_lo     = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN6),
    .gpio2_hi     = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN6),
    .uart_tx_in   = LP_INPUT_S (GPIO_PORTD, GPIO_PIN15),   /* PD15 (TX) */
    .uart_tx_hi   = LP_OUT_HI_S(GPIO_PORTD, GPIO_PIN15),
    .uart_tx_lo   = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN15),
    .uart_rx_in   = LP_INPUT_S (GPIO_PORTD, GPIO_PIN14),   /* PD14 (RX) */
    .uart_rx_lo   = LP_OUT_LO_S(GPIO_PORTD, GPIO_PIN14),
    .uart_buf_lo  = LP_OUT_LO_S(GPIO_PORTC, GPIO_PIN5),    /* PC5  */
    .uart_buf_hi  = LP_OUT_HI_S(GPIO_PORTC, GPIO_PIN5),
    .uart_tx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTD, GPIO_PIN15),
    .uart_rx_af   = LP_AF(GPIO_AF11, GPIO_SPEED_50MHz, GPIO_PORTD, GPIO_PIN14),
    .port_index   = 5,
  },
};

static struct legoport_state_s g_legoport_state[BOARD_LEGOPORT_COUNT];
static struct work_s            g_legoport_work;
static bool                     g_legoport_initialized;

/* HPWORK cadence monitoring — see `LEGOPORT_GET_STATS` ioctl. */

static volatile uint32_t g_max_step_us;
static volatile uint32_t g_max_interval_us;
static volatile uint32_t g_total_invocations;
static volatile uint32_t g_late_4ms;
static volatile uint32_t g_late_10ms;
static volatile uint32_t g_late_100ms;
static clock_t           g_last_invoke_ticks;

/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

static void legoport_dcm_step(int port);
static void legoport_dcm_worker(FAR void *arg);
static void legoport_post_change(struct legoport_state_s *s, uint8_t new_type);
static void legoport_invoke_handoff(int port);

/****************************************************************************
 * Helpers
 ****************************************************************************/

/* Microsecond timestamp helper.  USEC_PER_TICK is 10 in this build, so
 * the worst rounding is ±10 µs which is fine for the cadence stats.
 */

static inline uint32_t legoport_now_us(void)
{
  return (uint32_t)TICK2USEC(clock_systime_ticks());
}

/* Wrapper so debug printfs from the worker have a port label. */

#define LEGOPORT_PORT_CHAR(port)  ((char)('A' + (port)))

/* Update the published flags field after `confirmed_type` changes.  Caller
 * must hold `s->lock`.
 */

static void legoport_update_flags(struct legoport_state_s *s)
{
  uint8_t f = 0;

  if (s->confirmed_type != LEGOPORT_TYPE_NONE)
    {
      f |= LEGOPORT_FLAG_CONNECTED;
    }

  if (s->confirmed_type == LEGOPORT_TYPE_LPF2_UNKNOWN_UART)
    {
      f |= LEGOPORT_FLAG_IS_UART;
    }
  else if (s->confirmed_type != LEGOPORT_TYPE_NONE)
    {
      f |= LEGOPORT_FLAG_IS_PASSIVE;
    }

  if (s->state == DCM_LATCHED_UART_OWNED)
    {
      f |= LEGOPORT_FLAG_HANDOFF_OK;
    }

  s->flags = f;
}

/* Post a state-change edge: bump counter, post sem (only if waiters), and
 * publish flags.  Caller must hold `s->lock`.
 */

static void legoport_post_change(struct legoport_state_s *s, uint8_t new_type)
{
  if (s->confirmed_type == new_type)
    {
      return;
    }

  s->confirmed_type = new_type;
  s->event_counter++;
  legoport_update_flags(s);

  /* Wake every waiter without spamming posts beyond what's needed.  This
   * is the dual of the "rxsem" pattern in stm32_btuart_chardev.c.
   */

  int sval;
  if (nxsem_get_value(&s->state_change_sem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&s->state_change_sem);
    }
}

/****************************************************************************
 * Handoff invocation (lock-protected, with inflight + generation guards)
 ****************************************************************************/

static void legoport_invoke_handoff(int port)
{
  struct legoport_state_s *s = &g_legoport_state[port];
  legoport_uart_handoff_cb_t cb_local;
  void                     *priv_local;
  uint32_t                  gen_local;
  int                       ret = -ENOSYS;

  nxmutex_lock(&s->lock);
  cb_local   = s->handoff_cb;
  priv_local = s->handoff_priv;
  gen_local  = s->handoff_generation;
  if (cb_local != NULL)
    {
      s->handoff_inflight++;
    }

  nxmutex_unlock(&s->lock);

  if (cb_local != NULL)
    {
      ret = cb_local(port, &g_legoport_pins[port], priv_local);
    }

  nxmutex_lock(&s->lock);

  /* Only apply the result if no concurrent unregister happened. */

  if (cb_local != NULL && gen_local == s->handoff_generation)
    {
      if (ret == OK)
        {
          s->state  = DCM_LATCHED_UART_OWNED;
          s->flags |= LEGOPORT_FLAG_HANDOFF_OK;
        }
      else
        {
          /* Fall back to UNOWNED gated re-scan. */
          s->state                = DCM_LATCHED_UART_UNOWNED_IDLE;
          s->in_unowned_rescan    = false;
          s->unowned_disc_streak  = 0;
          s->unowned_other_streak = 0;
          s->unowned_other_type   = LEGOPORT_TYPE_NONE;
          s->unowned_idle_until   = clock_systime_ticks() +
                                    MSEC2TICK(LEGOPORT_UNOWNED_PROBE_MS);
        }
    }
  else if (cb_local == NULL)
    {
      /* No CB registered at the time of confirmation — go straight to
       * UNOWNED gated re-scan.
       */
      s->state                = DCM_LATCHED_UART_UNOWNED_IDLE;
      s->in_unowned_rescan    = false;
      s->unowned_disc_streak  = 0;
      s->unowned_other_streak = 0;
      s->unowned_other_type   = LEGOPORT_TYPE_NONE;
      s->unowned_idle_until   = clock_systime_ticks() +
                                MSEC2TICK(LEGOPORT_UNOWNED_PROBE_MS);
    }

  /* Always drop the inflight refcount and wake any drainers. */

  if (cb_local != NULL)
    {
      s->handoff_inflight--;
      if (s->handoff_inflight == 0 && s->handoff_waiters > 0)
        {
          nxsem_post(&s->handoff_quiescent);
        }
    }

  nxmutex_unlock(&s->lock);
}

/****************************************************************************
 * DCM step — one HPWORK invocation per port, fall through non-yield states
 ****************************************************************************/

static void legoport_dcm_step(int port)
{
#ifdef CONFIG_LEGO_PORT_DCM_NOOP
  /* Diagnostic build: HPWORK monitoring code runs but DCM does no GPIO
   * work.  Used to rule the DCM out of HPWORK contention measurements.
   */
  UNUSED(port);
  return;
#else
  struct legoport_state_s   *s = &g_legoport_state[port];
  const struct legoport_pin_s *p = &g_legoport_pins[port];

  for (; ; )
    {
      switch (s->state)
        {
          /* --- Latched: paused, no work --- */
          case DCM_LATCHED_UART_OWNED:
            return;

          case DCM_LATCHED_UART_UNOWNED_IDLE:
            if ((sclock_t)(clock_systime_ticks() - s->unowned_idle_until) < 0)
              {
                /* Gate not yet expired — return without consuming a tick
                 * for the state machine.
                 */
                return;
              }

            /* Re-arm scan.  Reset debounce so the rescan starts fresh. */

            s->state             = DCM_S0_INIT;
            s->in_unowned_rescan = true;
            s->match_count       = 0;
            s->prev_type_id      = LEGOPORT_TYPE_NONE;
            continue;  /* fall through into S0 */

          /* --- Active scan --- */

          case DCM_S0_INIT:
            /* pybricks 121-130: drive ID1 high (uart_tx high, uart_buf low),
             * ID2 input.  Then yield.
             */
            stm32_configgpio(p->uart_tx_hi);
            stm32_configgpio(p->uart_buf_lo);
            stm32_configgpio(p->gpio2_in);
            s->type_id      = LEGOPORT_TYPE_NONE;
            s->id1_group    = DEV_ID_GROUP_OPEN;
            s->state        = DCM_S1_READ_ID2_A;
            return;

          case DCM_S1_READ_ID2_A:
            /* pybricks 133-138: sample ID2 → prev, drive ID1 low, yield. */
            s->prev_gpio_value = stm32_gpioread(p->gpio2_in) ? 1 : 0;
            stm32_configgpio(p->uart_tx_lo);
            s->state        = DCM_S2_READ_ID2_B;
            return;

          case DCM_S2_READ_ID2_B:
            /* pybricks 140-167: sample ID2 → cur, branch on transition. */
            s->gpio_value = stm32_gpioread(p->gpio2_in) ? 1 : 0;

            if (s->prev_gpio_value == 1 && s->gpio_value == 0)
              {
                /* TOUCH (5) — pybricks 148-160. */
                s->type_id = LEGOPORT_TYPE_LPF2_TOUCH;
                stm32_configgpio(p->uart_buf_hi);
                stm32_configgpio(p->uart_tx_in);
                s->state   = DCM_S2A_TOUCH_SETTLE;
                return;  /* yield: pybricks 152 */
              }

            if (s->prev_gpio_value == 0 && s->gpio_value == 1)
              {
                /* TPOINT (11) — pybricks 162-163. */
                s->type_id = LEGOPORT_TYPE_LPF2_TPOINT;
                s->state   = DCM_S8_PRE_FINAL_WAIT;
                continue;  /* fallthrough into the unconditional final yield */
              }

            /* Else: continue with ID1 group probe. */

            s->state = DCM_S3_READ_ID1_A;
            continue;

          case DCM_S2A_TOUCH_SETTLE:
            /* No-op tick: pybricks 152 yield was here.  Fall through to
             * sample (which intentionally discards the analog value).
             */
            s->state = DCM_S2B_TOUCH_SAMPLE;
            continue;

          case DCM_S2B_TOUCH_SAMPLE:
            /* pybricks 158-160: read gpio1 to update analog touch value.
             * #42 does NOT expose this analog reading — type=TOUCH is
             * already latched.  Reading the pin and discarding the value
             * keeps the GPIO sequence identical to pybricks for any
             * cap-debounce behaviour the LEGO hardware relies on.
             */
            (void)stm32_gpioread(p->gpio1_in);
            s->state = DCM_S8_PRE_FINAL_WAIT;
            continue;

          case DCM_S3_READ_ID1_A:
            /* pybricks 164-168: sample gpio1 → prev, drive ID1 high, yield. */
            s->prev_gpio_value = stm32_gpioread(p->gpio1_in) ? 1 : 0;
            stm32_configgpio(p->uart_tx_hi);
            s->state           = DCM_S3_READ_ID1_B;
            return;

          case DCM_S3_READ_ID1_B:
            /* pybricks 170-197: sample gpio1 → cur, classify ID1 group. */
            s->gpio_value = stm32_gpioread(p->gpio1_in) ? 1 : 0;

            if (s->prev_gpio_value == 1 && s->gpio_value == 1)
              {
                s->id1_group = DEV_ID_GROUP_VCC;
                s->state     = DCM_S4_ID1_SETTLE_WAIT;
                continue;  /* fall through to the unconditional yield */
              }

            if (s->prev_gpio_value == 0 && s->gpio_value == 0)
              {
                s->id1_group = DEV_ID_GROUP_GND;
                s->state     = DCM_S4_ID1_SETTLE_WAIT;
                continue;
              }

            /* OPEN / PULL_DOWN slow path: switch ID1 to input + buf high,
             * yield, then sample again.
             */
            stm32_configgpio(p->uart_buf_hi);
            stm32_configgpio(p->uart_tx_in);
            s->state = DCM_S4_OPEN_PD;
            return;  /* yield: pybricks 187 */

          case DCM_S4_OPEN_PD:
            /* pybricks 189-196: gpio1==1 → OPEN, ==0 → PULL_DOWN. */
            s->id1_group = (stm32_gpioread(p->gpio1_in)
                            ? DEV_ID_GROUP_OPEN : DEV_ID_GROUP_PULL_DOWN);
            s->state     = DCM_S4_ID1_SETTLE_WAIT;
            continue;  /* fall through to unconditional yield */

          case DCM_S4_ID1_SETTLE_WAIT:
            /* pybricks 199 UNCONDITIONAL yield — settle before driving
             * ID2 high.  Real yield boundary: do not skip.
             */
            s->state = DCM_S5_DRIVE_ID2_HI;
            return;

          case DCM_S5_DRIVE_ID2_HI:
            /* pybricks 201-208: ID1 input + buf high, gpio2 high, yield. */
            stm32_configgpio(p->uart_buf_hi);
            stm32_configgpio(p->uart_tx_in);
            stm32_configgpio(p->gpio2_hi);
            s->state = DCM_S6_READ_ID1_C;
            return;

          case DCM_S6_READ_ID1_C:
            /* pybricks 210-216: sample gpio1 → prev, gpio2 low, yield. */
            s->prev_gpio_value = stm32_gpioread(p->gpio1_in) ? 1 : 0;
            stm32_configgpio(p->gpio2_lo);
            s->state           = DCM_S6_READ_ID1_D;
            return;

          case DCM_S6_READ_ID1_D:
            /* pybricks 218-275: sample gpio1 → cur, branch. */
            s->gpio_value = stm32_gpioread(p->gpio1_in) ? 1 : 0;

            if (s->prev_gpio_value == 1 && s->gpio_value == 0)
              {
                if (s->id1_group == DEV_ID_GROUP_OPEN)
                  {
                    s->type_id = LEGOPORT_TYPE_LPF2_3_PART;
                  }
                /* else: type_id stays NONE for this scan */
                s->state = DCM_S8_PRE_FINAL_WAIT;
                continue;
              }

            if (s->prev_gpio_value == 0 && s->gpio_value == 1)
              {
                s->type_id = LEGOPORT_TYPE_LPF2_EXPLOD;
                s->state   = DCM_S8_PRE_FINAL_WAIT;
                continue;
              }

            /* Else: probe uart_rx for ID2 group. */

            stm32_configgpio(p->uart_tx_hi);
            stm32_configgpio(p->uart_buf_lo);
            stm32_configgpio(p->gpio2_hi);
            s->state = DCM_S7_PROBE_RX;
            return;  /* yield: pybricks 241 */

          case DCM_S7_PROBE_RX:
            /* pybricks 243-274: sample uart_rx, branch. */
            if (stm32_gpioread(p->uart_rx_in))
              {
                /* uart_rx high — drive gpio2 low + uart_rx output low
                 * (cap-debounce trick from pybricks 246-253).
                 */
                stm32_configgpio(p->gpio2_lo);
                stm32_configgpio(p->uart_rx_lo);
                s->state = DCM_S7_READ_RX_HI;
                return;  /* yield: pybricks 255 */
              }
            else
              {
                /* uart_rx low — id2 == GND.  Lookup or UNKNOWN_UART. */
                if (s->id1_group < 3)
                  {
                    s->type_id =
                        g_type_lookup[s->id1_group][DEV_ID_GROUP_GND];
                  }
                else
                  {
                    s->type_id = LEGOPORT_TYPE_LPF2_UNKNOWN_UART;
                  }
                s->state = DCM_S8_PRE_FINAL_WAIT;
                continue;
              }

          case DCM_S7_READ_RX_HI:
            /* pybricks 257-265: sample uart_rx after cap-debounce.
             * The cap-debounce step left uart_rx in GPIO OUTPUT LOW; we
             * must switch it back to INPUT before reading or
             * stm32_gpioread() returns the (still-low) output drive
             * value forever and the next scan misclassifies the port
             * as UNKNOWN_UART.  Pybricks' pbdrv_gpio_input() helper
             * does this implicitly; NuttX's stm32_gpioread() does not.
             */
            stm32_configgpio(p->uart_rx_in);
            if (!stm32_gpioread(p->uart_rx_in))
              {
                /* Stayed low → ID2 group = PULL_DOWN. */
                if (s->id1_group < 3)
                  {
                    s->type_id =
                        g_type_lookup[s->id1_group][DEV_ID_GROUP_PULL_DOWN];
                  }
              }
            else
              {
                /* Bounced back high → ID2 group = VCC. */
                if (s->id1_group < 3)
                  {
                    s->type_id =
                        g_type_lookup[s->id1_group][DEV_ID_GROUP_VCC];
                  }
              }
            s->state = DCM_S7B_READ_RX_LO;
            continue;

          case DCM_S7B_READ_RX_LO:
            /* Placeholder fallthrough after S7_READ_RX_HI processing. */
            s->state = DCM_S8_PRE_FINAL_WAIT;
            continue;

          case DCM_S8_PRE_FINAL_WAIT:
            /* pybricks 278 UNCONDITIONAL yield — every classification path
             * funnels through this wait before the finalize block.
             */
            s->state = DCM_S8_FINALIZE;
            return;

          case DCM_S8_FINALIZE:
            {
              /* pybricks 280-298: reset pins to safe state, debounce.
               * Also restore uart_rx to INPUT in case any scan path
               * ended with it in OUTPUT LOW (cap-debounce trick) so the
               * next scan reads the actual line level.
               */
              stm32_configgpio(p->gpio2_in);
              stm32_configgpio(p->uart_tx_hi);
              stm32_configgpio(p->uart_buf_lo);
              stm32_configgpio(p->uart_rx_in);

              if (s->type_id == s->prev_type_id)
                {
                  if (s->match_count < UINT8_MAX)
                    {
                      s->match_count++;
                    }

                  if (s->match_count >= LEGOPORT_AFFIRMATIVE_MATCH)
                    {
                      uint8_t new_confirmed = s->type_id;

                      /* Lock for confirmed_type / event_counter / flags. */

                      nxmutex_lock(&s->lock);
                      legoport_post_change(s, new_confirmed);
                      nxmutex_unlock(&s->lock);

                      if (s->in_unowned_rescan)
                        {
                          /* UNOWNED rescan: streak logic for disconnect /
                           * type-change.
                           */
                          if (new_confirmed == LEGOPORT_TYPE_LPF2_UNKNOWN_UART)
                            {
                              /* Same UART device still there; re-arm gate. */
                              s->unowned_disc_streak  = 0;
                              s->unowned_other_streak = 0;
                              s->in_unowned_rescan    = false;
                              s->state                = DCM_LATCHED_UART_UNOWNED_IDLE;
                              s->unowned_idle_until   = clock_systime_ticks()
                                  + MSEC2TICK(LEGOPORT_UNOWNED_PROBE_MS);
                              return;
                            }
                          else if (new_confirmed == LEGOPORT_TYPE_NONE)
                            {
                              s->unowned_disc_streak++;
                              if (s->unowned_disc_streak
                                  >= LEGOPORT_UNOWNED_DISCONNECT_K)
                                {
                                  s->in_unowned_rescan = false;
                                  s->state             = DCM_S0_INIT;
                                  s->match_count       = 0;
                                  s->prev_type_id      = LEGOPORT_TYPE_NONE;
                                  return;
                                }
                              s->in_unowned_rescan  = false;
                              s->state              = DCM_LATCHED_UART_UNOWNED_IDLE;
                              s->unowned_idle_until = clock_systime_ticks()
                                  + MSEC2TICK(LEGOPORT_UNOWNED_PROBE_MS);
                              return;
                            }
                          else
                            {
                              /* New non-UART, non-NONE type — track it. */
                              if (s->unowned_other_type == new_confirmed)
                                {
                                  s->unowned_other_streak++;
                                }
                              else
                                {
                                  s->unowned_other_streak = 1;
                                  s->unowned_other_type   = new_confirmed;
                                }

                              if (s->unowned_other_streak
                                  >= LEGOPORT_UNOWNED_DISCONNECT_K)
                                {
                                  s->in_unowned_rescan = false;
                                  s->state             = DCM_S0_INIT;
                                  s->match_count       = 0;
                                  s->prev_type_id      = LEGOPORT_TYPE_NONE;
                                  return;
                                }
                              s->in_unowned_rescan  = false;
                              s->state              = DCM_LATCHED_UART_UNOWNED_IDLE;
                              s->unowned_idle_until = clock_systime_ticks()
                                  + MSEC2TICK(LEGOPORT_UNOWNED_PROBE_MS);
                              return;
                            }
                        }

                      /* Normal scan path: if we just confirmed UNKNOWN_UART
                       * for the first time, attempt handoff.
                       */
                      if (new_confirmed == LEGOPORT_TYPE_LPF2_UNKNOWN_UART)
                        {
                          /* legoport_invoke_handoff() drops + reacquires
                           * the lock and sets the final state (OWNED or
                           * UNOWNED_IDLE) under the lock based on the CB
                           * result, so we can return immediately.
                           */
                          legoport_invoke_handoff(port);
                          return;
                        }
                    }
                }
              else
                {
                  s->match_count = 0;
                }

              s->prev_type_id = s->type_id;

              /* Restart the scan. */

              s->state = DCM_S0_INIT;
              return;
            }

          default:
            /* Defensive: shouldn't happen.  Reset to S0. */
            s->state = DCM_S0_INIT;
            return;
        }
    }
#endif /* CONFIG_LEGO_PORT_DCM_NOOP */
}

/****************************************************************************
 * HPWORK worker
 ****************************************************************************/

static void legoport_dcm_worker(FAR void *arg)
{
  UNUSED(arg);

  clock_t  now_ticks = clock_systime_ticks();
  uint32_t t0_us     = (uint32_t)TICK2USEC(now_ticks);

  g_total_invocations++;

  if (g_last_invoke_ticks != 0)
    {
      uint32_t interval =
          (uint32_t)TICK2USEC((sclock_t)(now_ticks - g_last_invoke_ticks));
      if (interval > g_max_interval_us)
        {
          g_max_interval_us = interval;
        }
      if (interval > 4000)   g_late_4ms++;
      if (interval > 10000)  g_late_10ms++;
      if (interval > 100000)
        {
          g_late_100ms++;
          syslog(LOG_INFO,
                 "legoport: HPWORK gap %lu us at t=%lu ms\n",
                 (unsigned long)interval,
                 (unsigned long)TICK2MSEC(now_ticks));
        }
    }

  g_last_invoke_ticks = now_ticks;

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      legoport_dcm_step(p);
    }

  uint32_t dt = legoport_now_us() - t0_us;
  if (dt > g_max_step_us)
    {
      g_max_step_us = dt;
    }

  work_queue(HPWORK, &g_legoport_work, legoport_dcm_worker, NULL,
             MSEC2TICK(LEGOPORT_TICK_MS));
}

/****************************************************************************
 * Public API
 ****************************************************************************/

int stm32_legoport_initialize(void)
{
  if (g_legoport_initialized)
    {
      return OK;
    }

  /* Configure all pins to a safe DCM-start state and init per-port state. */

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      const struct legoport_pin_s *pin = &g_legoport_pins[p];
      struct legoport_state_s     *s   = &g_legoport_state[p];

      memset(s, 0, sizeof(*s));
      s->state          = DCM_S0_INIT;
      s->confirmed_type = LEGOPORT_TYPE_NONE;
      s->prev_type_id   = LEGOPORT_TYPE_NONE;
      nxmutex_init(&s->lock);
      nxsem_init(&s->state_change_sem, 0, 0);
      nxsem_init(&s->handoff_quiescent, 0, 0);

      /* Power-on default: ID1 high, buf low (= buffer enabled), ID2 input.
       * Same as DCM_S0_INIT entry conditions.
       */
      stm32_configgpio(pin->uart_tx_hi);
      stm32_configgpio(pin->uart_buf_lo);
      stm32_configgpio(pin->gpio2_in);
      stm32_configgpio(pin->gpio1_in);
      stm32_configgpio(pin->uart_rx_in);
    }

  g_max_step_us       = 0;
  g_max_interval_us   = 0;
  g_last_invoke_ticks = 0;
  g_legoport_initialized = true;

  /* Kick the HPWORK loop. */

  return work_queue(HPWORK, &g_legoport_work, legoport_dcm_worker, NULL,
                    MSEC2TICK(LEGOPORT_TICK_MS));
}

int stm32_legoport_register_uart_handoff(int port,
                                         legoport_uart_handoff_cb_t cb,
                                         void *priv)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct legoport_state_s *s = &g_legoport_state[port];

  nxmutex_lock(&s->lock);

  s->handoff_cb   = cb;
  s->handoff_priv = priv;
  s->handoff_generation++;

  /* Drain any in-flight CB so the caller can free `priv` after we
   * return.  Use the uninterruptible variant to keep the lifetime
   * guarantee intact across signals.
   */

  while (s->handoff_inflight != 0)
    {
      s->handoff_waiters++;
      nxmutex_unlock(&s->lock);
      nxsem_wait_uninterruptible(&s->handoff_quiescent);
      nxmutex_lock(&s->lock);
      s->handoff_waiters--;
    }

  nxmutex_unlock(&s->lock);
  return OK;
}

int stm32_legoport_release_uart(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct legoport_state_s     *s = &g_legoport_state[port];
  const struct legoport_pin_s *p = &g_legoport_pins[port];

  nxmutex_lock(&s->lock);

  /* Bump generation and clear the CB so any concurrent invoke aborts. */

  s->handoff_cb = NULL;
  s->handoff_priv = NULL;
  s->handoff_generation++;

  while (s->handoff_inflight != 0)
    {
      s->handoff_waiters++;
      nxmutex_unlock(&s->lock);
      nxsem_wait_uninterruptible(&s->handoff_quiescent);
      nxmutex_lock(&s->lock);
      s->handoff_waiters--;
    }

  /* Reset pins back to DCM-start state regardless of which latched state
   * we were in.  This is the GPIO equivalent of "go to S0_INIT".
   */

  stm32_configgpio(p->uart_tx_hi);
  stm32_configgpio(p->uart_buf_lo);
  stm32_configgpio(p->gpio2_in);
  stm32_configgpio(p->gpio1_in);
  stm32_configgpio(p->uart_rx_in);

  s->state                = DCM_S0_INIT;
  s->match_count          = 0;
  s->prev_type_id         = LEGOPORT_TYPE_NONE;
  s->in_unowned_rescan    = false;
  s->unowned_disc_streak  = 0;
  s->unowned_other_streak = 0;
  s->unowned_other_type   = LEGOPORT_TYPE_NONE;
  s->flags               &= ~LEGOPORT_FLAG_HANDOFF_OK;

  /* Clearing confirmed_type also raises a state-change edge so any
   * waiter for "disconnect" wakes up.
   */
  legoport_post_change(s, LEGOPORT_TYPE_NONE);

  nxmutex_unlock(&s->lock);
  return OK;
}

uint32_t stm32_legoport_get_max_step_us(void)
{
  return g_max_step_us;
}

uint32_t stm32_legoport_get_max_interval_us(void)
{
  return g_max_interval_us;
}

void stm32_legoport_reset_stats(void)
{
  g_max_step_us       = 0;
  g_max_interval_us   = 0;
  g_total_invocations = 0;
  g_late_4ms          = 0;
  g_late_10ms         = 0;
  g_late_100ms        = 0;
  g_last_invoke_ticks = 0;
}

void stm32_legoport_get_stats(FAR struct legoport_stats_s *out)
{
  out->max_step_us       = g_max_step_us;
  out->max_interval_us   = g_max_interval_us;
  out->total_invocations = g_total_invocations;
  out->late_4ms          = g_late_4ms;
  out->late_10ms         = g_late_10ms;
  out->late_100ms        = g_late_100ms;
}

/* Accessors used by the chardev shim — exposed via spike_prime_hub.h. */

int stm32_legoport_get_info(int port, FAR struct legoport_info_s *out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  struct legoport_state_s *s = &g_legoport_state[port];

  nxmutex_lock(&s->lock);
  out->device_type   = s->confirmed_type;
  out->flags         = s->flags;
  out->reserved[0]   = 0;
  out->reserved[1]   = 0;
  out->event_counter = s->event_counter;
  nxmutex_unlock(&s->lock);

  return OK;
}

int stm32_legoport_wait_change(int port, uint32_t snapshot,
                               uint32_t timeout_ms,
                               FAR uint32_t *new_counter_out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct legoport_state_s *s = &g_legoport_state[port];

  for (; ; )
    {
      nxmutex_lock(&s->lock);
      if (s->event_counter != snapshot)
        {
          uint32_t cur = s->event_counter;
          nxmutex_unlock(&s->lock);
          if (new_counter_out != NULL)
            {
              *new_counter_out = cur;
            }
          return OK;
        }

      nxmutex_unlock(&s->lock);

      int ret;
      if (timeout_ms == 0)
        {
          ret = nxsem_wait(&s->state_change_sem);
        }
      else
        {
          ret = nxsem_tickwait(&s->state_change_sem, MSEC2TICK(timeout_ms));
        }

      if (ret == -ETIMEDOUT)
        {
          return -ETIMEDOUT;
        }

      if (ret == -EINTR)
        {
          /* WAIT ioctls are user-callable so respect EINTR. */
          return -EINTR;
        }
    }
}

#endif /* CONFIG_LEGO_PORT */
