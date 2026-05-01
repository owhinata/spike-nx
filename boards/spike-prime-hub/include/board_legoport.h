/****************************************************************************
 * boards/spike-prime-hub/include/board_legoport.h
 *
 * Public ABI for the SPIKE Prime Hub I/O port device-connection-manager
 * (DCM) char devices /dev/legoport0../dev/legoport5 (Issue #42).  Each node
 * exposes one of the six I/O ports (A-F) and reports the device type that
 * the kernel-side DCM has detected via the resistor-divider passive scheme
 * ported from pybricks `lib/pbio/drv/legodev/legodev_pup.c`.
 *
 * Type IDs match pybricks `pbdrv_legodev_type_id_t` so that pybricks
 * expectations carry across.  In particular UNKNOWN_UART = 14 (NOT 8 — 8
 * is LIGHT).
 *
 * Lifecycle:
 *   - Nodes are registered statically for all 6 ports at boot.
 *   - Single-open (mirrors /dev/ttyBT) — open() returns -EBUSY if another
 *     fd is already holding the port.
 *   - read() returns -EAGAIN; streaming sensor/motor data is the job of
 *     /dev/legosensor[N] and /dev/legomotor[N] (Issue #44/#45).
 *   - write() returns -ENOTSUP.
 *   - poll(POLLIN) fires whenever the confirmed device type changes.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOPORT_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOPORT_H

#include <stdint.h>
#include <nuttx/fs/ioctl.h>

#define BOARD_LEGOPORT_COUNT          6
#define BOARD_LEGOPORT_DEVPATH_FMT    "/dev/legoport%d"

/* Device type enum — values are the pybricks PBDRV_LEGODEV_TYPE_ID_LPF2_*
 * constants (`pybricks/lib/pbio/include/pbdrv/legodev.h`).  We expose the
 * subset that the SPIKE Prime Hub passive DCM can resolve, plus the
 * UNKNOWN_UART placeholder used for any UART-capable device until the LUMP
 * engine identifies it (Issue #43).
 */

enum legoport_type_id_e
{
  LEGOPORT_TYPE_NONE              = 0,
  LEGOPORT_TYPE_LPF2_MMOTOR       = 1,    /* WeDo medium / 45303 */
  LEGOPORT_TYPE_LPF2_TRAIN        = 2,
  LEGOPORT_TYPE_LPF2_TURN         = 3,
  LEGOPORT_TYPE_LPF2_POWER        = 4,
  LEGOPORT_TYPE_LPF2_TOUCH        = 5,
  LEGOPORT_TYPE_LPF2_LMOTOR       = 6,
  LEGOPORT_TYPE_LPF2_XMOTOR       = 7,
  LEGOPORT_TYPE_LPF2_LIGHT        = 8,    /* 88005 Powered Up Lights */
  LEGOPORT_TYPE_LPF2_LIGHT1       = 9,
  LEGOPORT_TYPE_LPF2_LIGHT2       = 10,
  LEGOPORT_TYPE_LPF2_TPOINT       = 11,
  LEGOPORT_TYPE_LPF2_EXPLOD       = 12,
  LEGOPORT_TYPE_LPF2_3_PART       = 13,
  LEGOPORT_TYPE_LPF2_UNKNOWN_UART = 14,   /* latched until #43 LUMP IDs it */
};

/* Flags for `legoport_info_s::flags` */

#define LEGOPORT_FLAG_CONNECTED   (1u << 0)  /* confirmed type != NONE */
#define LEGOPORT_FLAG_IS_UART     (1u << 1)  /* UNKNOWN_UART or LUMP-resolved */
#define LEGOPORT_FLAG_IS_PASSIVE  (1u << 2)  /* resistor-divider device */
#define LEGOPORT_FLAG_HANDOFF_OK  (1u << 3)  /* #43 took ownership of UART */

struct legoport_info_s
{
  uint8_t  device_type;       /* one of legoport_type_id_e */
  uint8_t  flags;             /* LEGOPORT_FLAG_* */
  uint8_t  reserved[2];
  uint32_t event_counter;     /* monotonic edge counter (~136 yr at 1Hz) */
};

/* HPWORK cadence stats returned by LEGOPORT_GET_STATS — used by the
 * `legoport stats` CLI and tests/test_legoport.py to catch HPWORK
 * contention regressions when DCM coexists with TLC5955 LED frame sync,
 * battery polling, IMU DRDY, etc.
 */

struct legoport_stats_s
{
  uint32_t max_step_us;       /* worst worker execution time, microseconds */
  uint32_t max_interval_us;   /* worst gap between worker invocations */
  uint32_t total_invocations; /* total worker calls since last reset */
  uint32_t late_4ms;          /* count of intervals > 4 ms */
  uint32_t late_10ms;         /* count of intervals > 10 ms */
  uint32_t late_100ms;        /* count of intervals > 100 ms */
};

/* IOC base — picks 0x4800 which is unused in <nuttx/fs/ioctl.h> (next free
 * slot above _PTPBASE=0x4700, well below _WLIOCBASE=0x8b00).  The _IOC
 * macro packs as `(base | nr)` so the high byte must not collide with
 * other ranges.
 */

#define _LEGOPORTBASE             (0x4800)
#define _LEGOPORTIOC(nr)          _IOC(_LEGOPORTBASE, nr)

#define LEGOPORT_GET_DEVICE_TYPE  _LEGOPORTIOC(0x0001) /* arg: uint8_t* */
#define LEGOPORT_GET_DEVICE_INFO  _LEGOPORTIOC(0x0002) /* arg: struct legoport_info_s* */
#define LEGOPORT_WAIT_CONNECT     _LEGOPORTIOC(0x0003) /* arg: timeout_ms (0=infinite) */
#define LEGOPORT_WAIT_DISCONNECT  _LEGOPORTIOC(0x0004) /* arg: timeout_ms (0=infinite) */
#define LEGOPORT_GET_STATS        _LEGOPORTIOC(0x0005) /* arg: struct legoport_stats_s* */
#define LEGOPORT_RESET_STATS      _LEGOPORTIOC(0x0006) /* arg: 0 (clear max_step_us / max_interval_us) */

/* LUMP UART engine diagnostics (Issue #43, CONFIG_LEGO_LUMP_DIAG only).
 * Triggers a kernel-side `lump_uart_hw_dump()` that prints RCC / USART /
 * NVIC register state to syslog for the 6 LUMP UARTs.  No arg.
 */

#define LEGOPORT_LUMP_HW_DUMP     _LEGOPORTIOC(0x0007)

/* LUMP UART engine info snapshot (Issue #43 Phase 2 onward).  Returns
 * the per-port `lump_device_info_s` once the engine has reached the
 * SYNCED state.  arg: `struct lump_device_info_s *`.  Returns -EAGAIN
 * if not yet synced.  Defined in `board_lump.h`.
 */

#define LEGOPORT_LUMP_GET_INFO    _LEGOPORTIOC(0x0008)

/* LUMP UART engine TX requests (Issue #43 Phase 3).  Posted to the
 * per-port kthread and serviced between RX byte reads in the DATA
 * loop.  All return 0 on accept (kthread will issue the wire TX
 * within ~20 ms); -EAGAIN if not synced; -EINVAL if mode out of range
 * or other validation failure; -ENOTSUP for SEND on non-writable mode.
 */

#define LEGOPORT_LUMP_SELECT      _LEGOPORTIOC(0x0009)  /* arg: uint8_t mode */

/* `arg`: `struct legoport_lump_send_arg_s *` (mode + len + bytes). */

struct legoport_lump_send_arg_s
{
  uint8_t  mode;
  uint8_t  len;
  uint8_t  reserved[2];
  uint8_t  data[32];   /* LUMP_MAX_PAYLOAD */
};
#define LEGOPORT_LUMP_SEND        _LEGOPORTIOC(0x000A)

/* Poll one DATA frame from the per-port engine ring.  arg:
 * `struct lump_data_frame_s *`.  Returns 0 with frame, -EAGAIN if
 * empty, -EAGAIN also if not yet synced.  Used by `legoport lump
 * watch`; ~10 ms polling cadence is comfortable.
 */

#define LEGOPORT_LUMP_POLL_DATA   _LEGOPORTIOC(0x000B)

/* Full per-port engine status (Issue #43 Phase 4).  arg:
 * `struct lump_status_full_s *`.  Used by `legoport lump status`.
 */

#define LEGOPORT_LUMP_GET_STATUS_EX _LEGOPORTIOC(0x000C)

/* H-bridge PWM port driver (Issue #80, integrated into the legoport
 * chardev because LUMP SYNC drives the H-bridge for sensors that
 * advertise NEEDS_SUPPLY_PIN1/PIN2).
 *
 *   - The LEGO Powered Up I/O port drives "passive" H-bridge devices
 *     (M-motor 45303, Train motor 88011, simple DC outputs) directly,
 *     and is also the supply pin for active sensors (SPIKE Color /
 *     Ultrasonic) — pybricks `legodev_pup_uart.c:894-900`.
 *   - LUMP can pin a port as supply (auto-driven on SYNC).  While
 *     pinned, SET_DUTY / COAST / BRAKE return -EBUSY so userspace
 *     cannot disturb the supply rail; GET_PWM_STATUS still works for
 *     observation.
 *   - When unpinned (no LUMP supply requirement), userspace owns the
 *     duty cycle — pybricks-equivalent of `pbio_dcmotor_set_voltage()`.
 *   - close(fd) auto-COASTs the port if it isn't pinned.
 */

#define LEGOPORT_PWM_DUTY_MAX     (10000)
#define LEGOPORT_PWM_DUTY_MIN     (-10000)

enum legoport_pwm_state_e
{
  LEGOPORT_PWM_STATE_COAST = 0,  /* H-bridge floating, motor coasts */
  LEGOPORT_PWM_STATE_BRAKE = 1,  /* both pins HIGH, windings shorted */
  LEGOPORT_PWM_STATE_PWM   = 2,  /* drive at the cached duty */
};

#define LEGOPORT_PWM_FLAG_PINNED  (1u << 0)  /* LUMP holds the H-bridge */

struct legoport_pwm_status_s
{
  int16_t  duty;                 /* -10000..10000 (sign = direction) */
  uint8_t  state;                /* enum legoport_pwm_state_e */
  uint8_t  flags;                /* LEGOPORT_PWM_FLAG_* */
  uint8_t  reserved[4];
};

#define LEGOPORT_PWM_SET_DUTY     _LEGOPORTIOC(0x0010) /* arg: int16_t duty (-10000..10000) */
#define LEGOPORT_PWM_COAST        _LEGOPORTIOC(0x0011) /* arg: ignored */
#define LEGOPORT_PWM_BRAKE        _LEGOPORTIOC(0x0012) /* arg: ignored */
#define LEGOPORT_PWM_GET_STATUS   _LEGOPORTIOC(0x0013) /* arg: struct legoport_pwm_status_s * */

/* Pre-computed GPIO descriptors for one I/O port.  All entries except
 * `*_af` are NuttX `stm32_configgpio()` arguments — packed uint32_t with
 * mode/pull/speed/output/port/pin baked in.  `uart_tx_af` / `uart_rx_af`
 * are the AF-mode descriptors that #43 LUMP installs after taking
 * ownership.  PUPDR is `00 (no pull)` in every entry so resistor-divider
 * detection isn't biased.
 *
 * The `_lo`, `_hi`, `_in` suffixes denote the GPIO state (output low,
 * output high, input float) that #42 cycles through during DCM scans.
 */

struct legoport_pin_s
{
  uint32_t gpio1_in;          /* ID1 input — pin 5 read path */
  uint32_t gpio2_in;          /* ID2 input */
  uint32_t gpio2_lo;          /* ID2 output low */
  uint32_t gpio2_hi;          /* ID2 output high */
  uint32_t uart_tx_in;        /* uart_tx as GPIO input */
  uint32_t uart_tx_hi;        /* uart_tx as GPIO output high */
  uint32_t uart_tx_lo;        /* uart_tx as GPIO output low */
  uint32_t uart_rx_in;        /* uart_rx as GPIO input */
  uint32_t uart_rx_lo;        /* uart_rx as GPIO output low (cap-debounce) */
  uint32_t uart_buf_lo;       /* uart_buf as GPIO output low (buffer enabled) */
  uint32_t uart_buf_hi;       /* uart_buf as GPIO output high */
  uint32_t uart_tx_af;        /* installed by #43 handoff CB */
  uint32_t uart_rx_af;        /* installed by #43 handoff CB */
  uint8_t  port_index;        /* 0..5 for A..F — used by #43 to look up its UART instance */
};

/* Handoff callback — invoked by DCM when it confirms UNKNOWN_UART on a
 * port.  The callback owns: switching uart_tx/uart_rx to UART AF mode,
 * deasserting uart_buf, and starting any LUMP / per-port worker thread.
 *
 * The callback runs without the per-port lock held but its `priv` pointer
 * is pinned by the inflight refcount until it returns, so it is safe to
 * dereference even if a concurrent unregister is in progress (the
 * unregister will block on the inflight drain).
 *
 * IMPORTANT: the callback MUST NOT call stm32_legoport_register_uart_handoff()
 * or stm32_legoport_release_uart() for the SAME port from inside itself —
 * doing so deadlocks on the inflight drain.  Cross-port calls are fine.
 *
 * Once the callback returns OK, #43 owns the port and MUST call
 * stm32_legoport_release_uart(port) from every exit path (disconnect,
 * timeout, driver shutdown, error).  There is no DCM-side watchdog: a
 * leaked owner keeps the port latched forever until reboot.
 */

typedef int (*legoport_uart_handoff_cb_t)(int port,
                                          const struct legoport_pin_s *pins,
                                          void *priv);

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOPORT_H */
