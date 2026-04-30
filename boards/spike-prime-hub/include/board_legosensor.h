/****************************************************************************
 * boards/spike-prime-hub/include/board_legosensor.h
 *
 * Public ABI for the SPIKE Prime Hub LEGO sensor uORB driver (Issue #79).
 *
 * Replaces the per-port topology of Issue #45.  Six device-class topics
 * are statically registered at boot:
 *
 *     /dev/uorb/sensor_color         (LPF2 type 61)
 *     /dev/uorb/sensor_ultrasonic    (LPF2 type 62)
 *     /dev/uorb/sensor_force         (LPF2 type 63)
 *     /dev/uorb/sensor_motor_m       (LPF2 type 49 — module / arm motor)
 *     /dev/uorb/sensor_motor_r       (LPF2 type 48 on port A/C/E)
 *     /dev/uorb/sensor_motor_l       (LPF2 type 48 on port B/D/F)
 *
 * Each topic is bound to at most one physical port at any instant: when
 * `on_sync(port, info)` arrives, `legosensor_classify(type_id, port)`
 * routes the port to its class slot under a class-global bind lock.  If
 * another port already owns that class slot, the lower port number wins
 * — the loser's frames are dropped.  When the bound port disconnects or
 * the class is reclaimed by a lower-numbered port, the prior owner's
 * subscribers receive a disconnect sentinel and any control-plane CLAIM
 * is invalidated (write-side ioctls return `-ENODEV` until re-CLAIM).
 *
 * Subscribers `read()` `struct lump_sample_s` snapshots and may issue
 * the `LEGOSENSOR_*` ioctls below.  The lower-half rejects `O_WROK`
 * opens to prevent userspace from injecting fake samples through
 * `sensor_write()`.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOSENSOR_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOSENSOR_H

#include <stdint.h>
#include <stddef.h>

#include <nuttx/sensors/ioctl.h>

#include "board_lump.h"   /* lump_device_info_s, lump_status_full_s,
                           * enum lump_data_type_e, LUMP_MAX_PAYLOAD
                           */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LEGOSENSOR_NUM_PORTS         6
#define LEGOSENSOR_CLASS_NUM         6
#define LEGOSENSOR_MAX_DATA_BYTES    LUMP_MAX_PAYLOAD   /* 32 */

/* uORB ring buffer depth — instance-wide circbuf capacity (per-subscriber
 * stores only a position cursor).  Sized for 100 Hz LUMP devices read at
 * 100 ms cadence with ~160 ms slack.
 */

#define LEGOSENSOR_NBUFFER           16

/* Custom ioctl numbers.  `_SNIOC()` resolves to the same group as the
 * standard sensor ioctls so dispatcher routing is uniform.
 */

#define LEGOSENSOR_GET_INFO          _SNIOC(0x00f1)  /* legosensor_info_arg_s *   */
#define LEGOSENSOR_GET_STATUS        _SNIOC(0x00f2)  /* lump_status_full_s *       */
#define LEGOSENSOR_SELECT            _SNIOC(0x00f3)  /* legosensor_select_arg_s *  */
#define LEGOSENSOR_SEND              _SNIOC(0x00f4)  /* legosensor_send_arg_s *    */
#define LEGOSENSOR_CLAIM             _SNIOC(0x00f5)  /* arg ignored                */
#define LEGOSENSOR_RELEASE           _SNIOC(0x00f6)  /* arg ignored (owner only)   */
#define LEGOSENSOR_SET_PWM           _SNIOC(0x00f7)  /* legosensor_pwm_arg_s *     */

/* Reserved class-specific ioctl ranges (currently `-ENOTTY`).  Future
 * per-class commands should pick a number from the matching block so
 * collisions with the common range above stay impossible.
 */

#define LEGOSENSOR_COLOR_BASE        _SNIOC(0x0100)  /* 0x0100..0x010f */
#define LEGOSENSOR_ULTRASONIC_BASE   _SNIOC(0x0110)  /* 0x0110..0x011f */
#define LEGOSENSOR_FORCE_BASE        _SNIOC(0x0120)  /* 0x0120..0x012f */
#define LEGOSENSOR_MOTOR_M_BASE      _SNIOC(0x0130)  /* 0x0130..0x013f */
#define LEGOSENSOR_MOTOR_R_BASE      _SNIOC(0x0140)  /* 0x0140..0x014f */
#define LEGOSENSOR_MOTOR_L_BASE      _SNIOC(0x0150)  /* 0x0150..0x015f */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Device class index — one entry per /dev/uorb/sensor_* topic. */

enum legosensor_class_e
{
  LEGOSENSOR_CLASS_NONE       = -1,
  LEGOSENSOR_CLASS_COLOR      =  0,
  LEGOSENSOR_CLASS_ULTRASONIC,
  LEGOSENSOR_CLASS_FORCE,
  LEGOSENSOR_CLASS_MOTOR_M,
  LEGOSENSOR_CLASS_MOTOR_R,
  LEGOSENSOR_CLASS_MOTOR_L,
};

/* Sample envelope published on every class topic.  Identical 56-byte
 * shape that the per-port driver of Issue #45 used — only the routing
 * changed.  See `port` for the underlying physical port at the time the
 * frame was emitted.
 *
 *   timestamp   - μs since boot, taken from `sensor_get_timestamp()`
 *   seq         - per-port monotonic counter; subscribers detect drops
 *                 via gaps.  Resets implicitly when bind ownership of the
 *                 class topic moves to a different port (the new port has
 *                 its own seq).
 *   generation  - bumps on initial SYNC, on a SELECT acknowledged by the
 *                 device (next `on_data` with `mode == requested`), and
 *                 on disconnect / bind-ownership change
 *   port        - 0..5 (A..F) — physical port currently bound to the class
 *   type_id     - LPF2 type (0 means disconnected sentinel)
 *   mode_id     - currently reported mode index
 *   data_type   - enum lump_data_type_e (INT8/INT16/INT32/FLOAT)
 *   num_values  - INFO_FORMAT[2]; how many entries of `data_type` `data`
 *                 holds
 *   len         - byte count of the live payload in `data.raw`
 *
 * Sentinel encoding:
 *   type_id != 0, len == 0  → SYNC complete (mode info now available)
 *   type_id == 0, len == 0  → port disconnected / bind-ownership lost
 */

struct lump_sample_s
{
  uint64_t timestamp;
  uint32_t seq;
  uint32_t generation;
  uint8_t  port;
  uint8_t  type_id;
  uint8_t  mode_id;
  uint8_t  data_type;
  uint8_t  num_values;
  uint8_t  len;
  uint16_t reserved;
  union
  {
    uint8_t  raw[LEGOSENSOR_MAX_DATA_BYTES];
    int8_t   i8 [LEGOSENSOR_MAX_DATA_BYTES];
    int16_t  i16[LEGOSENSOR_MAX_DATA_BYTES / 2];
    int32_t  i32[LEGOSENSOR_MAX_DATA_BYTES / 4];
    float    f32[LEGOSENSOR_MAX_DATA_BYTES / 4];
  } data;
};

_Static_assert(sizeof(struct lump_sample_s) == 56,
               "lump_sample_s ABI: total size");
_Static_assert(offsetof(struct lump_sample_s, data) == 24,
               "lump_sample_s ABI: data offset");

/* `LEGOSENSOR_GET_INFO` — combined snapshot of the bound port number and
 * the LUMP device info, returned in a single ioctl so subscribers do not
 * need to chase the port through `lump_sample_s::port` separately.
 */

struct legosensor_info_arg_s
{
  uint8_t  port;                /* bound port (0..5) */
  uint8_t  reserved[3];
  struct lump_device_info_s info;
};

/* `LEGOSENSOR_SELECT` — request the bound LUMP device to switch to mode. */

struct legosensor_select_arg_s
{
  uint8_t  mode;
  uint8_t  reserved[3];
};

/* `LEGOSENSOR_SEND` — write `data[0..len-1]` to a writable LUMP mode.
 * `mode` selects the writable mode index.  This bypasses the active SELECT
 * — writable-mode SEND does not affect the device's reporting mode.
 */

struct legosensor_send_arg_s
{
  uint8_t  mode;
  uint8_t  len;                 /* 1..LEGOSENSOR_MAX_DATA_BYTES */
  uint8_t  reserved[2];
  uint8_t  data[LEGOSENSOR_MAX_DATA_BYTES];
};

/* `LEGOSENSOR_SET_PWM` — class-aware PWM / LED control.
 *
 *   COLOR     : num_channels = 3, channels[0..2] = LED 0..2 brightness
 *               in 0..10000 (.01 % units).  Routed via LUMP writable
 *               LIGHT mode.
 *   ULTRASONIC: num_channels = 4, channels[0..3] = eye LED 0..3 brightness
 *               in 0..10000.  Routed via LUMP writable LIGHT mode.
 *   FORCE     : returns `-ENOTSUP` (no actuator on the device).
 *   MOTOR_*   : num_channels = 1, channels[0] = signed duty -10000..10000.
 *               Returns `-ENOTSUP` until #80 lands the H-bridge backend.
 *
 * Any out-of-range value, mismatched `num_channels`, or negative LED
 * value yields `-EINVAL`.  When the bound device's firmware does not
 * advertise a "LIGHT" writable mode of the expected shape (3×INT8 for
 * color, 4×INT8 for ultrasonic), `-ENOTSUP` is returned.
 */

struct legosensor_pwm_arg_s
{
  uint8_t  num_channels;        /* class-dependent: 3 / 4 / 1 */
  uint8_t  reserved[3];
  int16_t  channels[4];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Register the six device-class uORB topics under `/dev/uorb/sensor_*`
 * and attach the per-port LUMP publish callbacks.  Must be called after
 * the LUMP engine has been brought up (`stm32_legoport_lump_register()`)
 * because `lump_attach()` may invoke `on_sync` synchronously.  Returns 0
 * on success or a negated errno on failure; partial registrations are
 * cleaned up before the function returns.
 */

int legosensor_uorb_register(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOSENSOR_H */
