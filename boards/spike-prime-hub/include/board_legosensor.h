/****************************************************************************
 * boards/spike-prime-hub/include/board_legosensor.h
 *
 * Public ABI for the SPIKE Prime Hub LEGO sensor uORB driver (Issue #45).
 *
 * Each I/O port (A..F) exposes telemetry from the attached Powered Up
 * device through a single uORB topic at `/dev/uorb/sensor_lego[0..5]`.
 * The same envelope carries any LUMP-capable device — sensors (Color,
 * Ultrasonic, Force) and motor encoder telemetry (BOOST/Technic/SPIKE
 * angular position, speed, ...).  The user discriminates the active
 * shape via `data_type` / `num_values` / `mode_id`.
 *
 * Subscribers `read()` `struct lump_sample_s` snapshots and may issue
 * the `LEGOSENSOR_*` ioctls below for control plane operations.  The
 * sensor lower-half rejects `O_WROK` opens to keep userspace from
 * injecting fake samples through `sensor_write()`.
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
#define LEGOSENSOR_MAX_DATA_BYTES    LUMP_MAX_PAYLOAD   /* 32 */

/* uORB ring buffer depth — instance-wide circbuf capacity (per-subscriber
 * stores only a position cursor).  Sized for 100 Hz LUMP devices read at
 * 100 ms cadence with ~160 ms slack.
 */

#define LEGOSENSOR_NBUFFER           16

/* Custom ioctl numbers.  `_SNIOC()` resolves to the same group as the
 * standard sensor ioctls so dispatcher routing is uniform.
 */

#define LEGOSENSOR_GET_INFO          _SNIOC(0x00f1)  /* lump_device_info_s * */
#define LEGOSENSOR_GET_STATUS        _SNIOC(0x00f2)  /* lump_status_full_s * */
#define LEGOSENSOR_SELECT            _SNIOC(0x00f3)  /* uint8_t mode (claim req) */
#define LEGOSENSOR_SEND              _SNIOC(0x00f4)  /* legosensor_send_arg_s * */
#define LEGOSENSOR_CLAIM             _SNIOC(0x00f5)  /* arg ignored */
#define LEGOSENSOR_RELEASE           _SNIOC(0x00f6)  /* arg ignored (owner only) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Sample envelope published on every `/dev/uorb/sensor_lego[N]` topic.
 *
 *   timestamp   - μs since boot, taken from `sensor_get_timestamp()`
 *   seq         - per-port monotonic counter; subscribers detect drops
 *                 via gaps
 *   generation  - bumps on initial SYNC, on a SELECT acknowledged by the
 *                 device (next `on_data` with `mode == requested`), and
 *                 on disconnect.  Subscribers that recorded the prior
 *                 generation can ignore stale samples after SELECT
 *   port        - 0..5 (A..F)
 *   type_id     - LPF2 type (0 means disconnected sentinel)
 *   mode_id     - currently reported mode index
 *   data_type   - enum lump_data_type_e (INT8/INT16/INT32/FLOAT)
 *   num_values  - INFO_FORMAT[2]; how many entries of `data_type` `data`
 *                 holds
 *   len         - byte count of the live payload in `data.raw`
 *
 * Sentinel encoding:
 *   type_id != 0, len == 0  → SYNC complete (mode info now available)
 *   type_id == 0, len == 0  → port disconnected
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

/* Argument for `LEGOSENSOR_SEND`. `mode` is the writable mode index;
 * `data[0..len-1]` is copied into the LUMP DATA frame.
 */

struct legosensor_send_arg_s
{
  uint8_t  mode;
  uint8_t  len;
  uint8_t  reserved[2];
  uint8_t  data[LUMP_MAX_PAYLOAD];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Register all six `/dev/uorb/sensor_lego[0..5]` topics and attach the
 * per-port LUMP publish callbacks.  Must be called after the LUMP engine
 * has been brought up (`stm32_legoport_lump_register()`).  Returns 0 on
 * success or a negated errno on failure; partial registrations are
 * cleaned up before the function returns.
 */

int legosensor_uorb_register(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LEGOSENSOR_H */
