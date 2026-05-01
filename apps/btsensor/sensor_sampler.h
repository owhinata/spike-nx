/****************************************************************************
 * apps/btsensor/sensor_sampler.h
 *
 * LEGO Powered Up sensor (uORB legosensor) snapshot helper for the
 * BUNDLE emitter.
 *
 * Issue B (this commit): full read-side implementation — opens the six
 * /dev/uorb/sensor_* class topics on `set_enabled(true)`, registers each
 * fd as a btstack data source, and tracks the latest publish per class.
 * `snapshot()` returns BOUND/FRESH/age/payload that bundle_emitter
 * serialises into the TLV section.  Write APIs (SELECT / SEND / SET_PWM)
 * are deferred to Issue C.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_SENSOR_SAMPLER_H
#define __APPS_BTSENSOR_SENSOR_SAMPLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "btsensor_wire.h"

#if defined __cplusplus
extern "C" {
#endif

/* Per-class state collected by sensor_sampler_snapshot().  Mirrors the
 * TLV wire layout (btsensor_wire.h) so bundle_emitter can serialise
 * each entry with a couple of byte writes.
 */

struct sensor_class_state_s
{
  uint8_t  port_id;          /* 0..5 when bound, 0xFF otherwise */
  uint8_t  mode_id;          /* current LUMP mode, 0 when unbound */
  uint8_t  data_type;        /* 0:INT8 1:INT16 2:INT32 3:FLOAT */
  uint8_t  num_values;       /* INFO_FORMAT[2] */
  uint8_t  payload_len;      /* 0..32; 0 unless FRESH */
  uint8_t  flags;            /* BTSENSOR_TLV_FLAG_BOUND / _FRESH */
  uint8_t  age_10ms;         /* 0..254, 0xFF saturated / never */
  uint16_t seq;              /* lump_sample_s.seq & 0xFFFF */
  uint8_t  payload[BTSENSOR_TLV_PAYLOAD_MAX];
};

/* Initialise the module.  Returns 0.  fds are NOT opened here — that
 * happens in `set_enabled(true)`.
 */

int  sensor_sampler_init(void);

/* Module shutdown.  Implicitly disables sampling first so any open fd
 * is released.  Safe from the BTstack main thread.
 */

void sensor_sampler_deinit(void);

/* Toggle SENSOR streaming.  on=true opens the 6 /dev/uorb/sensor_*
 * topics and registers them as btstack data sources; on=false closes
 * them.  Must run on the BTstack main thread.  No `LEGOSENSOR_CLAIM`
 * is taken here — write APIs (Issue C) take CLAIM transiently around
 * each ioctl.
 */

int  sensor_sampler_set_enabled(bool on);
bool sensor_sampler_is_enabled(void);

/* Fill `out` with one entry per uORB sensor class in fixed order
 * (LEGOSENSOR_CLASS_*).  After the call every FRESH flag the sampler
 * was tracking is cleared, so a subsequent snapshot only reports
 * publishes that occurred between the two snapshots.  When SENSOR is
 * disabled, every entry is reported BOUND=false / payload_len=0 /
 * age=saturated regardless of cached state.
 *
 * Must run on the BTstack main thread.
 */

void sensor_sampler_snapshot(struct sensor_class_state_s out[BTSENSOR_TLV_COUNT]);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_SENSOR_SAMPLER_H */
