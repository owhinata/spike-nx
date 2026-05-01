/****************************************************************************
 * apps/btsensor/sensor_sampler.h
 *
 * LEGO Powered Up sensor (uORB legosensor) snapshot helper for the
 * BUNDLE emitter (Issue #88).
 *
 * Issue A scope: provide the 6-class snapshot interface so bundle_emitter
 * can fill its TLV section with stub entries (all BOUND=false /
 * payload_len=0 / age=saturated).  Real publish-tracking + write APIs
 * land in Issues #89 (B) and #90 (C); the function signatures are
 * declared up front to keep the firmware ABI stable across the three
 * incremental commits.
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

/* Initialise the module.  Issue A: no fd opens.  Returns 0. */

int  sensor_sampler_init(void);

/* Module shutdown.  Implicitly disables sampling first so any held
 * resource (CLAIM, fd) is released.  Safe from the BTstack main thread.
 */

void sensor_sampler_deinit(void);

/* Toggle SENSOR streaming.  Issue A: just records the flag; Issue B
 * will open the 6 uORB topics and Issue C will hold LEGOSENSOR_CLAIM
 * across the on=true window.  Must run on the BTstack main thread.
 */

int  sensor_sampler_set_enabled(bool on);
bool sensor_sampler_is_enabled(void);

/* Fill `out` with one entry per uORB sensor class in fixed order
 * (BTSENSOR_LEGO_CLASS_*).  After the call every FRESH flag the sampler
 * was tracking is cleared, so a subsequent snapshot only reports
 * publishes that occurred between the two snapshots.  Issue A always
 * returns BOUND=false / payload_len=0 / age=saturated.
 *
 * Must run on the BTstack main thread.
 */

void sensor_sampler_snapshot(struct sensor_class_state_s out[BTSENSOR_TLV_COUNT]);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_SENSOR_SAMPLER_H */
