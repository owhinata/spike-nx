/****************************************************************************
 * apps/btsensor/bundle_emitter.c
 *
 * 100 Hz BUNDLE frame emitter (Issue #88).  See bundle_emitter.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include "btstack_run_loop.h"

#include <arch/board/board_lsm6dsl.h>

#include "bundle_emitter.h"
#include "btsensor_tx.h"
#include "btsensor_wire.h"
#include "imu_sampler.h"
#include "sensor_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_APP_BTSENSOR_BUNDLE_TICK_MS
#  define CONFIG_APP_BTSENSOR_BUNDLE_TICK_MS  10
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_timer_source_t g_tick_timer;
static bool                   g_initialized;
static bool                   g_imu_on;
static bool                   g_sensor_on;
static bool                   g_timer_armed;
static uint16_t               g_seq;

/* Issue #139: idx -> physical lookup tables for BUNDLE header source.
 * Must match the driver-private enums (lsm6dsl_uorb.c:124-154).  The
 * gyro/xl tables are sparse: invalid driver idx slots get 0 and the
 * encoder simply emits 0 in that field (the host's matching table
 * will likewise resolve to 0/unknown — fail-soft).
 */

static const uint16_t g_odr_hz_table[] =
{
  [0]  = 0,        /* ODR_OFF */
  [1]  = 13,       /* ODR_12_5HZ */
  [2]  = 26,       /* ODR_26HZ */
  [3]  = 52,       /* ODR_52HZ */
  [4]  = 104,      /* ODR_104HZ */
  [5]  = 208,      /* ODR_208HZ */
  [6]  = 416,      /* ODR_416HZ */
  [7]  = 833,      /* ODR_833HZ */
  [8]  = 1660,     /* ODR_1660HZ */
  [9]  = 3330,     /* ODR_3330HZ */
  [10] = 6660,     /* ODR_6660HZ */
};

static const uint8_t g_fsr_xl_g_table[] =
{
  [0] = 2,         /* FSR_XL_2G */
  [1] = 16,        /* FSR_XL_16G */
  [2] = 4,         /* FSR_XL_4G */
  [3] = 8,         /* FSR_XL_8G */
};

static const uint16_t g_fsr_gy_dps_table[] =
{
  [0] = 250,       /* FSR_GY_250DPS */
  [1] = 125,       /* FSR_GY_125DPS */
  [2] = 500,       /* FSR_GY_500DPS */
  [3] = 0,         /* unused */
  [4] = 1000,      /* FSR_GY_1000DPS */
  [5] = 0,         /* unused */
  [6] = 2000,      /* FSR_GY_2000DPS */
};

/* Last-known config idx, updated on every non-empty drain.  Used as
 * the BUNDLE header source when a tick drains 0 samples but IMU is
 * ON (e.g. low ODR where the 10 ms tick beats the next DRDY); without
 * this fallback the header would emit 0/0/0 and host UI would briefly
 * latch invalid values.
 */

static uint8_t s_last_odr_idx;
static uint8_t s_last_fsr_xl_idx;
static uint8_t s_last_fsr_gy_idx;

/* One static scratch buffer reused on every tick.  The TX arbiter
 * (btsensor_tx_try_enqueue_frame) memcpy's the contents into its
 * internal ring before returning, so re-using the same backing memory
 * each tick is safe — the run loop is single-threaded and the tick
 * callback runs to completion before the next.
 */

static uint8_t g_bundle_buf[BTSENSOR_TX_FRAME_MAX_SIZE];

_Static_assert(BTSENSOR_BUNDLE_FRAME_MAX <= sizeof(g_bundle_buf),
               "g_bundle_buf too small for worst-case BUNDLE frame");

/****************************************************************************
 * Private Functions — little-endian writers
 ****************************************************************************/

static inline void put_u16le(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)(v & 0xff);
  dst[1] = (uint8_t)((v >> 8) & 0xff);
}

static inline void put_u32le(uint8_t *dst, uint32_t v)
{
  dst[0] = (uint8_t)(v & 0xff);
  dst[1] = (uint8_t)((v >> 8) & 0xff);
  dst[2] = (uint8_t)((v >> 16) & 0xff);
  dst[3] = (uint8_t)((v >> 24) & 0xff);
}

/****************************************************************************
 * Private Functions — bundle assembly
 ****************************************************************************/

/* Serialise one IMU sample into the 16-byte slot at `dst`. */

static void encode_imu_sample(uint8_t *dst, const struct sensor_imu *s,
                              uint32_t ts_delta_us)
{
  put_u16le(dst + 0,  (uint16_t)s->ax);
  put_u16le(dst + 2,  (uint16_t)s->ay);
  put_u16le(dst + 4,  (uint16_t)s->az);
  put_u16le(dst + 6,  (uint16_t)s->gx);
  put_u16le(dst + 8,  (uint16_t)s->gy);
  put_u16le(dst + 10, (uint16_t)s->gz);
  put_u32le(dst + 12, ts_delta_us);
}

/* Serialise one TLV entry.  Returns the number of bytes written
 * (10 + payload_len).  When `class_id_override` is non-negative, it
 * replaces state->payload[]'s class identification — the caller
 * passes the table index so a snapshot that left class_id zero still
 * gets the right wire value.
 */

static size_t encode_tlv(uint8_t *dst,
                         uint8_t class_id,
                         const struct sensor_class_state_s *state)
{
  dst[0] = class_id;
  dst[1] = state->port_id;
  dst[2] = state->mode_id;
  dst[3] = state->data_type;
  dst[4] = state->num_values;

  uint8_t payload_len =
      (state->flags & BTSENSOR_TLV_FLAG_FRESH) ? state->payload_len : 0;
  if (payload_len > BTSENSOR_TLV_PAYLOAD_MAX)
    {
      payload_len = BTSENSOR_TLV_PAYLOAD_MAX;
    }

  dst[5] = payload_len;
  dst[6] = state->flags;
  dst[7] = state->age_10ms;
  put_u16le(dst + 8, state->seq);

  if (payload_len > 0)
    {
      memcpy(dst + BTSENSOR_TLV_HEADER_SIZE, state->payload, payload_len);
    }

  return BTSENSOR_TLV_HEADER_SIZE + payload_len;
}

/* Build one BUNDLE frame at `g_bundle_buf` and hand it to the TX
 * arbiter.  Returns 0 on enqueue, otherwise the rc from
 * btsensor_tx_try_enqueue_frame() (which is informational; the TX
 * arbiter still latches the new frame even when -ENOSPC is returned
 * via drop-oldest).
 */

/* Encode and enqueue one BUNDLE frame.  `imu_samples` MUST be a slice
 * whose elements share the same odr_idx / fsr_xl_idx / fsr_gy_idx (the
 * caller, emit_bundle(), splits drained samples by idx homogeneity so
 * each frame is internally consistent).  When `imu_count == 0` and
 * g_imu_on, the header sources rate/fsr from the s_last_*_idx cache;
 * when !g_imu_on, the header rate/fsr fields are 0.
 *
 * Returns the rc from btsensor_tx_try_enqueue_frame().
 */

static int emit_one_frame(const struct sensor_imu *imu_samples,
                          size_t imu_count)
{
  uint32_t tick_ts_us;
  if (imu_count > 0)
    {
      /* Use the oldest sample's timestamp so every ts_delta_us is
       * non-negative (Codex review).
       */

      tick_ts_us = imu_samples[0].timestamp;
    }
  else
    {
      tick_ts_us = imu_sampler_now_us();
    }

  /* Resolve the BUNDLE header's rate/fsr from the sample idx (or the
   * last-known cache when this frame has no samples but IMU is ON).
   * Issue #139: replaces the previous imu_sampler_get_*() cache path.
   */

  uint8_t odr_idx = 0;
  uint8_t xl_idx  = 0;
  uint8_t gy_idx  = 0;
  if (imu_count > 0)
    {
      odr_idx = imu_samples[0].odr_idx;
      xl_idx  = imu_samples[0].fsr_xl_idx;
      gy_idx  = imu_samples[0].fsr_gy_idx;
      s_last_odr_idx    = odr_idx;
      s_last_fsr_xl_idx = xl_idx;
      s_last_fsr_gy_idx = gy_idx;
    }
  else if (g_imu_on)
    {
      odr_idx = s_last_odr_idx;
      xl_idx  = s_last_fsr_xl_idx;
      gy_idx  = s_last_fsr_gy_idx;
    }

  uint16_t hdr_odr_hz = 0;
  uint8_t  hdr_xl_g   = 0;
  uint16_t hdr_gy_dps = 0;
  if (odr_idx < (sizeof(g_odr_hz_table) / sizeof(g_odr_hz_table[0])))
    {
      hdr_odr_hz = g_odr_hz_table[odr_idx];
    }
  if (xl_idx < (sizeof(g_fsr_xl_g_table) / sizeof(g_fsr_xl_g_table[0])))
    {
      hdr_xl_g = g_fsr_xl_g_table[xl_idx];
    }
  if (gy_idx < (sizeof(g_fsr_gy_dps_table) / sizeof(g_fsr_gy_dps_table[0])))
    {
      hdr_gy_dps = g_fsr_gy_dps_table[gy_idx];
    }

  /* Take a sensor snapshot.  Each call clears FRESH on the sampler so
   * if we emit several frames in one tick (idx-split case), only the
   * first frame carries fresh sensor payload — subsequent frames see
   * FRESH=false and skip the per-sensor payload bytes.  This matches
   * the wire format contract (TLV count is fixed at 6 every frame).
   */

  struct sensor_class_state_s sensor_state[BTSENSOR_TLV_COUNT];
  sensor_sampler_snapshot(sensor_state);

  /* Layout (envelope @ 0, header @ 5, IMU @ 21, TLVs @ ...). */

  uint8_t *p   = g_bundle_buf;
  uint16_t off = 0;

  put_u16le(p + 0, BTSENSOR_FRAME_MAGIC);
  p[2] = BTSENSOR_FRAME_TYPE_BUNDLE;
  off  = BTSENSOR_BUNDLE_ENVELOPE_SIZE;       /* 5 */

  uint8_t *hdr = p + off;
  put_u16le(hdr + 0, g_seq);
  put_u32le(hdr + 2, tick_ts_us);

  uint16_t imu_section_len = (uint16_t)(BTSENSOR_IMU_SAMPLE_SIZE * imu_count);
  put_u16le(hdr + 6, imu_section_len);
  hdr[8]  = (uint8_t)imu_count;
  hdr[9]  = (uint8_t)BTSENSOR_TLV_COUNT;
  put_u16le(hdr + 10, hdr_odr_hz);
  hdr[12] = hdr_xl_g;
  put_u16le(hdr + 13, hdr_gy_dps);

  uint8_t flags = 0;
  if (g_imu_on)
    {
      flags |= BTSENSOR_FLAG_IMU_ON;
    }

  if (g_sensor_on)
    {
      flags |= BTSENSOR_FLAG_SENSOR_ON;
    }

  hdr[15] = flags;
  off += BTSENSOR_BUNDLE_HEADER_SIZE;         /* 21 */

  /* IMU subsection. */

  for (size_t i = 0; i < imu_count; i++)
    {
      uint32_t delta = imu_samples[i].timestamp - tick_ts_us;  /* mod 2^32 */
      encode_imu_sample(p + off, &imu_samples[i], delta);
      off += BTSENSOR_IMU_SAMPLE_SIZE;
    }

  /* TLV subsection. */

  for (uint8_t cls = 0; cls < BTSENSOR_TLV_COUNT; cls++)
    {
      off += (uint16_t)encode_tlv(p + off, cls, &sensor_state[cls]);
    }

  /* Patch frame_len now that we know it. */

  put_u16le(p + 3, off);

  g_seq++;

  return btsensor_tx_try_enqueue_frame(g_bundle_buf, off);
}

/* Issue #139: drain a tickful of IMU samples and emit one BUNDLE frame
 * per maximal homogeneous run (matching odr_idx / fsr_xl_idx /
 * fsr_gy_idx).  In steady state (no live SET) this collapses to a
 * single frame per tick, so the 100 Hz pacing is preserved; right
 * after a SET, a tick may emit 2-3 frames back-to-back so each frame's
 * BUNDLE header rate/fsr fields stay consistent with its sample idx.
 * The TX arbiter ring absorbs the burst.
 */

static int emit_bundle(void)
{
  if (!g_imu_on)
    {
      return emit_one_frame(NULL, 0);
    }

  struct sensor_imu imu_buf[BTSENSOR_IMU_SAMPLES_MAX];
  size_t imu_count = imu_sampler_drain(imu_buf, BTSENSOR_IMU_SAMPLES_MAX);

  if (imu_count == 0)
    {
      return emit_one_frame(NULL, 0);     /* last-known idx fallback */
    }

  /* Walk the drained samples and split at every idx boundary.  Most
   * ticks see a single homogeneous run so the outer while loop emits
   * exactly one frame.
   */

  size_t start = 0;
  int first_rc = 0;
  while (start < imu_count)
    {
      size_t end = start + 1;
      while (end < imu_count
             && imu_buf[end].odr_idx    == imu_buf[start].odr_idx
             && imu_buf[end].fsr_xl_idx == imu_buf[start].fsr_xl_idx
             && imu_buf[end].fsr_gy_idx == imu_buf[start].fsr_gy_idx)
        {
          end++;
        }

      int rc = emit_one_frame(&imu_buf[start], end - start);
      if (rc < 0 && first_rc == 0)
        {
          first_rc = rc;                  /* report first non-zero rc */
        }

      start = end;
    }

  return first_rc;
}

/****************************************************************************
 * Private Functions — timer plumbing
 ****************************************************************************/

static void tick_handler(btstack_timer_source_t *ts);

static void timer_arm(void)
{
  if (g_timer_armed)
    {
      return;
    }

  btstack_run_loop_set_timer_handler(&g_tick_timer, tick_handler);
  btstack_run_loop_set_timer(&g_tick_timer,
                             CONFIG_APP_BTSENSOR_BUNDLE_TICK_MS);
  btstack_run_loop_add_timer(&g_tick_timer);
  g_timer_armed = true;
}

static void timer_disarm(void)
{
  if (!g_timer_armed)
    {
      return;
    }

  btstack_run_loop_remove_timer(&g_tick_timer);
  g_timer_armed = false;
}

static void tick_handler(btstack_timer_source_t *ts)
{
  (void)ts;

  /* Both off → don't re-arm.  Whoever flipped one back on (run loop
   * thread) will call timer_arm() at that point.
   */

  if (!g_imu_on && !g_sensor_on)
    {
      g_timer_armed = false;            /* timer fired and is now idle */
      return;
    }

  /* Only emit when a PC is actively listening — otherwise the TX ring
   * fills up immediately and drops every frame, inflating drop counters
   * for no good reason.  We still drain the IMU side ring so it does
   * not overflow silently while waiting for a consumer; sensor_sampler
   * is event-driven via btstack data sources, so its cache stays fresh
   * on its own.
   */

  if (btsensor_tx_has_consumer())
    {
      (void)emit_bundle();
    }
  else if (g_imu_on)
    {
      struct sensor_imu drop_buf[BTSENSOR_IMU_SAMPLES_MAX];
      while (imu_sampler_drain(drop_buf, BTSENSOR_IMU_SAMPLES_MAX) > 0)
        {
          /* discard while no PC is connected */
        }
    }

  /* Re-arm for the next tick. */

  btstack_run_loop_set_timer(&g_tick_timer,
                             CONFIG_APP_BTSENSOR_BUNDLE_TICK_MS);
  btstack_run_loop_add_timer(&g_tick_timer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bundle_emitter_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  memset(&g_tick_timer, 0, sizeof(g_tick_timer));
  g_imu_on       = false;
  g_sensor_on    = false;
  g_timer_armed  = false;
  g_seq          = 0;
  g_initialized  = true;
  return 0;
}

void bundle_emitter_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  /* Stop the tick first so no callback can race the imu/sensor sampler
   * teardown that follows in btsensor_main.c (Codex review #5).
   */

  timer_disarm();

  if (g_imu_on)
    {
      imu_sampler_set_enabled(false);
      g_imu_on = false;
    }

  if (g_sensor_on)
    {
      sensor_sampler_set_enabled(false);
      g_sensor_on = false;
    }

  g_initialized = false;
}

int bundle_emitter_set_imu_enabled(bool on)
{
  if (!g_initialized)
    {
      return -EINVAL;
    }

  if (on == g_imu_on)
    {
      return 0;
    }

  int rc = imu_sampler_set_enabled(on);
  if (rc != 0)
    {
      return rc;
    }

  g_imu_on = on;
  if (on)
    {
      timer_arm();
    }
  else if (!g_sensor_on)
    {
      timer_disarm();
    }

  return 0;
}

int bundle_emitter_set_sensor_enabled(bool on)
{
  if (!g_initialized)
    {
      return -EINVAL;
    }

  if (on == g_sensor_on)
    {
      return 0;
    }

  int rc = sensor_sampler_set_enabled(on);
  if (rc != 0)
    {
      return rc;
    }

  g_sensor_on = on;
  if (on)
    {
      timer_arm();
    }
  else if (!g_imu_on)
    {
      timer_disarm();
    }

  return 0;
}

bool bundle_emitter_is_imu_enabled(void)
{
  return g_imu_on;
}

bool bundle_emitter_is_sensor_enabled(void)
{
  return g_sensor_on;
}
