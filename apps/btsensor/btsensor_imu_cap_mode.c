/****************************************************************************
 * apps/btsensor/btsensor_imu_cap_mode.c
 *
 * IMU_CAP mode handler — see btsensor_imu_cap_mode.h.
 *
 * Architecture:
 *   - enter() switches the LSM6DSL driver to 104 Hz, opens
 *     /dev/uorb/sensor_imu0 O_RDONLY|O_NONBLOCK, pauses BUNDLE
 *     emitters, and attaches the fd to a btstack data source so the
 *     run-loop poll wakes us per sample.
 *   - on_read() reads one struct sensor_imu per callback (matching the
 *     uORB driver's per-record I/O), encodes it as a 27 B IMU_CAP
 *     frame (envelope + 22 B payload), and forwards via
 *     btsensor_tx_try_enqueue_frame().  5 ms throttle paces the next
 *     read so back-to-back enqueues do not burst-flood the CC2564C
 *     ACL TX queue (Issue #54 / #109 root cause).
 *   - Drop policy: no back-pressure check.  btsensor_capture_mode
 *     can back-pressure because /dev/btcap is a chardev with a real
 *     writer that blocks on a full pipe, so skipping reads is
 *     lossless.  IMU_CAP reads /dev/uorb/sensor_imu0 whose upper-half
 *     ring overwrites old samples (circbuf_overwrite), so skipping
 *     reads does *not* preserve data — it just hides the loss.  We
 *     always enqueue and let btsensor_tx_try_enqueue_frame's
 *     drop-oldest policy handle tx-ring overflow.  seq advances on
 *     every emit, even on -ENOSPC, so the host sees explicit gaps
 *     and rejects the file (plan Phase 2.5 §B: host accepts only
 *     seq-drop=0 sessions).
 *   - exit() always restores ODR to 833 Hz regardless of how we get
 *     here (explicit stop, duration timeout, error path).  This is
 *     layer 1 of the 3-layer ODR rollback defense (see
 *     [[project_phase_2_5_plan]]).
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

#include "btstack_run_loop.h"

#include <arch/board/board_lsm6dsl.h>

#include "btsensor_imu_cap_mode.h"
#include "btsensor_tx.h"
#include "btsensor_wire.h"
#include "bundle_emitter.h"
#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_CAP_DEVPATH        "/dev/uorb/sensor_imu0"
#define IMU_CAP_ODR_HZ         104   /* Tedaldi static-detect ODR */
#define IMU_CAP_ODR_RESTORE    833   /* drivebase default, always set on exit */
#define IMU_CAP_THROTTLE_MS    5     /* matches btsensor_capture_mode */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imu_cap_state_s
{
  bool                   active;
  bool                   ds_registered;
  bool                   ds_callbacks_on;
  bool                   throttle_armed;
  bool                   duration_armed;
  bool                   paused_imu;
  bool                   paused_sensor;
  int                    fd;
  uint16_t               seq;
  uint32_t               frames_sent;
  uint32_t               frames_dropped;
  /* Diagnostic counters — logged on exit so we can tell on_read
   * activity from sensor publish rate.  Hard to debug remotely
   * without these since the hot path runs on the btstack thread.
   */
  uint32_t               diag_on_read_calls;
  uint32_t               diag_read_eagain;
  uint32_t               diag_read_short;
  uint32_t               diag_read_ok;
  btstack_data_source_t  ds;
  btstack_timer_source_t throttle_timer;
  btstack_timer_source_t duration_timer;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imu_cap_state_s g_imu_cap;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static void imu_cap_throttle_disable_callbacks(void);
static void imu_cap_throttle_arm(void);
static void imu_cap_throttle_timer_handler(btstack_timer_source_t *ts);
static void imu_cap_duration_timer_handler(btstack_timer_source_t *ts);
static void imu_cap_on_read(btstack_data_source_t *ds,
                            btstack_data_source_callback_type_t type);
static int  imu_cap_exit_locked(void);

/****************************************************************************
 * Private Functions — little-endian encoders
 ****************************************************************************/

static inline void put_u16le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xff);
  p[1] = (uint8_t)((v >> 8) & 0xff);
}

static inline void put_u32le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xff);
  p[1] = (uint8_t)((v >> 8) & 0xff);
  p[2] = (uint8_t)((v >> 16) & 0xff);
  p[3] = (uint8_t)((v >> 24) & 0xff);
}

static inline void put_s16le(uint8_t *p, int16_t v)
{
  put_u16le(p, (uint16_t)v);
}

/****************************************************************************
 * Private Functions — throttle / duration timers
 ****************************************************************************/

static void imu_cap_throttle_disable_callbacks(void)
{
  if (g_imu_cap.ds_registered && g_imu_cap.ds_callbacks_on)
    {
      btstack_run_loop_disable_data_source_callbacks(
          &g_imu_cap.ds, DATA_SOURCE_CALLBACK_READ);
      g_imu_cap.ds_callbacks_on = false;
    }
}

static void imu_cap_throttle_arm(void)
{
  if (g_imu_cap.throttle_armed)
    {
      return;
    }

  btstack_run_loop_remove_timer(&g_imu_cap.throttle_timer);
  btstack_run_loop_set_timer_handler(&g_imu_cap.throttle_timer,
                                     imu_cap_throttle_timer_handler);
  btstack_run_loop_set_timer(&g_imu_cap.throttle_timer, IMU_CAP_THROTTLE_MS);
  btstack_run_loop_add_timer(&g_imu_cap.throttle_timer);
  g_imu_cap.throttle_armed = true;
}

static void imu_cap_throttle_timer_handler(btstack_timer_source_t *ts)
{
  (void)ts;
  g_imu_cap.throttle_armed = false;

  if (!g_imu_cap.active || g_imu_cap.fd < 0 || !g_imu_cap.ds_registered)
    {
      return;
    }

  if (!g_imu_cap.ds_callbacks_on)
    {
      btstack_run_loop_enable_data_source_callbacks(
          &g_imu_cap.ds, DATA_SOURCE_CALLBACK_READ);
      g_imu_cap.ds_callbacks_on = true;
    }
}

static void imu_cap_duration_timer_handler(btstack_timer_source_t *ts)
{
  (void)ts;
  g_imu_cap.duration_armed = false;
  syslog(LOG_INFO, "btsensor: imu_cap duration reached, exiting\n");
  (void)imu_cap_exit_locked();
}

/****************************************************************************
 * Private Functions — frame encode + read callback
 ****************************************************************************/

static int imu_cap_emit_sample(const struct sensor_imu *s)
{
  uint8_t buf[BTSENSOR_IMU_CAP_FRAME_SIZE];

  /* Envelope (5 B): magic + frame_type + frame_len. */

  put_u16le(&buf[0], BTSENSOR_FRAME_MAGIC);
  buf[2] = BTSENSOR_FRAME_TYPE_IMU_CAP;
  put_u16le(&buf[3], (uint16_t)sizeof(buf));

  /* Payload (22 B). */

  uint8_t *p = &buf[BTSENSOR_BUNDLE_ENVELOPE_SIZE];
  put_u32le(p +  0, s->timestamp);
  put_s16le(p +  4, s->ax);
  put_s16le(p +  6, s->ay);
  put_s16le(p +  8, s->az);
  put_s16le(p + 10, s->gx);
  put_s16le(p + 12, s->gy);
  put_s16le(p + 14, s->gz);
  put_s16le(p + 16, s->temperature_raw);
  p[18] = s->fsr_xl_idx;
  p[19] = s->fsr_gy_idx;
  put_u16le(p + 20, g_imu_cap.seq);

  /* Always advance seq, regardless of whether btsensor_tx had to drop
   * an older frame to make room.  btsensor_tx_try_enqueue_frame writes
   * the new frame either way (it drops oldest on overflow, returns
   * -ENOSPC, but the new frame is still in the ring).  If the seq
   * stayed put on ENOSPC, the next call would encode the same seq
   * value into a different sample's frame — the host would see a
   * duplicate seq, fail to detect the gap, and silently accept a file
   * with missing samples (the very condition the host reject path is
   * supposed to catch).
   */

  int rc = btsensor_tx_try_enqueue_frame(buf, sizeof(buf));
  g_imu_cap.seq++;
  if (rc == 0)
    {
      g_imu_cap.frames_sent++;
      return 0;
    }

  g_imu_cap.frames_dropped++;
  return -EAGAIN;
}

static void imu_cap_on_read(btstack_data_source_t *ds,
                            btstack_data_source_callback_type_t type)
{
  (void)ds;
  (void)type;

  g_imu_cap.diag_on_read_calls++;

  if (!g_imu_cap.active || g_imu_cap.fd < 0)
    {
      return;
    }

  /* No back-pressure check.  An earlier draft skipped the read while
   * btsensor_tx was at capacity (same pattern as
   * btsensor_capture_mode), but that pattern only works when a BT
   * peer is actively draining the tx ring.  IMU_CAP runs at 104 Hz
   * for ~minutes during a Tedaldi session — if the host disconnects
   * (or the bench has no peer at all) the ring fills in ~0.3 s and
   * we would stall the data source indefinitely.  Instead we always
   * enqueue and rely on btsensor_tx_try_enqueue_frame's drop-oldest
   * policy on overflow; the seq counter advances unconditionally so
   * the host sees explicit gaps and rejects the file (see plan
   * Phase 2.5 D — host accepts only seq-drop=0 sessions).
   */

  struct sensor_imu s;
  ssize_t n = read(g_imu_cap.fd, &s, sizeof(s));
  if (n != (ssize_t)sizeof(s))
    {
      if (n < 0 && errno == EAGAIN)
        {
          g_imu_cap.diag_read_eagain++;
        }
      else if (n < 0)
        {
          syslog(LOG_WARNING, "btsensor: imu_cap read errno %d\n", errno);
        }
      else
        {
          g_imu_cap.diag_read_short++;
        }

      return;
    }

  g_imu_cap.diag_read_ok++;
  (void)imu_cap_emit_sample(&s);

  /* Pace the next read so multiple frames per run-loop iteration
   * don't burst-flood the controller.  Throttle re-enables the data
   * source poll after IMU_CAP_THROTTLE_MS.  At 104 Hz the throttle
   * (5 ms) is shorter than the sample period (~9.6 ms) so sample
   * arrival is the actual bottleneck, not the throttle.
   */

  imu_cap_throttle_disable_callbacks();
  imu_cap_throttle_arm();
}

/****************************************************************************
 * Private Functions — teardown
 ****************************************************************************/

static int imu_cap_exit_locked(void)
{
  if (!g_imu_cap.active)
    {
      return 0;
    }

  if (g_imu_cap.throttle_armed)
    {
      btstack_run_loop_remove_timer(&g_imu_cap.throttle_timer);
      g_imu_cap.throttle_armed = false;
    }

  if (g_imu_cap.duration_armed)
    {
      btstack_run_loop_remove_timer(&g_imu_cap.duration_timer);
      g_imu_cap.duration_armed = false;
    }

  if (g_imu_cap.ds_registered)
    {
      btstack_run_loop_remove_data_source(&g_imu_cap.ds);
      g_imu_cap.ds_registered   = false;
      g_imu_cap.ds_callbacks_on = false;
    }

  if (g_imu_cap.fd >= 0)
    {
      close(g_imu_cap.fd);
      g_imu_cap.fd = -1;
    }

  /* Layer 1 of the 3-layer ODR rollback: always restore 833 Hz on the
   * way out, regardless of which path led here.  Layer 2 (drivebase
   * start force) and layer 3 (BT-side re-entry guard) cover the
   * crash / kill paths where this exit_locked never runs.
   */

  int rc_odr = imu_sampler_set_odr_hz(IMU_CAP_ODR_RESTORE);
  if (rc_odr < 0)
    {
      syslog(LOG_WARNING,
             "btsensor: imu_cap ODR restore (%d Hz) failed rc=%d\n",
             IMU_CAP_ODR_RESTORE, rc_odr);
    }

  /* Restore BUNDLE emitters to whatever they were before we paused. */

  if (g_imu_cap.paused_imu)
    {
      (void)bundle_emitter_set_imu_enabled(true);
      g_imu_cap.paused_imu = false;
    }

  if (g_imu_cap.paused_sensor)
    {
      (void)bundle_emitter_set_sensor_enabled(true);
      g_imu_cap.paused_sensor = false;
    }

  syslog(LOG_INFO,
         "btsensor: imu_cap exited (frames sent=%u dropped=%u "
         "on_read=%u eagain=%u short=%u read_ok=%u)\n",
         (unsigned)g_imu_cap.frames_sent,
         (unsigned)g_imu_cap.frames_dropped,
         (unsigned)g_imu_cap.diag_on_read_calls,
         (unsigned)g_imu_cap.diag_read_eagain,
         (unsigned)g_imu_cap.diag_read_short,
         (unsigned)g_imu_cap.diag_read_ok);

  g_imu_cap.active             = false;
  g_imu_cap.seq                = 0;
  g_imu_cap.frames_sent        = 0;
  g_imu_cap.frames_dropped     = 0;
  g_imu_cap.diag_on_read_calls = 0;
  g_imu_cap.diag_read_eagain   = 0;
  g_imu_cap.diag_read_short    = 0;
  g_imu_cap.diag_read_ok       = 0;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btsensor_imu_cap_mode_init(void)
{
  memset(&g_imu_cap, 0, sizeof(g_imu_cap));
  g_imu_cap.fd = -1;
  return 0;
}

void btsensor_imu_cap_mode_deinit(void)
{
  (void)imu_cap_exit_locked();
}

bool btsensor_imu_cap_mode_is_active(void)
{
  return g_imu_cap.active;
}

int btsensor_imu_cap_mode_enter(uint32_t duration_sec)
{
  if (g_imu_cap.active)
    {
      return -EBUSY;
    }

  /* Switch ODR to 104 Hz first so the topic opens with the new rate
   * already in effect.  The driver may already be active through
   * bundle_emitter / drivebase; #139 made live SET legal so this just
   * works without unsubscribing them.  We will pause bundle_emitter
   * below anyway to keep the BT byte stream IMU_CAP-only.
   */

  int rc = imu_sampler_set_odr_hz(IMU_CAP_ODR_HZ);
  if (rc < 0)
    {
      syslog(LOG_ERR,
             "btsensor: imu_cap ODR set (%d Hz) failed rc=%d\n",
             IMU_CAP_ODR_HZ, rc);
      return rc;
    }

  int fd = open(IMU_CAP_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      int err = errno;
      (void)imu_sampler_set_odr_hz(IMU_CAP_ODR_RESTORE);
      syslog(LOG_ERR, "btsensor: imu_cap open(%s) errno %d\n",
             IMU_CAP_DEVPATH, err);
      return -err;
    }

  /* Pause BUNDLE emitters so the BT byte stream contains only IMU_CAP
   * frames.  Track each so exit restores the pre-pause state.
   */

  if (bundle_emitter_is_imu_enabled())
    {
      (void)bundle_emitter_set_imu_enabled(false);
      g_imu_cap.paused_imu = true;
    }

  if (bundle_emitter_is_sensor_enabled())
    {
      (void)bundle_emitter_set_sensor_enabled(false);
      g_imu_cap.paused_sensor = true;
    }

  g_imu_cap.fd             = fd;
  g_imu_cap.seq            = 0;
  g_imu_cap.frames_sent    = 0;
  g_imu_cap.frames_dropped = 0;
  g_imu_cap.active         = true;

  btstack_run_loop_set_data_source_fd(&g_imu_cap.ds, fd);
  btstack_run_loop_set_data_source_handler(&g_imu_cap.ds, imu_cap_on_read);
  btstack_run_loop_add_data_source(&g_imu_cap.ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_imu_cap.ds, DATA_SOURCE_CALLBACK_READ);
  g_imu_cap.ds_callbacks_on = true;
  g_imu_cap.ds_registered   = true;

  if (duration_sec > 0)
    {
      btstack_run_loop_set_timer_handler(&g_imu_cap.duration_timer,
                                         imu_cap_duration_timer_handler);
      btstack_run_loop_set_timer(&g_imu_cap.duration_timer,
                                 duration_sec * 1000);
      btstack_run_loop_add_timer(&g_imu_cap.duration_timer);
      g_imu_cap.duration_armed = true;
    }

  syslog(LOG_INFO, "btsensor: imu_cap entered (duration_sec=%u, odr=%d)\n",
         (unsigned)duration_sec, IMU_CAP_ODR_HZ);
  return 0;
}

int btsensor_imu_cap_mode_exit(void)
{
  return imu_cap_exit_locked();
}
