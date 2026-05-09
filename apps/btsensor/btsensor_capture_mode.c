/****************************************************************************
 * apps/btsensor/btsensor_capture_mode.c
 *
 * MODE CAPTURE handler for the BT-side capture forwarder (Issue #122).
 * Drains /dev/btcap and pushes the resulting byte stream over RFCOMM
 * with BTCS + meta + payload + (BTCE | BTAB) framing so the C# host
 * tool can demux capture sessions out of the SPP byte stream.
 *
 * Architecture:
 *   - btsensor_capture_mode_enter() opens /dev/btcap O_RDONLY|O_NONBLOCK,
 *     fetches the writer-registered session metadata, sends the BTCS
 *     header, then attaches the fd to a btstack data source so the
 *     run-loop poll wakes us whenever bytes land.
 *   - on_read() reads up to MAX_CHUNK bytes per callback and forwards
 *     them through btsensor_tx_try_enqueue_frame().  When read() returns
 *     zero (EOF, writer-finalized + drained) we close the session with
 *     BTCE; when QUERY_STATE reports ABORTED we close with BTAB.
 *   - The IMU / sensor BUNDLE emitters are paused for the lifetime of
 *     the session so the BT byte stream contains only capture framing.
 *
 * The drop-oldest behaviour of btsensor_tx is acceptable here because
 * (a) telemetry is paused so the ring stays nearly empty, and (b) the
 * /dev/btcap pipe-style ring back-pressures the writer the moment our
 * reader callback stalls.  A fully lossless paced sender (NFR-9) is a
 * follow-up optimisation tracked separately.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

#include "btstack_run_loop.h"

#include <arch/board/board_btcap.h>

#include "btsensor_capture_mode.h"
#include "btsensor_tx.h"
#include "bundle_emitter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAP_CHUNK_BYTES        256

static const uint8_t BTCS_MAGIC[4] = { 'B', 'T', 'C', 'S' };
static const uint8_t BTCE_MAGIC[4] = { 'B', 'T', 'C', 'E' };
static const uint8_t BTAB_MAGIC[4] = { 'B', 'T', 'A', 'B' };

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cap_state_s
{
  bool                          active;
  bool                          ds_registered;
  bool                          paused_imu;
  bool                          paused_sensor;
  int                           fd;
  uint32_t                      total_bytes;
  uint32_t                      bytes_sent;
  btstack_data_source_t         ds;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cap_state_s g_cap;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static void cap_send_marker(const uint8_t magic[4]);
static void cap_exit_locked(void);
static void on_read(btstack_data_source_t *ds,
                    btstack_data_source_callback_type_t type);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cap_send_marker(const uint8_t magic[4])
{
  /* Best-effort enqueue; if the ring is full we accept the drop because
   * the marker is only useful when the surrounding payload made it
   * through anyway, and the host scanner re-syncs on the next BTCS.
   */

  (void)btsensor_tx_try_enqueue_frame(magic, 4);
}

static void cap_exit_locked(void)
{
  if (!g_cap.active)
    {
      return;
    }

  if (g_cap.ds_registered)
    {
      btstack_run_loop_remove_data_source(&g_cap.ds);
      g_cap.ds_registered = false;
    }

  if (g_cap.fd >= 0)
    {
      close(g_cap.fd);
      g_cap.fd = -1;
    }

  /* Restore BUNDLE emitters to whatever they were before we paused. */

  if (g_cap.paused_imu)
    {
      bundle_emitter_set_imu_enabled(true);
      g_cap.paused_imu = false;
    }

  if (g_cap.paused_sensor)
    {
      bundle_emitter_set_sensor_enabled(true);
      g_cap.paused_sensor = false;
    }

  g_cap.active = false;
  g_cap.bytes_sent = 0;
  g_cap.total_bytes = 0;
}

static void on_read(btstack_data_source_t *ds,
                    btstack_data_source_callback_type_t type)
{
  (void)ds;
  (void)type;

  if (!g_cap.active || g_cap.fd < 0)
    {
      return;
    }

  uint8_t buf[CAP_CHUNK_BYTES];
  ssize_t n = read(g_cap.fd, buf, sizeof(buf));

  if (n > 0)
    {
      (void)btsensor_tx_try_enqueue_frame(buf, (size_t)n);
      g_cap.bytes_sent += (uint32_t)n;
      return;
    }

  if (n < 0)
    {
      if (errno == EAGAIN)
        {
          /* No data right now; data source will fire again. */

          return;
        }

      /* Read error — treat as truncated session. */

      cap_send_marker(BTAB_MAGIC);
      cap_exit_locked();
      return;
    }

  /* n == 0: end of stream.  Disambiguate normal end vs abort by
   * querying the chardev state — if the writer FINALIZE-d cleanly we
   * see IDLE here (ring drained, state reset), abort path leaves the
   * state at ABORTED until both fds close.
   */

  int32_t state = -1;
  (void)ioctl(g_cap.fd, BTCAPIOC_QUERY_STATE, (unsigned long)&state);

  if (state == BTCAP_STATE_ABORTED)
    {
      cap_send_marker(BTAB_MAGIC);
    }
  else
    {
      cap_send_marker(BTCE_MAGIC);
    }

  cap_exit_locked();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btsensor_capture_mode_init(void)
{
  memset(&g_cap, 0, sizeof(g_cap));
  g_cap.fd = -1;
  return 0;
}

void btsensor_capture_mode_deinit(void)
{
  cap_exit_locked();
}

bool btsensor_capture_mode_is_active(void)
{
  return g_cap.active;
}

int btsensor_capture_mode_enter(void)
{
  if (g_cap.active)
    {
      return -EBUSY;
    }

  int fd = open(BTCAP_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      int err = errno;
      syslog(LOG_ERR, "btsensor: capture: open(%s): %d\n",
             BTCAP_DEVPATH, err);
      return -err;
    }

  /* Refuse to engage if no session has been registered.  Apps that ask
   * for MODE CAPTURE without a writer in flight get an error reply
   * rather than parking the daemon.
   */

  struct btcap_session_meta_s meta;
  if (ioctl(fd, BTCAPIOC_GET_SESSION_META,
            (unsigned long)&meta) < 0)
    {
      int err = errno;
      close(fd);
      if (err != ENOENT)
        {
          syslog(LOG_ERR, "btsensor: capture: GET_META errno %d\n", err);
        }

      return -err;
    }

  /* Pause BUNDLE emitters so the BT byte stream contains only the
   * capture framing.  Remember whether we changed anything so exit can
   * restore the pre-pause state.
   */

  if (bundle_emitter_is_imu_enabled())
    {
      bundle_emitter_set_imu_enabled(false);
      g_cap.paused_imu = true;
    }

  if (bundle_emitter_is_sensor_enabled())
    {
      bundle_emitter_set_sensor_enabled(false);
      g_cap.paused_sensor = true;
    }

  g_cap.fd          = fd;
  g_cap.total_bytes = meta.total_bytes;
  g_cap.bytes_sent  = 0;
  g_cap.active      = true;

  /* Send the session header: BTCS magic immediately followed by the 38
   * byte meta blob (schema_magic u16 + reserved0 u16 + total_bytes u32
   * + name[32]).  Sized to fit one BT frame so the host receives them
   * atomically — no risk of an interleaved telemetry frame arriving
   * between BTCS and the meta because BUNDLE emission was paused above.
   */

  uint8_t header_buf[sizeof(BTCS_MAGIC) + sizeof(meta)];
  memcpy(&header_buf[0], BTCS_MAGIC, sizeof(BTCS_MAGIC));
  memcpy(&header_buf[sizeof(BTCS_MAGIC)], &meta, sizeof(meta));
  (void)btsensor_tx_try_enqueue_frame(header_buf, sizeof(header_buf));

  /* Hand the fd off to the run-loop poll. */

  btstack_run_loop_set_data_source_fd(&g_cap.ds, fd);
  btstack_run_loop_set_data_source_handler(&g_cap.ds, on_read);
  btstack_run_loop_add_data_source(&g_cap.ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_cap.ds, DATA_SOURCE_CALLBACK_READ);
  g_cap.ds_registered = true;

  return 0;
}
