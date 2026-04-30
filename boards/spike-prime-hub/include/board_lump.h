/****************************************************************************
 * boards/spike-prime-hub/include/board_lump.h
 *
 * Public ABI for the SPIKE Prime Hub LUMP UART protocol engine (Issue #43).
 *
 * After Issue #42's DCM (`/dev/legoport[0-5]`) confirms `UNKNOWN_UART (14)`
 * on a port, the LUMP engine takes ownership of the corresponding UART
 * (UART7/4/8/5/10/9 for ports A..F) and drives the LEGO UART Messaging
 * Protocol state machine: speed negotiation, type / mode-info exchange,
 * 100 ms `SYS_NACK` keepalive, and bidirectional `DATA` frames.
 *
 * Consumers are the upcoming motor (#44, `/dev/legomotor*`) and sensor
 * (#45, `/dev/legosensor*`) drivers.  This header is intentionally
 * separate from `board_legoport.h` so they only depend on the LUMP API
 * and not on the DCM ABI.
 *
 * Threading: one kernel thread per port (pre-created at boot,
 * SCHED_PRIORITY_DEFAULT, 2 KB stack) drives each port's state machine.
 * Public API is thread-safe across ports; per-port serialisation is by
 * the engine.  Callbacks (`on_sync` / `on_data` / `on_error`) are invoked
 * from the per-port kthread context with the per-port lock released —
 * but **same-port re-entry from inside a callback is forbidden** and
 * returns `-EDEADLK`.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LUMP_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LUMP_H

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LUMP_MAX_MODES           8     /* devices we care about cap at 8 */
#define LUMP_MAX_NAME_LEN        12    /* LUMP_MAX_NAME_SIZE from pybricks */
#define LUMP_MAX_UNITS_LEN       4
#define LUMP_MAX_PAYLOAD         32    /* LUMP frame payload cap */

/* `lump_device_info_s::flags` */

#define LUMP_FLAG_SYNCED         (1u << 0)  /* SYNC complete; modes valid */
#define LUMP_FLAG_DATA_OK        (1u << 1)  /* Last DATA < keepalive ago */
#define LUMP_FLAG_ERROR          (1u << 2)  /* Engine in ERR state */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum lump_data_type_e
{
  LUMP_DATA_INT8  = 0,
  LUMP_DATA_INT16 = 1,
  LUMP_DATA_INT32 = 2,
  LUMP_DATA_FLOAT = 3,
};

struct lump_mode_info_s
{
  char     name[LUMP_MAX_NAME_LEN + 1]; /* zero-terminated */
  uint8_t  num_values;                  /* INFO_FORMAT [2] */
  uint8_t  data_type;                   /* enum lump_data_type_e */
  uint8_t  writable;                    /* INFO_MAPPING bit set */
  uint8_t  reserved;
  float    raw_min, raw_max;
  float    pct_min, pct_max;
  float    si_min,  si_max;
  char     units[LUMP_MAX_UNITS_LEN + 1];
};

struct lump_device_info_s
{
  uint8_t  type_id;             /* LPF2 type once LUMP IDs the device */
  uint8_t  num_modes;
  uint8_t  current_mode;
  uint8_t  flags;               /* LUMP_FLAG_* */
  uint32_t baud;                /* current line rate */
  uint32_t fw_version;
  uint32_t hw_version;
  struct lump_mode_info_s modes[LUMP_MAX_MODES];
};

/* DATA frame snapshot returned by `LEGOPORT_LUMP_POLL_DATA` ioctl
 * (Issue #43 Phase 3).  `len` is the actual payload size in bytes
 * (1..LUMP_MAX_PAYLOAD); `data[len..]` is unused.  `mode` is the
 * device's reporting mode at the time the frame was received.
 */

struct lump_data_frame_s
{
  uint8_t  mode;
  uint8_t  len;
  uint8_t  reserved[2];
  uint8_t  data[LUMP_MAX_PAYLOAD];
};

#define LUMP_DATA_QUEUE   16    /* engine-side DATA frame ring depth */

/* Callback invoked once on SYNCING -> DATA transition.  `info` is owned
 * by the engine and remains valid until the next SYNC cycle on the same
 * port.  Caller may snapshot fields they need.
 */

typedef void (*lump_on_sync_t)(int port,
                               const struct lump_device_info_s *info,
                               void *priv);

/* Callback invoked for each DATA frame.  `data`/`len` are stack-borrowed —
 * copy if you need them past return.  `mode` is the current mode at the
 * time the frame was received.
 */

typedef void (*lump_on_data_t)(int port, uint8_t mode,
                               const uint8_t *data, size_t len,
                               void *priv);

struct lump_callbacks_s
{
  lump_on_sync_t on_sync;
  lump_on_data_t on_data;
  lump_on_data_t on_error;   /* nullable */
  void          *priv;
};

/****************************************************************************
 * Public API
 ****************************************************************************/

/* Attach the consumer (motor/sensor driver) to a port.  If the port is
 * already SYNCED, `on_sync` fires synchronously before this returns
 * (lock-released path; same-port re-entry is forbidden).  Returns 0 on
 * success, `-EINVAL` if `port` out of range, `-EBUSY` if another consumer
 * already attached, `-EDEADLK` if called from inside a same-port callback.
 */

int lump_attach(int port, const struct lump_callbacks_s *cb);

/* Detach — clears callbacks atomically.  Pending callback dispatches
 * complete before this returns.  Idempotent.
 */

int lump_detach(int port);

/* Request a mode switch (`CMD SELECT`).  The kthread sends the command
 * on its next TX window; the consumer can detect arrival via `on_data`
 * with the new mode index.
 */

int lump_select_mode(int port, uint8_t mode);

/* Send a writable-mode payload (`DATA` with mode in header).  `mode`
 * MUST currently be selected and writable; otherwise `-EINVAL` /
 * `-ENOTSUP` is returned.
 */

int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len);

/* Snapshot the device info into `out`.  Returns `-EAGAIN` if SYNC has
 * not completed yet on this port.
 */

int lump_get_info(int port, struct lump_device_info_s *out);

/* Lightweight status snapshot — engine flags + traffic counters.  Either
 * `flags_out` / `rx_bytes_out` / `tx_bytes_out` may be NULL.
 */

int lump_get_status(int port, uint8_t *flags_out,
                    uint32_t *rx_bytes_out,
                    uint32_t *tx_bytes_out);

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LUMP_H */
