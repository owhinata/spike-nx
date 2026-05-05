/****************************************************************************
 * apps/btsensor/btsensor_shell.h
 *
 * BT-side NSH shell mode for btsensor (Issue #108).
 *
 * Mutually exclusive with the telemetry-mode ASCII command parser:
 * `MODE SHELL\n` from the SPP peer parks the IMU/SENSOR pumps and
 * bridges the RFCOMM byte stream to a freshly-spawned NSH child
 * (btnsh_main) via two FIFOs (/dev/btnsh_in for stdin, /dev/btnsh_out
 * for stdout/stderr).  See docs/{ja,en}/development/bt-nsh-shell.md
 * for the wire-level protocol contract.
 *
 * All public functions other than init/deinit/is_active must run on
 * the BTstack main thread.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_SHELL_H
#define __APPS_BTSENSOR_BTSENSOR_SHELL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Mode visible to spp / cmd / main
 ****************************************************************************/

enum btsensor_mode_e
{
  BTSENSOR_MODE_TELEMETRY = 0,    /* Default — RFCOMM RX → cmd_feed */
  BTSENSOR_MODE_SHELL_STARTING,   /* shell child spawned, OK\n in flight */
  BTSENSOR_MODE_SHELL,            /* RFCOMM RX → NSH stdin */
};

/* Shell-exit reason codes passed to btsensor_shell_exit_async(). */

enum btsensor_shell_reason_e
{
  BTSENSOR_SHELL_REASON_NSH_EXIT = 0,
  BTSENSOR_SHELL_REASON_PEER_CLOSED,
  BTSENSOR_SHELL_REASON_USB_REQUEST,
  BTSENSOR_SHELL_REASON_DAEMON_STOP,
  BTSENSOR_SHELL_REASON_TX_DRAIN_TIMEOUT,
};

/****************************************************************************
 * Public API
 ****************************************************************************/

/* Daemon-startup hook.  Creates /dev/btnsh_in / /dev/btnsh_out via
 * mkfifo() (EEXIST treated as success).  Returns 0 on success, -errno
 * if a FIFO could not be created — in that case the daemon still
 * starts but `MODE SHELL` is rejected with `ERR shell_unavailable`.
 */

int  btsensor_shell_init(void);

/* Daemon-teardown hook.  Force-exits the shell if active, then unlinks
 * the FIFOs.  Idempotent.
 */

void btsensor_shell_deinit(void);

/* Returns true iff the shell is in STARTING or SHELL state.  Safe to
 * call from any thread (atomic read of an enum).
 */

bool btsensor_shell_is_active(void);

/* Current mode accessor.  btstack-thread-only. */

enum btsensor_mode_e btsensor_shell_get_mode(void);

/* Enter shell mode (BTstack thread only).  Steps:
 *   1. pipe(ctrl_pipe) for self-pipe shutdown wakeup.
 *   2. open btsensor-side FIFOs (stdin O_NONBLOCK, stdout blocking).
 *   3. open child-side FIFO ends + posix_spawn dup2 setup.
 *   4. task_spawn("btnsh", btnsh_main, ...).
 *   5. close parent's copies of child fds.
 *   6. pthread_create reader.
 *   7. set mode = SHELL_STARTING and bump the generation counter.
 *
 * All failure paths fully roll back any partial state.
 *
 * Returns 0 on success or -errno (mode stays TELEMETRY on failure).
 */

int  btsensor_shell_enter(void);

/* Async exit (BTstack thread only).  Tears down:
 *   - cancels post_drain callback / 500ms timer
 *   - SIGKILL the NSH child (always first, in case it's blocked in
 *     read() on stdin)
 *   - writes self-pipe to wake the reader
 *   - pthread_join the reader
 *   - blocking waitpid(nsh_pid, ...) to reap the child
 *   - close all four FIFO/self-pipe fds
 *   - reset cmd_feed line buffer
 *   - reason permitting, enqueue READY\n on the response queue
 *   - set mode = TELEMETRY and bump the generation counter.
 *
 * Idempotent — calling on an inactive shell is a no-op.
 */

void btsensor_shell_exit_async(enum btsensor_shell_reason_e reason);

/* RFCOMM payload from the peer routed here when mode == SHELL.
 * Performs a non-blocking write to /dev/btnsh_in; over-flow drops the
 * packet with a warning syslog.  STARTING-state packets must NOT
 * reach here (the spp packet handler drops them upstream).
 */

void btsensor_shell_on_rfcomm_data(const uint8_t *data, uint16_t len);

/* RFCOMM_EVENT_CAN_SEND_NOW handler.  Drains the pending tx_buf
 * (NSH stdout bytes accumulated by the reader pthread) via rfcomm_send
 * and re-arms the can-send-now request if more is queued.  No-op when
 * inactive.
 */

void btsensor_shell_on_can_send_now(void);

/* RFCOMM_EVENT_CHANNEL_CLOSED handler.  If the shell is active,
 * triggers exit_async(REASON_PEER_CLOSED).  No-op otherwise.
 */

void btsensor_shell_on_rfcomm_closed(void);

/* post_drain_callback target.  Invoked from btsensor_tx after the
 * MODE SHELL response queue has fully drained; flips
 * SHELL_STARTING → SHELL provided the generation matches (stale
 * callback detection) and the shell is still active.
 */

void btsensor_shell_transition_to_active(void *ctx);

/* post_drain timeout target.  Triggers exit_async(REASON_TX_DRAIN_TIMEOUT)
 * if still in STARTING.  No flip into SHELL on timeout — the pending
 * OK\n cannot be guaranteed to have reached the peer, so we tear
 * down rather than risk interleaving the shell stream.
 */

void btsensor_shell_drain_timeout(void *ctx);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_SHELL_H */
