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

/* Diagnostic counters for the BT-side NSH shell TX path.  Read by
 * `btsensor diag` from a separate USB-NSH task; written from the BTstack
 * thread (send-side fields) and the reader pthread (drop-side fields).
 * No locking on read — counters are word-aligned uint32_t so a torn
 * read at most loses last update, which is acceptable for measurement.
 *
 * Originally added during #109 follow-up to localize a CC2564C ACL TX
 * queue stall (Issue #54) — kept after fix as an operational health
 * window into the shell-mode TX pipe.
 */

struct btsensor_shell_diag_s
{
  /* reader pthread → tx_buf */

  uint32_t reader_appends;           /* successful append rounds */
  uint32_t reader_drops;              /* rounds where copy < n */
  uint32_t drop_bytes_total;          /* total bytes dropped (cumulative) */
  uint32_t tx_buf_high_water;         /* peak tx_len observed */

  /* on_can_send_now → rfcomm_send */

  uint32_t can_send_now_calls;        /* btstack callback fires */
  uint32_t send_ok;                   /* rfcomm_send returned 0 */
  uint32_t send_no_credit;            /* returned 0x72 NO_OUTGOING_CREDITS */
  uint32_t send_exceeds_mtu;          /* returned 0x74 (invariant break) */
  uint32_t send_other_err;            /* any other non-zero rc */
  uint32_t send_zero_mtu;             /* mtu==0 early return */
  uint8_t  last_send_err;             /* last non-zero rc from rfcomm_send */
  uint16_t last_send_len;             /* len of the last rfcomm_send attempt */
  uint16_t last_send_mtu;             /* mtu observed at the last attempt */

  /* pump timer */

  uint32_t pump_arms;                 /* shell_arm_pump_timer entries */
  uint32_t pump_fires;                /* timer handler fires */
  uint32_t pump_fires_no_data;        /* fired but tx_len==0 */
  uint32_t pump_fires_no_cid;         /* fired but cid==0 */

  /* HCI / RFCOMM probe at each pump_timer fire with data pending.
   * Surfaces an ACL-stall recurrence as `hci_blocked` rising while
   * `acl_free` stays low.
   */

  uint32_t probe_hci_blocked;         /* hci_can_send_acl_classic_packet_now()==false */
  uint8_t  probe_last_hci_now;        /* hci_can_send_acl_classic_packet_now at last probe */
  uint16_t probe_last_acl_buf_len;    /* hci_max_acl_data_packet_length() */
  uint16_t probe_last_acl_free;       /* hci_number_free_acl_slots_for_connection_type(ACL) */
  uint16_t probe_last_rfcomm_mtu;     /* rfcomm_get_max_frame_size() */

  /* peer-side ACK observation via HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS */

  uint32_t hci_completed_events;      /* event-handler invocations */
  uint32_t hci_completed_packets;     /* total packets reported completed */
  uint32_t hci_completed_last_count;  /* the last per-event packet count */
};

/* btstack hci packet handler hook for the host-side
 * HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS event.  Drives the
 * `hci_completed_*` counters.  Called from the BTstack thread.
 */

void btsensor_shell_on_hci_completed_packets(uint16_t packets);

void btsensor_shell_get_diag(struct btsensor_shell_diag_s *out);

/* Reset counters.  Useful before a fresh reproduction run so the
 * caller can see a clean delta.
 */

void btsensor_shell_reset_diag(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_SHELL_H */
