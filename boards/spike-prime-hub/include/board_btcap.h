/****************************************************************************
 * boards/spike-prime-hub/include/board_btcap.h
 *
 * /dev/btcap character-device ABI for the spike-nx capture pipeline
 * (Issue #122).  This is a kernel-resident pipe-style channel that
 * carries lossless capture sessions from a userspace writer (e.g.
 * apps/sensor `do_capture`) to a userspace reader (the btsensor MODE
 * CAPTURE handler), which forwards the bytes over BT SPP.  The chardev
 * never buffers a whole capture session — writer and reader run in
 * lockstep through a small ring (default 1024 B) so RAM is bounded.
 *
 * Lifecycle (state machine, plus session_generation u32 monotonic):
 *
 *   IDLE ----REGISTER---->  READY ----FINALIZE+drain---> IDLE
 *                              \---ABORT/release-----> ABORTED -> IDLE
 *
 * READY covers both the writer-active and writer-finalized phases; an
 * internal `writer_done` flag distinguishes them so FINALIZE_SESSION can
 * be rejected after it has already been called.  External observers
 * only see IDLE / READY / ABORTED via QUERY_STATE.
 *
 * v1 contract:
 *   - Single instance (`/dev/btcap`), single session at a time.  A
 *     concurrent REGISTER returns -EBUSY.
 *   - There is no `BT-connected` precondition: REGISTER succeeds even
 *     when btsensor is in MODE SHELL or BT is not paired.  The writer
 *     simply blocks in write() until a reader engages (via `btsensor
 *     mode capture` or BT `MODE CAPTURE`).  If the writer no longer
 *     wants to wait, the user kills it; NuttX `nxsem_recover()` +
 *     `release` fop clean the session up automatically.
 *   - There is no timeout knob.  Block-forever-or-die is the only mode.
 *
 * ABI rules (32 vs 64-bit interop / FUSE portability):
 *   - Fixed-width types only.  No `long`, no pointer-in-struct, no
 *     `time_t`, no `bool`.
 *   - All structs are sized & laid out with `_Static_assert` below.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTCAP_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTCAP_H

#include <stddef.h>
#include <stdint.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions: device path
 ****************************************************************************/

#define BTCAP_DEVPATH                "/dev/btcap"

/****************************************************************************
 * Pre-processor Definitions: ioctl numbers
 ****************************************************************************/

#define _BTCAPBASE                   (0x4A00)
#define _BTCAPIOC(nr)                _IOC(_BTCAPBASE, nr)

/* Writer-side: open a session.  Allowed only in state IDLE; otherwise
 * returns -EBUSY.  On success the chardev transitions to READY.  No
 * payload bytes are written to the chardev as a side-effect of this
 * ioctl: the .cap file header is staged on the writer side and is only
 * pushed into the ring on the first successful write().
 */

#define BTCAPIOC_REGISTER_SESSION    _BTCAPIOC(0x01)  /* btcap_session_meta_s */

/* Writer-side: signal end-of-stream.  Allowed only when the session is
 * still being written to (state READY, writer_done == false).  The ring
 * is drained by the reader as usual; the session ends when the reader
 * has consumed everything.
 */

#define BTCAPIOC_FINALIZE_SESSION    _BTCAPIOC(0x02)

/* Either side: cancel the session.  Idempotent: returns 0 in both
 * IDLE and ABORTED.  In READY the session is torn down and any blocked
 * waiter wakes up with -ECANCELED.
 */

#define BTCAPIOC_ABORT_SESSION       _BTCAPIOC(0x03)

/* Reader-side: fetch the metadata the writer registered (schema_magic,
 * total_bytes, name).  Returns -ENOENT in IDLE/ABORTED so a reader that
 * raced an abort can back out without sending a half-frame.
 */

#define BTCAPIOC_GET_SESSION_META    _BTCAPIOC(0x04)  /* btcap_session_meta_s */

/* Either side: poll the externally visible state (one of the values
 * below).  This is a snapshot; a writer should rely on write() return
 * values, not on QUERY_STATE, for sequencing.
 */

#define BTCAPIOC_QUERY_STATE         _BTCAPIOC(0x05)  /* int32_t              */

/****************************************************************************
 * Pre-processor Definitions: externally visible session state values
 ****************************************************************************/

#define BTCAP_STATE_IDLE             0   /* No session registered           */
#define BTCAP_STATE_READY            1   /* Session in flight (writer       */
                                         /* may or may not be finalized)    */
#define BTCAP_STATE_ABORTED          2   /* Session torn down; cleanup in   */
                                         /* progress; will return to IDLE   */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#define BTCAP_NAME_MAX               32

/* Session metadata.  `total_bytes` is the full size of the byte stream
 * the writer intends to push (.cap file header + field descriptors +
 * record payload), used by the BT side to frame BTCS/meta/BTCE on the
 * wire.  `name` is null-padded; truncation is up to the caller.
 */

struct btcap_session_meta_s
{
  uint16_t schema_magic;
  uint16_t reserved0;
  uint32_t total_bytes;
  char     name[BTCAP_NAME_MAX];
};

/****************************************************************************
 * ABI size / offset locks
 ****************************************************************************/

_Static_assert(sizeof(struct btcap_session_meta_s) == 40,
               "btcap_session_meta_s ABI");
_Static_assert(offsetof(struct btcap_session_meta_s, total_bytes) == 4,
               "btcap_session_meta_s.total_bytes offset");
_Static_assert(offsetof(struct btcap_session_meta_s, name) == 8,
               "btcap_session_meta_s.name offset");

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTCAP_H */
