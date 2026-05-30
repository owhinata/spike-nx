# Sensor capture (`MODE CAPTURE`)

The combination of `apps/sensor`, `apps/btsensor`, and the
`/dev/btcap` chardev provides a **drop-free capture pipeline that
ships internal sensor / RT-control state out over BT** for offline
analysis on the host (Issue #122).  Built for line-following work
where comparing Color sensor MODEs (Reflection vs RGBI etc.) and
correlating drive logs after the fact would otherwise miss samples.

USB CDC ACM string logging is **not** part of this pipeline —
printf jitter on USB perturbs the RT control loop, so capture goes
exclusively over BT SPP.

## Architecture

```
apps/sensor (NSH)
   │   sensor color capture <ms>
   │   1) drain duration_ms of uORB samples into a heap buffer
   │   2) capture_init/_write/_deinit pushes to /dev/btcap
   │              (blocks until the reader engages — no timeout)
   ▼
/dev/btcap (kernel chardev, board-side)
   │   pipe-style 1024 B ring + IDLE/READY/ABORTED state machine
   │   ioctl: REGISTER / FINALIZE / ABORT / DRAIN_START/END /
   │          QUERY_STATE / GET_SESSION_META
   ▼
apps/btsensor MODE CAPTURE (userspace)
   │   3) drain /dev/btcap, ship BTCS + meta + payload + (BTCE|BTAB)
   │   4) BUNDLE telemetry pauses while the drain runs, restored after
   ▼
host/CaptureViewer/ (.NET 10)
       BTCS scan → sanity check → `.cap` file → plot
```

`apps/capture` is a thin export library (four functions:
`capture_init/_write/_deinit/_abort`).  `apps/sensor` is the only v1
client; it owns both the capture phase and the export call.

## Workflow examples

### Linetrace (Reflection vs RGBI run comparison)

```
nsh> sensor color select 1                  # MODE 1 (Reflection)
nsh> sleep 1                                # let the sensor settle
nsh> drivebase straight 200 &               # drive (background)
nsh> sensor color capture 3000 &            # 3 s capture (mode is implicit)
nsh> btsensor mode capture                  # drain to PC
nsh> wait                                   # confirm background tasks done

nsh> sensor color select 5                  # MODE 5 (RGBI)
nsh> sleep 1
nsh> drivebase straight 200 &
nsh> sensor color capture 3000 &
nsh> btsensor mode capture                  # second drain
nsh> wait
                                            # → CaptureViewer overlays the two
```

`sensor color capture` on its own does **not** transfer data — the
writer parks waiting for a reader.  `btsensor mode capture` (NSH) or
the BT-side `MODE CAPTURE\n` is mandatory to drain the chardev.

To abandon a session, `kill <apps/sensor pid>` — the kernel chardev
release fop reclaims the session (no timeout knob exists).

### Linetrace lap capture (Issue #166)

The `linetrace` PID daemon can record a per-tick lap trace (one record
per loop tick) and export it through the same `/dev/btcap` pipeline as a
`.cap` file (schema `linetrace_lap_run`, magic 0x0012; see
capture-schemas).

```
nsh> linetrace start
nsh> linetrace cap arm 2000        # ~20 s of buffer @ 100 Hz
nsh> linetrace run 200 0.36 512    # drive the lap
nsh>                               # ... drive a full lap ...
nsh> linetrace cap stop            # freeze (or it auto-stops when full)
nsh> linetrace brake
nsh> linetrace cap export &        # blocks for the reader
nsh> btsensor mode capture         # second session: drains /dev/btcap
host$ cat /dev/rfcomm0 > lap.cap   # host captures the stream
```

Operator notes:

- `cap arm <n>` does not start motion.  Arm then `run`, or `run` then
  arm — both orders work; whatever ticks happen while ARMED (idle or
  engaged) are recorded.  `cap_max` = 3449 records (64 KB / 19 B).
- `brake` **freezes and keeps** an in-flight capture (a partial lap is
  still useful to P0c).  `cap stop` is the explicit "freeze now" verb.
- `linetrace stop` (daemon exit) drops an un-exported capture.  Run
  `cap export` before `linetrace stop`.
- The export half is identical to `sensor color capture`'s export (same
  `/dev/btcap` single-session contract, same "kill the writer to cancel"
  semantics).  Only ONE btcap session can be in flight, so you cannot
  run `sensor … capture` and `linetrace cap export` at the same time.
- A hard `kill -9 <export pid>` is reclaimed lazily: the next `cap
  arm`/`cap export`/`cap status`/`cap abort` reaps the dead exporter and
  returns the FSM to `idle`.  Use `cap abort` to discard explicitly.

### What the capture phase does

`sensor <class> capture [duration_ms]` (default 1000 ms):

1. Open the uORB topic, read one sample, learn the **implicit mode**.
   If `(class, mode_id)` is not in the static schema table, exit with
   `-ENOENT` (`select` first).
2. Accumulate samples into a heap buffer until `duration_ms` or
   `CONFIG_APP_CAPTURE_MAX_HEAP_BYTES` (default 64 KiB) — whichever is
   smaller.  A mid-run mode change exits with `-EILSEQ`.
3. `capture_init` REGISTERs the session on `/dev/btcap`.
4. `capture_write` blocks until the reader (`btsensor mode capture`)
   issues `DRAIN_START`, then writes the bytes through (pipe-style
   back-pressure paces the writer to the controller's drain rate).
5. `capture_deinit` FINALIZEs and blocks until the kernel returns to
   IDLE.

SIGINT / SIGTERM set a flag the loop polls and the export aborts with
`-ECANCELED`.  SIGKILL goes through the kernel default-action path:
the chardev release fop cleans the session up.

### `MODE CAPTURE` switching

`btsensor mode capture` (NSH) or `MODE CAPTURE\n` over BT:

1. Open `/dev/btcap` `O_RDONLY|O_NONBLOCK`.  No session in flight →
   `ENOENT`, prints `btsensor: no capture session in flight`.
2. Pause the BUNDLE emitters (IMU / sensor) and attach the chardev fd
   to the btstack run loop as a data source.
3. First frame on the wire: **BTCS (4 B) + meta (40 B = u16
   schema_magic + u16 reserved + u32 total_bytes + char[32] name)**.
4. Read the chardev in 256-byte chunks → enqueue on RFCOMM with a
   5 ms throttle (`CAP_TX_THROTTLE_MS`).  When the `btsensor_tx` ring
   is full, skip the read so the writer back-pressures inside the
   kernel `write()` (NFR-9 lossless paced sender).
5. On EOF, query the chardev state.  `READY` → emit BTCE (clean end).
   `ABORTED` → emit BTAB (truncated session).
6. `DRAIN_END` returns the chardev to IDLE; the BUNDLE emitters
   restore to their pre-capture state and MODE TELEMETRY resumes.

While CAPTURE is engaged, no telemetry frames or ASCII replies are
emitted (mutually exclusive).

## Receiving on the host

### Today (CaptureViewer.App not yet shipped)

A raw capture works with `cat /dev/rfcomm0` — but the RFCOMM device
defaults to a tty whose line discipline silently swallows control
bytes (^C / ^Q / ^S / ^D / ...) inside the binary stream.  Always go
raw before reading:

```bash
stty -F /dev/rfcomm0 raw -echo -icanon -ixon -ixoff -opost min 1 time 0
cat /dev/rfcomm0 > capture.bin &
# On the Hub: sensor color capture & btsensor mode capture
# → BTCS + meta + payload + BTCE lands in capture.bin
```

`xxd capture.bin | head` — first four bytes should be
`42 54 43 53` (`"BTCS"`).

### Parsing with `host/CaptureViewer/` (.NET 10)

`CaptureViewer.Core` directly:

```csharp
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;

var cap = CaptureFile.Open("capture.cap");
Console.WriteLine($"{cap.SchemaName} {cap.RecordCount} records");

// Typed access via the generated parser
foreach (var i in Enumerable.Range(0, cap.RecordCount))
{
    var rec = SchemaColorReflectionRun.Parse(cap.Records(i).Span);
    Console.WriteLine($"ts={rec.ts_us}us refl={rec.reflection_pct}%");
}

// Pull a session out of a continuous BT byte stream
SessionScanner.TryScan(buffer.Span,
    magic => KnownSchemas.ByMagic.ContainsKey(magic),
    out var scan);
```

See [Schema reference](capture-schemas.md) and the `host/CaptureViewer/`
sources for details.

## Wire format

One session on the BT wire:

```
+----------+--------------+------------------+----------+
| BTCS     | meta (40B)   | payload (.cap)   | BTCE/BTAB|
| 4B ASCII | u16 + u16    | u32 magic "CAPB" | 4B ASCII |
| "BTCS"   | + u32        | + 60B header     |          |
|          | + char[32]   | + N*48B fields   |          |
|          |              | + records        |          |
+----------+--------------+------------------+----------+
```

Inside `payload`, the layout is `capture_file_header_s` (64 B) →
`capture_field_desc_s[]` (48 B each) → records, all little-endian.
Authoritative declarations live in
`apps/capture/include/capture_format.h`.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `capture: no schema for class=color mode=N` | (class, mode) → schema is not registered | Add an entry to `g_capture_schemas[]` in `apps/sensor/sensor_main.c` and a schema header under `apps/capture/include/` |
| `capture: no sample within Nms` | uORB topic is silent (port empty / `select` not issued) | Check the sensor cable; run `sensor <class> select <mode>` and retry |
| `btsensor: no capture session in flight` | `btsensor mode capture` ran with no writer parked | Start `sensor <class> capture` in another NSH session (or with `&`) first |
| Capture phase finishes but no BTCE arrives | RFCOMM dropped / pairing lost / long stall on the 5 ms throttle | `btsensor diag` for controller state, re-pair on the host |
| `btsensor status` shows `dropped_full > 0` | Back-pressure regression (see commit 6a4740d) | Verify `btsensor_tx_frame_ring_full()` is checked in `apps/btsensor/btsensor_capture_mode.c::on_read` |
| `dropped_oldest > 0` during a CAPTURE-only run | A capture chunk took the drop-oldest path through `btsensor_tx_try_enqueue_frame` | Same investigation — CAPTURE chunks must not be eligible for drop-oldest |
| `cat /dev/rfcomm0` truncates mid-session | tty line discipline filtered control bytes | `stty -F /dev/rfcomm0 raw -echo -icanon -ixon -ixoff -opost min 1 time 0` |

## Kconfig

| Config | Default | Purpose |
|---|---|---|
| `CONFIG_APP_CAPTURE` | `y` (usbnsh defconfig) | userspace capture lib + `sensor capture` verb |
| `CONFIG_APP_CAPTURE_MAX_HANDLES` | 1 | concurrent in-flight sessions (v1 fixed at 1) |
| `CONFIG_APP_CAPTURE_MAX_HEAP_BYTES` | 65536 | per-session heap buffer cap in `apps/sensor` |
| `CONFIG_APP_CAPTURE_BTCAP_RING_BYTES` | 1024 | kernel chardev pipe ring size |
| `CONFIG_BOARD_BTCAP_CHARDEV` | `y` (usbnsh defconfig) | board-local `/dev/btcap` chardev driver |

There are intentionally **no** timeout Kconfigs (`WRITE_DEADLINE_MS`,
`WAIT_DRAIN_TIMEOUT_MS`).  Missing readers are the operator's problem
to resolve via `kill`.

## Automated tests

`tests/test_capture.py` (Issue #122):

| ID | mark | What it covers |
|---|---|---|
| K-1 | normal | `/dev/btcap` is registered |
| K-2 | normal | `sensor` usage advertises the `capture` verb + drain hint |
| K-3 | normal | unmapped mode hits the ENOENT path cleanly |
| K-4 | normal | `btsensor mode capture` with no writer rejects |
| K-5 | interactive | end-to-end BT round-trip (needs Color sensor + BlueZ pair) |

```bash
# Smoke (4 cases)
.venv/bin/pytest tests/test_capture.py -m "not slow and not interactive" -D /dev/ttyACM0

# K-5 only (operator confirms pairing)
.venv/bin/pytest tests/test_capture.py::test_capture_round_trip_via_rfcomm -D /dev/ttyACM0
```

Host side `CaptureViewer.Core`:

```bash
dotnet test host/CaptureViewer/CaptureViewer.slnx          # 15 cases
```

## Related issues / commits

- Issue #122 (parent Issue for this pipeline)
- Issue #109 / #110 (BT NSH SHELL throttle / back-pressure design — capture borrows the same pacing)
- Issue #54 (CC2564C ACL TX queue stall — rationale for the 5 ms throttle)
- Issue #111 (rcS auto-launch — `btsensor` is up by the time you ask)
