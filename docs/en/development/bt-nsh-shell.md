# BT-side NSH shell (`MODE SHELL`)

The `btsensor` daemon can expose an **NSH console over the existing
SPP RFCOMM channel** (Issue #108).  Useful when the Hub is running
on battery (e.g. a roving robot) and you need to query state or send
commands without re-tethering USB.  The USB-side NSH on `/dev/console`
is unaffected.

## Architecture

```
PC ── BT/SPP/RFCOMM ── btsensor (userspace)
                          │
                          ├─ MODE_TELEMETRY (default) — RFCOMM RX → btsensor_cmd_feed
                          │     IMU/sensor BUNDLE frames at 100 Hz
                          │
                          ├─ MODE_SHELL_STARTING (transient) — OK\n in flight; stdin send forbidden
                          │
                          └─ MODE_SHELL — RFCOMM ↔ /dev/btnsh_in/out FIFO ↔ NSH child task
                                           btnsh_main calls nsh_session() directly
```

Telemetry and shell are **mutually exclusive**.  Entering shell mode
auto-disables the IMU/SENSOR pumps; they stay off after exiting (you
must re-issue `IMU ON` / `SENSOR ON` explicitly).

## Protocol

### Entering shell mode

Over the existing SPP/RFCOMM connection:

```
PC -> Hub:  MODE SHELL\n
Hub -> PC:  OK\n               <- MODE_SHELL_STARTING up to here (do NOT send stdin yet)
PC -> Hub:  ls /dev\n          <- After OK, bytes go to NSH stdin
Hub -> PC:  /dev/btnsh_in\n
            /dev/btnsh_out\n
            /dev/console\n
            ...
            nsh>\n
```

Anything sent before `OK\n` arrives is defensively dropped on the Hub
side (logged as `syslog` warning).  **Peer contract: no stdin until
`OK\n` is observed.**

### Leaving shell mode

Type `exit` inside NSH or drop the RFCOMM link from the PC:

```
PC -> Hub:  exit\n
Hub -> PC:  ... NSH output ...
            <- child task exits, stdout EOF, internal teardown runs
Hub -> PC:  READY\n            <- telemetry mode resumed; commands accepted again
PC -> Hub:  IMU ON\n           <- explicit re-enable (not auto-resumed)
Hub -> PC:  OK\n
```

`READY\n` indicates the daemon is back in telemetry mode and ready
for ASCII commands.

If the PC drops RFCOMM, the Hub does `kill(SIGKILL)` + reap + cleanup
internally and falls back to MODE_TELEMETRY.  `READY\n` is **not**
sent in that case (no peer to receive it).

### `MODE SHELL` failures

If `shell_enter` fails (`pipe()` / `open()` / `posix_spawn`) or the
TX response queue cannot accept `OK\n`:

```
Hub -> PC:  ERR shell_<step> <errno>\n
```

Or:

```
Hub -> PC:  ERR shell_no_buffer\n
Hub -> PC:  ERR shell_unavailable\n   <- mkfifo failed at daemon start
```

**Note**: by the time `MODE SHELL` is parsed, IMU/SENSOR pumps are
already disabled.  `ERR` does **not** restore them; re-issue
`IMU ON\n` / `SENSOR ON\n` if needed.

## USB-NSH side

The `btsensor mode` builtin can switch from the USB side too:

```
nsh> btsensor mode shell
OK
nsh> # USB NSH is still usable; the spawned BT shell waits for a peer
nsh> btsensor mode telemetry
OK
```

`btsensor mode telemetry` from USB is a force-teardown escape hatch —
useful if the BT peer wedged the shell.

## Known limitations

- **No Ctrl-C / job control**: the FIFO is not a tty and the child
  is spawned with `isctty=false`, so `TIOCSCTTY` is not issued.
  Long-running commands can only be aborted by dropping RFCOMM.
- **No CRLF / ANSI normalisation**: configure local echo and CRLF
  translation on the peer's terminal emulator (`screen`/`picocom`
  with their respective flags).
- **stdin overflow**: pasting more than the FIFO buffer (4096 B by
  default) drops the overflow.  Interactive typing is unaffected.
  Drops are logged in the Hub `syslog`; the peer is not notified.
- **Large stdout commands (`dmesg`, long `help`) are unreliable**
  (Issue #109): RFCOMM credit-based flow control combined with NSH's
  line-by-line writes exhausts the peer's outgoing credit faster than
  Linux BlueZ refreshes it.  The TX coalescing buffer fills, the
  reader pthread starts dropping bytes, and the visible output gets
  gaps.  The Hub-side session stays alive (subsequent short commands
  still work) but you should not rely on `dmesg` over BT.  **For
  large outputs, use the USB NSH on `/dev/ttyACM0` instead.**  BT NSH
  is optimised for short interactive commands (`ps`, `ls /dev`,
  `free`, `md5`, etc.).
- **No concurrent SPP clients**: same as the rest of `btsensor` —
  one RFCOMM channel only.
- **stdout overflow**: very chatty NSH output (`dmesg`, etc.) past
  the TX coalescing buffer (1024 B default) is dropped by the
  reader thread with a syslog warning.
- **No simultaneous telemetry frames**: shell and BUNDLE telemetry
  are exclusive; switch back via `MODE TELEMETRY` (or `exit`) to
  resume telemetry.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No reply to `MODE SHELL` | IMU/SENSOR pump still on | Send `IMU OFF\n` / `SENSOR OFF\n` first |
| `ERR shell_unavailable` | `mkfifo` failed at daemon init (CONFIG_PIPES / CONFIG_DEV_FIFO_SIZE missing) | Check defconfig; `btsensor stop` → `start` |
| `OK\n` arrives but no NSH output | Child task spawn / stack starvation | `nsh> ps` to inspect `btnsh` task; raise `CONFIG_APP_BTSENSOR_SHELL_NSH_STACK` |
| Large output truncated | TX coalescing buffer overflowed | Raise `CONFIG_APP_BTSENSOR_SHELL_TX_BUF` (2048/4096) |
| No `READY\n` on exit | Exit happened via RFCOMM drop | Expected; reconnect and confirm telemetry by `IMU ON` |

## Related Kconfig

| Config | Default | Purpose |
|---|---|---|
| `CONFIG_APP_BTSENSOR_SHELL_MODE` | `n` (`y` in usbnsh defconfig) | Enable shell mode |
| `CONFIG_APP_BTSENSOR_SHELL_TX_BUF` | 1024 | NSH stdout coalescing buffer (B) |
| `CONFIG_APP_BTSENSOR_SHELL_NSH_STACK` | 6144 | NSH child task stack (B) |
| `CONFIG_APP_BTSENSOR_SHELL_READER_STACK` | 2048 | Reader pthread stack (B) |
| `CONFIG_DEV_FIFO_SIZE` | 4096 (usbnsh defconfig) | FIFO buffer size (B) |
