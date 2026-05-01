# LUMP UART Protocol Engine

## 1. Overview

A **LUMP (LEGO UART Messaging Protocol)** engine that talks to the Powered Up smart devices (LPF2 motors / sensors) plugged into the SPIKE Prime Hub's six I/O ports (A–F).  Lives in NuttX kernel space and feeds mode info + DATA frames to the upcoming Motor (#44) and Sensor (#45) drivers via a `lump_*` API (Issue #43).

Reference implementation: `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` (1264 lines).  pybricks's Contiki protothread model is replaced with **one NuttX kthread per port (six total)**.

## 2. Architecture

### 2.1 Layering

```
[user]   apps/port/port_main.c (CLI)
   │
[user]   /dev/legoport[N] ioctl (LUMP_*)
   │
[kernel] stm32_legoport_chardev.c (chardev shim)
   │
[kernel] stm32_legoport_lump.c (engine, kthread × 6)
   │
[kernel] stm32_legoport_uart_hw.c (USART register wrapper)
   │
[hw]     UART4/5/7/8/9/10
```

### 2.2 Threading: kthread per port

- Six kthreads are pre-created in `stm32_legoport_lump_register()` at boot (priority `SCHED_PRIORITY_DEFAULT` = 100, 2048 B stack each → 12 KB total)
- Each kthread sleeps on a per-port `nxsem_t lump_wakeup`
- The DCM (#42) handoff callback runs in HPWORK (priority 192) context and only does `nxsem_post(&lump_wakeup[port])`, so the 2 ms DCM cadence is preserved
- HPWORK is intentionally not used as the engine driver — RX byte reads block for up to 250 ms during sync, which would freeze the global HPWORK queue

### 2.3 Bypassing the NuttX serial driver

- The defconfig deliberately leaves `CONFIG_STM32_UART{4,5,7,8,9,10}` *unset* — setting them auto-pulls in `_SERIALDRIVER` and registers `/dev/ttyS*` for ports the LUMP engine needs to own
- `lump_uart_open()` enables the RCC clock by hand via `modifyreg32(STM32_RCC_APB1ENR/APB2ENR, ...)`
- USART CR1/BRR/CR2/CR3 are programmed directly; per-port IRQ via `irq_attach + up_enable_irq`

### 2.4 NVIC priority — slot 0x90, six UARTs co-equal

`docs/{ja,en}/hardware/dma-irq.md:151` reserves 0x90 for the LUMP UARTs.  `stm32_bringup.c` sets all six UART IRQs (UART4/5/7/8/9/10) to 0x90 inside the `CONFIG_ARCH_IRQPRIO` block.

- pybricks puts these IRQs at preempt 0 (highest); NuttX BASEPRI constraints compress that to 0x90, but the relative ordering (LUMP > BT > everything else) is preserved
- Because all six are co-equal, the ISR is kept short: read SR, drain DR into the per-port ring, clear ORE; no per-byte `nxsem_post` (a `post_pending` flag bounds the wake rate)

## 3. State machine

```
       lump_wakeup (DCM handoff CB posts the sem)
              │
              ▼
   ┌──────[ IDLE ]──────┐
   │                    │
   │ open USART AF      │
   │ send CMD SPEED     │
   │  → fall back to    │
   │    2400 baud if no │
   │    ACK in 10 ms    │
   ▼                    │
[ SYNCING ]             │
   │ wait for CMD TYPE  │
   │ (10 retries on bad │
   │  id / checksum)    │
   ▼                    │
[ INFO ]                │
   │ NAME, RAW, PCT,    │
   │ SI, UNITS, MAPPING,│
   │ FORMAT × N modes   │
   │ + SYS_ACK from dev │
   │ send our SYS_ACK   │
   │ wait 10 ms         │
   │ switch baud        │
   ▼                    │
[ DATA ] ◄── 100 ms NACK keepalive
   │                    │
   │ recv DATA frames   │
   │ → on_data callback │
   │ → ring buffer      │
   │ drain TX queue     │
   │  (CMD SELECT,      │
   │   CMD EXT_MODE,    │
   │   DATA writable)   │
   │                    │
   │ disconnect when    │
   │  - 6 missed        │
   │    keepalives      │
   │    (~600 ms silent)│
   │  - watchdog 2 s    │
   ▼                    │
[ ERR ] ── close ──────┘
   release_uart →
   register_uart_handoff →
   capped exp backoff (100ms → 1s → 5s → 30s)
```

## 4. Wire format (LUMP)

```
Header byte: [TT SSS CCC]
  TT  (bit 7-6): message type  SYS=00, CMD=01, INFO=10, DATA=11
  SSS (bit 5-3): payload size  0..5 = 1, 2, 4, 8, 16, 32 bytes
  CCC (bit 2-0): SYS=system code / CMD=command / INFO/DATA=mode

SYS  : header only (1 byte)
CMD  : header + payload + checksum
INFO : header + info_type byte + payload + checksum
DATA : header + payload + checksum

checksum = 0xFF XOR all preceding bytes
```

The constants follow `pybricks/lib/lego/lego_uart.h` exactly (`LUMP_CMD_TYPE/MODES/SPEED/SELECT/WRITE/EXT_MODE/VERSION`, `LUMP_INFO_NAME/RAW/PCT/SI/UNITS/MAPPING/FORMAT`, `LUMP_SYS_SYNC/NACK/ACK`).

## 5. Public API (`board_lump.h`)

```c
struct lump_device_info_s    /* type_id, num_modes, baud, modes[8] */
struct lump_mode_info_s      /* name, num_values, data_type, writable, raw/pct/si min-max, units */
struct lump_data_frame_s     /* mode, len, data[32] — DATA frame snapshot */
struct lump_status_full_s    /* state, type, mode, baud, rx/tx bytes, drops, backoff, stack high-water */

int lump_attach(int port, const struct lump_callbacks_s *cb);
int lump_detach(int port);
int lump_select_mode(int port, uint8_t mode);
int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len);
int lump_get_info(int port, struct lump_device_info_s *out);
int lump_get_status(int port, uint8_t *flags, uint32_t *rx, uint32_t *tx);
int lump_get_status_full(int port, struct lump_status_full_s *out);
```

`lump_attach` fires `on_sync` synchronously if the engine is already SYNCED (lock released first).  Same-port re-entry from inside a callback returns `-EDEADLK`.

### 5.1 Callback fire timing (Issue #76)

| Callback | Source | When |
|---|---|---|
| `on_sync` | per-port kthread | Right after each `SYNCING -> DATA` transition (every session, including re-sync after backoff).  `info` is a snapshot of the engine state at that point. |
| `on_sync` | calling thread | Synchronously inside `lump_attach()` if the engine is already SYNCED at attach time. |
| `on_data` | per-port kthread | Once per successfully parsed DATA frame, after `current_mode` is updated. |
| `on_error` | per-port kthread | Once when the DATA loop unwinds (sync failure / missed keepalives / watchdog stall, all unified) after `release_uart` has run.  Fires with `data == NULL && len == 0` so consumers can publish a disconnect sentinel. |

In all cases the engine drops `cb_lock` before invoking the callback, so it is safe to call back into `lump_get_info` / `lump_get_status` from inside.  Same-port re-entry into `lump_attach` / `lump_detach` (or a second callback path) is rejected with `-EDEADLK` via the `in_callback` flag.

Note that `on_error` fires on **every session-ending transition**, including pure sync failures where `on_sync` never ran — consumers should be idempotent (receiving "disconnect" twice in a row must be a no-op).

## 6. ioctl on `/dev/legoport[N]`

| ioctl | arg | Behaviour |
|---|---|---|
| `LEGOPORT_LUMP_GET_INFO` | `lump_device_info_s *` | Full info if SYNCED, else `-EAGAIN` |
| `LEGOPORT_LUMP_SELECT` | `uint8_t mode` | Queue a CMD SELECT |
| `LEGOPORT_LUMP_SEND` | `legoport_lump_send_arg_s *` | Queue a DATA TX for the writable mode.  When `current_mode != arg.mode` the engine **prepends a CMD SELECT** in the same drain pass before emitting DATA |
| `LEGOPORT_LUMP_POLL_DATA` | `lump_data_frame_s *` | Pop one frame from the engine ring; empty → `-EAGAIN` |
| `LEGOPORT_LUMP_GET_STATUS_EX` | `lump_status_full_s *` | Full per-port snapshot |
| `LEGOPORT_LUMP_HW_DUMP` | (none) | Print RCC/USART/NVIC state to syslog (`CONFIG_LEGO_LUMP_DIAG`) |

## 7. CLI (`port lump <subcommand>`)

```
port lump status              - per-port engine state table
port lump info <N>            - dump full lump_device_info_s
port lump set-mode <N> <m>    - request CMD SELECT
port lump send <N> <m> <hex>...   - DATA TX (writable mode, 1..32 hex pairs)
port lump watch <N> <ms>      - dump DATA frames for `ms` ms (10 ms poll)
port lump-hw dump             - RCC/USART/NVIC dump (diag build only)
```

## 8. Handoff contract (with DCM, Issue #42)

1. At boot the engine calls `stm32_legoport_register_uart_handoff(port, cb, &g_lump[port])` for all six ports
2. When the DCM confirms `UNKNOWN_UART` on a port, the CB runs in HPWORK context and only stashes the pin descriptor + posts the per-port wakeup sem
3. The kthread wakes, runs SYNC, then enters the DATA loop
4. On disconnect / error the kthread teardown is **`close → release_uart → register_uart_handoff → backoff sleep`**, in that order — `release_uart` clears the CB pointer so we re-register immediately, before the backoff
5. Backoff is `100 ms → 1 s → 5 s → 30 s` capped; reset to 0 on a clean exit (e.g. CLI reset)

## 9. Tuning knobs

| Define | Value | Description |
|---|---|---|
| `LUMP_BAUD_INITIAL` | 115200 | SPEED probe baud |
| `LUMP_BAUD_FALLBACK` | 2400 | EV3-compat fallback |
| `LUMP_BAUD_MAX` | 460800 | Upper bound on device-negotiated baud |
| `LUMP_IO_TIMEOUT_MS` | 250 | SYNC/INFO single-byte read timeout |
| `LUMP_DATA_RECV_SLICE_MS` | 20 | DATA-loop frame-recv slice |
| `LUMP_KEEPALIVE_MS` | 100 | `SYS_NACK` cadence |
| `LUMP_DATA_MISS_LIMIT` | 6 | Disconnect after 6 silent keepalives (≈ 600 ms) |
| `LUMP_WATCHDOG_MS` | 2000 | Per-iteration DATA-loop watchdog |
| `LUMP_DATA_QUEUE` | 16 | Per-port DATA-frame ring depth |
| `LUMP_UART_RXRING_SIZE` | 256 | Per-port ISR-fed RX ring |
| `LUMP_UART_TX_BYTE_TIMEOUT_MS` | 10 | Per-byte poll-TXE timeout |
| Backoff schedule | `100/1000/5000/30000 ms` | capped exp; 5 consecutive failures → `LUMP_FAULT_BACKOFF` |

## 10. License

`stm32_legoport_lump.c` is a port from `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c`.  SPDX `MIT` (the MIT half of pybricks's dual licensing), with `Copyright (c) 2018-2023 The Pybricks Authors` retained.

## 11. References

- Design overview: [port-detection.md](port-detection.md) §4 (LUMP protocol + DCM handoff)
- Resource ledger: `docs/{ja,en}/hardware/dma-irq.md`
- pybricks origins: `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c`, `pybricks/lib/lego/lego_uart.h`
