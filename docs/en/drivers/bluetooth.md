# CC2564C Bluetooth (btstack + SPP)

Classic Bluetooth SPP (Serial Port Profile over RFCOMM) streaming of
LSM6DS3TR-C IMU samples from the SPIKE Prime Hub to a PC (Linux /
macOS), built on the on-board **TI CC2564C** BR/EDR + BLE dual-mode
controller.

Issue #47 shipped a working HCI bring-up on top of NuttX's stock BT
host stack, but that stack is LE only and has no RFCOMM / SDP, so Issue
#52 swapped it out for **btstack** (BlueKitchen) and removed the
upstream stack entirely.  The current architecture is three layers:

- **Board layer** (`boards/spike-prime-hub/src/`): USART2 + DMA
  bring-up and `/dev/ttyBT` character device.
- **btstack** (`libs/btstack/` submodule): HCI + L2CAP + RFCOMM + SDP,
  plus the official CC2564C chipset driver.
- **App layer** (`apps/btsensor/`): NuttX-side run loop + UART adapter
  that let btstack run in user mode, SPP service, IMU sampler.

## Hardware wiring

| Signal | Pin / peripheral | Notes |
|--------|-----------------|-------|
| TX | PD5 (AF7) | USART2 TX |
| RX | PD6 (AF7) | USART2 RX |
| CTS | PD3 (AF7) | HW flow control required |
| RTS | PD4 (AF7) | HW flow control required |
| nSHUTD | PA2 (GPIO output) | CC2564C chip enable (active HIGH, LOW at boot) |
| SLOWCLK | PC9 (AF3, TIM8 CH4) | 32.768 kHz 50 % PWM, stable before nSHUTD HIGH |
| DMA RX | DMA1 Stream 7 Channel 6 | F413 alt mapping #2 (VERY_HIGH) |
| DMA TX | DMA1 Stream 6 Channel 4 | VERY_HIGH |
| NVIC priority | 0xA0 (USART2, DMA1 S6/S7) | Issue #50 slot (LUMP=0x90 > BT=0xA0 > IMU=0xB0) |

Matches pybricks `lib/pbio/platform/prime_hub/platform.c`.  TIM8 CH4
32.768 kHz: PSC=0, ARR=2929, CCR4=1465 (APB2 96 MHz / 2930 =
32.765 kHz, −0.01 % error).

## Software layers

```
┌────────────────────────────────────────────────────────────┐
│ apps/btsensor/ (user-mode, Issue #52 Step C-E)             │
│  btsensor_main.c   NSH builtin, run-loop host              │
│  btsensor_spp.c    L2CAP + RFCOMM + SDP + SSP Just-Works   │
│  btsensor_tx.c     RFCOMM send arbiter (resp + frame ring) │
│  btsensor_button.c BT button events (open /dev/btbutton)   │
│  btsensor_led.c    BT LED helper (open /dev/rgbled0)       │
│  imu_sampler.c     uORB sensor_imu -> RFCOMM streaming     │
│  port/             btstack run loop + UART adapter         │
└────────────┬───────────────────────────────────────────────┘
             │ read/write/ioctl/poll on /dev/ttyBT
┌────────────▼───────────────────────────────────────────────┐
│ libs/btstack/                                              │
│  src/ hci.c l2cap.c btstack_run_loop_base.c ...            │
│  src/classic/ rfcomm.c sdp_server.c spp_server.c ...        │
│  chipset/cc256x/ btstack_chipset_cc256x.c (init script)    │
└────────────┬───────────────────────────────────────────────┘
             │ H4 over btstack_uart_t
┌────────────▼───────────────────────────────────────────────┐
│ boards/spike-prime-hub/src/ (kernel-mode)                  │
│  stm32_btuart.c          USART2 + DMA lower-half            │
│  stm32_btuart_chardev.c  /dev/ttyBT + poll()               │
│  stm32_bt_slowclk.c      TIM8 CH4 32.768 kHz PWM           │
│  stm32_bluetooth.c       nSHUTD + slow clock + chardev reg │
│                          (HCI bring-up moved to btstack)   │
└────────────────────────────────────────────────────────────┘
```

### Kernel side (`boards/spike-prime-hub/src/`)

- `stm32_btuart.c` — `struct btuart_lowerhalf_s` implementation
  (USART2 + DMA1 S6/S7).  RX is a 512-byte circular DMA with
  USART IDLE IRQ notification, TX is blocking DMA.  Exports a
  non-destructive `stm32_btuart_rx_available()` helper so the chardev
  can report POLLIN without perturbing the stream.
- `stm32_btuart_chardev.c` — wraps the above lower-half as a POSIX
  character device at `/dev/ttyBT`.  Implements `read`/`write`/`poll`
  and two ioctls:
    - `BTUART_IOC_SETBAUD` — change the UART baud
    - `BTUART_IOC_CHIPRESET` — pulse CC2564C nSHUTD low/high so the
      next `hci_init` sees a fresh chip.  Issue #56 follow-up:
      btstack's `hci_power_off` leaves the chip in a post-init-script
      state that the next session cannot drive back to
      HCI_STATE_WORKING, so the daemon issues this ioctl on every
      start.

  `poll()` setup reports POLLIN when the RX ring is non-empty and
  always-POLLOUT because the lower-half write is blocking-DMA based.
- `stm32_bluetooth.c` — nSHUTD toggle, slow clock start, chardev
  registration, and `stm32_bluetooth_chip_reset()` (the kernel-side
  backend for `BTUART_IOC_CHIPRESET`).  HCI reset / init script /
  baud switch are delegated to btstack.
- `stm32_bt_slowclk.c` — TIM8 CH4 PWM (unchanged from Issue #47).

### User side (`apps/btsensor/port/`)

NuttX port of the btstack `port/` layer.  Written against
`libs/btstack/platform/posix/btstack_run_loop_posix.c` and
`btstack_uart_posix.c`:

- `btstack_run_loop_nuttx.c` — single-threaded run loop that waits on
  the registered data sources via `poll(2)`.  ISR-side wake-ups come
  through the chardev's `poll_notify(POLLIN)`.
- `btstack_uart_nuttx.c` — `btstack_uart_t` backed by the `/dev/ttyBT`
  fd.  `receive_block` / `send_block` just flip the data-source
  READ/WRITE flags; the run loop's poll dispatch completes the I/O.
- `chipset/cc256x_init_script.c` — CC2564C v1.4 TI service pack with
  the eHCILL flag patched to zero (pybricks baseline).  Exports
  `cc256x_init_script[]` + `cc256x_init_script_size` consumed by
  `btstack_chipset_cc256x.c`.

### App layer (`apps/btsensor/`)

- `btsensor_main.c` — NSH builtin `btsensor start [batch]` /
  `stop` / `status`.  Issue #56 Commit B converts the foreground
  `btsensor &` form into a long-lived daemon spawned with
  `task_create()`; `stop` walks an event-driven teardown FSM (GAP off
  → RFCOMM disconnect → sampler/tx deinit → HCI off → run loop exit),
  each pending state guarded by a 3 s btstack-timer watchdog.  Commit
  C adds the **BT visibility state machine** (OFF / ADVERTISING /
  FAIL_BLINK / PAIRED) and short / long press handlers that drive the
  button (btsensor_button) and LED (btsensor_led) helpers.
- `btsensor_button.c` — registers `/dev/btbutton` (kernel-side ADC
  ladder polling) as a btstack data source; debounces short vs. long
  press using `CONFIG_APP_BTSENSOR_BTN_LONG_PRESS_MS` (default
  1500 ms).  Callbacks reach the BT state machine through
  `btstack_run_loop_execute_on_main_thread()` so all transitions stay
  on the run loop's single thread.
- `btsensor_led.c` — wraps `/dev/rgbled0` (TLC5955) BT_B / BT_G /
  BT_R channels (18 / 19 / 20).  Four modes: off, solid blue, slow
  blue blink (`CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS`), and a
  fail-pulse train (`CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS` toggles).
  Animations are driven by btstack timers so they share the run
  loop's cadence and stop cleanly during teardown.
- `btsensor_spp.c` — L2CAP + RFCOMM + SDP setup, SPP SDP record, SSP
  Just-Works pairing, RFCOMM channel lifecycle handlers.  **Default
  GAP posture is discoverable=0 / connectable=0** — Commit C wires the
  BT button to enable advertising on demand.
- `btsensor_tx.c` — single RFCOMM send arbiter introduced in Commit B.
  Holds a small ASCII response queue (Commit D, priority send) and a
  frame ringbuf (drop-oldest on back-pressure); both share one
  `rfcomm_request_can_send_now_event` registration so command replies
  and IMU telemetry never compete.
- `btsensor_cmd.c` — Commit D ASCII command parser.  Bytes received
  on `RFCOMM_DATA_PACKET` accumulate in a 64-byte line buffer; on
  `'\n'`, `process_line()` runs `strtok_r` dispatch and queues the
  reply (`OK\n` / `ERR <reason>\n`) via
  `btsensor_tx_enqueue_response()`.  Command set is `IMU ON/OFF` and
  `SET ODR/BATCH/ACCEL_FSR/GYRO_FSR <value>` only.
- `imu_sampler.c` — opens `/dev/uorb/sensor_imu0` (only while sampling
  is enabled), registers the fd as a btstack data source, copies each
  `struct sensor_imu` (paired raw int16 accel + gyro + ISR-captured
  timestamp) into a wire-format sample slot and hands a
  Kconfig-tunable batch to `btsensor_tx_try_enqueue_frame()`.  Commit
  D adds `imu_sampler_set_enabled()` plus cached SET helpers
  (`set_odr_hz` / `set_batch` / `set_accel_fsr` / `set_gyro_fsr`),
  each rejected with `-EBUSY` while sampling is active.

## NSH commands

```
nsh> btsensor start                       # launch daemon (BT adv / IMU both off)
nsh> btsensor status                      # running / pid / bt / imu / config / rfcomm / stats
nsh> btsensor stop                        # asynchronous teardown (HCI off completes within ~3 s)
nsh> btsensor bt    <on|off>              # BT visibility (mirrors short/long button press)
nsh> btsensor imu   <on|off>              # IMU sampling on/off (independent of BT state)
nsh> btsensor unpair                      # delete all stored Bluetooth pairings (Issue #72)
nsh> btsensor dump  [ms]                  # read /dev/uorb/sensor_imu0 directly, print raw int16 + timestamps
nsh> btsensor set   odr        <hz>       # ODR (IMU off only)
nsh> btsensor set   batch      <n>        # samples per RFCOMM frame (IMU off only)
nsh> btsensor set   accel_fsr  <g>        # accel FSR (IMU off only)
nsh> btsensor set   gyro_fsr   <dps>      # gyro FSR (IMU off only)
```

Batch size is changed dynamically at runtime via `btsensor set batch
<n>` while IMU is off.  Multiple-start is rejected with
`already running`.  `stop` / `bt` / `imu` / `set` post into the
BTstack main thread via `btstack_run_loop_execute_on_main_thread()`
and the NSH caller blocks on a semaphore (3 s timeout) for the
result, so the FSM and sampler stay single-threaded.  `dump` is the
only sub-command that runs directly in NSH context — it opens the
uORB topic itself, which auto-activates the driver if no other
subscriber holds it open.

`status` output:

```
running:    yes
pid:        9
bt:         off               # off / advertising / fail_blink / paired
imu:        on                # on / off
config:     odr=416Hz batch=8 accel_fsr=4g gyro_fsr=2000dps
rfcomm cid: 0
frames:     sent=0 dropped=194  # all dropped while rfcomm cid==0
```

`dump` output (one line per sample):

```
# ts_us ax ay az gx gy gz (raw int16, Hub body frame)
43933310 -76 32 -8220 -5342 -5929 -5789
43935640 -57 20 -8214 -5342 -5927 -5789
...
# 14 sample(s) over 200 ms
```

This makes the full Commit D PC command set (`IMU ON/OFF`,
`SET *`) reachable from NSH without a paired PC, and `dump` covers
the no-RFCOMM "I just want raw samples" use case.

## BT state machine

```
   OFF ──short/long press──▶ ADVERTISING ──pairing OK──▶ PAIRED
                                  │                          │
                                  │  pairing FAIL            │  long press
                                  ▼                          ▼
                             FAIL_BLINK ─auto──▶ OFF         OFF
                                                              ▲
                                  PAIRED ─link drop──▶ CONNECTABLE
                                                       │  ▲
                                  short/long press     │  │  RFCOMM open
                                  / `bt on`            │  │  (silent reconnect
                                                       │  │   with stored key)
                                                       ▼  │
                                                  ADVERTISING ────▶ PAIRED
```

- States: `OFF`, `ADVERTISING`, `CONNECTABLE`, `FAIL_BLINK`, `PAIRED`.
  `status` prints `bt: <state>`.
- `btsensor bt on` and short press are **link-key-aware** (Issue #71):
  if btstack's link-key DB already has a paired peer, OFF/FAIL_BLINK
  go to `CONNECTABLE` (silent reconnect, no inquiry exposure); if the
  DB is empty, OFF/FAIL_BLINK go to `ADVERTISING` so a fresh PC can
  pair. CONNECTABLE/ADVERTISING/PAIRED are no-ops.
- Long press: OFF/FAIL_BLINK → **forced ADVERTISING** (ignores the
  link-key DB; this is the escape hatch for adding another PC while
  a previous pairing still occupies the DB), ADVERTISING/CONNECTABLE
  → OFF, PAIRED → RFCOMM disconnect + OFF.
- `btsensor bt off` is unconditional (turns OFF from any reachable
  state).
- `btsensor unpair` (Issue #72) clears btstack's link-key DB without
  changing the current BT state; the next `bt off` → `bt on` cycle
  then routes through the link-key-aware path with an empty DB and
  lands in BT_ADVERTISING for a fresh pair. Use this to retire an
  old PC without rebooting.
- LED (Issue #73 — each state has a distinct rhythm; all-blue
  family is preserved):
    - `OFF` — off
    - `ADVERTISING` — symmetric blink (50% duty) at the
      `CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS` period (default 1 s)
    - `CONNECTABLE` — double-blink: two short ON pulses (~100 ms each
      with a 100 ms gap) then a long OFF rest filling the rest of the
      period. Reads as "ti-tick . . . . . . ti-tick" — clearly
      different rhythm from ADVERTISING.
    - `PAIRED` — solid blue
    - `FAIL_BLINK` — `CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS` short blue
      pulses (~150 ms × 2N) one-shot, then off
- Pairing completion routes through
  `HCI_EVENT_SIMPLE_PAIRING_COMPLETE`: status==0 → PAIRED (LED solid
  immediately, matching the Issue #56 spec "ペアリング成功で BT LED
  点灯"), status≠0 → FAIL_BLINK.  A subsequent
  `RFCOMM_EVENT_CHANNEL_OPENED` keeps the state at PAIRED (no LED
  change), and a link drop / disconnect routes to CONNECTABLE
  (connectable=1, discoverable=0; LED resumes the slow blink) so
  the same paired PC can reconnect by BD_ADDR while strangers stay
  unable to discover the Hub via inquiry. Issue #68.
- All transitions run on the BTstack main thread (worker / shell
  callers go through `btstack_run_loop_execute_on_main_thread()`).

## PC command protocol (Commit D)

After the PC opens the RFCOMM channel it can drive the IMU pipeline
with one ASCII line per command (`\n` terminated, `\r` ignored).  The
reply is `OK\n` on success or `ERR <reason>\n` on failure.

| Command | Action | Constraint |
|---|---|---|
| `IMU ON\n` | Start sampling (open the driver fd) | — |
| `IMU OFF\n` | Stop sampling (close the driver fd → auto-deactivate) | — |
| `SET ODR <hz>\n` | Set ODR (13/26/52/104/208/416/833/1660/3330/6660 Hz) | only while IMU OFF |
| `SET BATCH <n>\n` | Samples per RFCOMM frame (1..80) | only while IMU OFF |
| `SET ACCEL_FSR <g>\n` | Accel FSR (2/4/8/16) | only while IMU OFF |
| `SET GYRO_FSR <dps>\n` | Gyro FSR (125/250/500/1000/2000) | only while IMU OFF |

Reply patterns:
- success: `OK\n`
- SET while IMU on: `ERR busy\n`
- bad value / token: `ERR invalid <token>\n`
- line longer than 64 B: `ERR overflow\n`
- unknown command: `ERR unknown <cmd>\n`

Implementation notes:
- Driver fd is closed while IMU is OFF, so SET helpers open a
  **transient O_WRONLY fd** for the ioctl.  `O_WRONLY` skips the
  sensor upper-half's auto-activate (which only fires on `O_RDOK`),
  so the driver's `if (active) -EBUSY` check stays satisfied.
- Configured values live in `imu_sampler.c` cache variables
  (`g_odr_hz` / `g_accel_fsr_g` / `g_gyro_fsr_dps` / `g_batch`).  On
  ioctl failure the cache is **rolled back to the previous value**.
- The driver (`lsm6dsl_uorb.c`) holds a separate `cfg_odr` field so
  the user-configured ODR survives activate cycles; `activate(true)`
  applies `cfg_odr` to the chip via `set_odr()`.

The PC-side code lives outside this repo; an example client is
documented in `docs/{ja,en}/development/pc-receive-spp.md` (Commit
E will refresh it for the new wire format).

## EXTI0 / NVIC priority

`stm32_bringup.c` step 9 sets
`up_prioritize_irq(STM32_IRQ_EXTI0, NVIC_SYSH_PRIORITY_DEFAULT + 6 *
NVIC_SYSH_PRIORITY_STEP) = 0xE0`, placing BUTTON_USER (PA0 EXTI0) in
the ε layout's lowest peripheral band (alongside ADC and TLC5955) so
the IRQ stays below BASEPRI and can call NuttX work-queue APIs
freely.  See the NVIC table in `docs/{ja,en}/hardware/dma-irq.md`.

## Self-pipe wake (run loop)

`apps/btsensor/port/btstack_run_loop_nuttx.c` wakes the run loop on
cross-thread `execute_on_main_thread()` calls via a **self-pipe**:
- `pipe(p)` opened in init, both ends `O_NONBLOCK`.
- `g_wake_lock` (pthread_mutex) protects the write side against the
  open/close race during teardown.
- The read side is registered as a btstack data source so the main
  thread's poll() picks up the byte as POLLIN.
- Callback enqueues write a single 'x' byte to wake the loop.

This drops the cross-thread wake latency from up to
`NUTTX_RUN_LOOP_MAX_WAIT_MS` (1 s) to a few hundred microseconds —
essential for responsive button handling and short stop latency.

## Known follow-up

- **BT control button physical detection**:
  `boards/spike-prime-hub/src/stm32_btbutton.c` polls the PA1 ADC
  resistor ladder and exposes the BT button state at `/dev/btbutton`.
  On the hardware tested for Issue #56 Commit C the ADC value did not
  swing far enough on a button press to cleanly cross the
  pybricks-ported threshold table, so the BT state machine receives
  no reliable button events yet.  The pybricks
  `g_ladder_dev1_levels` table and the PA1 wiring need a fresh look —
  tracked as a follow-up to this commit.

## Bring-up sequence (btstack-driven)

1. NuttX boot: `stm32_bluetooth_initialize()` runs — nSHUTD LOW, slow
   clock up, USART2 lower-half instantiated, 50 ms LOW / HIGH / 150 ms
   settle, `/dev/ttyBT` registered.
2. `btsensor start` (daemon thread):
   - `btstack_run_loop_init(btstack_run_loop_nuttx_get_instance())`
   - `hci_init(transport, cfg)` + `hci_set_link_key_db()` +
     `hci_set_chipset(cc256x_instance)`
   - `spp_server_init()` registers L2CAP + RFCOMM + SDP (GAP
     discoverable/connectable both off)
   - `btsensor_tx_init()` + `imu_sampler_init()` hook
     `/dev/uorb/sensor_imu0` as a data source
   - `hci_power_control(HCI_POWER_ON)` drives the state machine
     through HCI_Reset → Read_Local_Version → HCI_VS_Update_UART_Baud
     (0xFF36) → chipset init script streaming (~40 chunks, ~200 ms)
     → Read_BD_ADDR → Write_Scan_Enable → `HCI_STATE_WORKING`.
3. `HCI working, BD_ADDR ...` is logged via syslog (advertising stays
   off until the BT button enables it in Commit C).
4. `btsensor stop` walks the teardown FSM through GAP off → RFCOMM
   disconnect (when cid != 0) → sampler/tx deinit →
   `hci_power_control(HCI_POWER_OFF)` → `BTSTACK_EVENT_STATE =
   HCI_STATE_OFF` → `btstack_run_loop_trigger_exit()`.  Each pending
   state has a 3 s watchdog.  After the run loop returns the daemon
   calls `hci_close()` to reset btstack state so the next `start`
   succeeds.

## SPP service

- **Local name**: `SPIKE-BT-Sensor`
- **Class of Device**: `0x001F00` (Uncategorized)
- **Security**: SSP Just-Works (`SSP_IO_CAPABILITY_DISPLAY_YES_NO`),
  `LEVEL_2`
- **Service name**: `SPIKE IMU Stream`
- **RFCOMM channel**: 1
- **SDP UUID**: `0x1101` (SPP), Profile Descriptor SPP v1.2

## RFCOMM payload (IMU frame)

Little-endian, 18-byte header + 8 samples × 16 bytes = 146 bytes per
frame at the Kconfig default; up to 80 samples per frame are supported.
Issue #56 Commit E reworked the wire format (new magic, FSR fields in
the header, per-sample `ts_delta_us`) — see
`docs/development/pc-receive-spp.md` for the full spec / parser.

```c
struct spp_frame_hdr {                // 18 bytes
    uint16_t magic;            // 0xB66B
    uint8_t  type;             // 0x01 = IMU
    uint8_t  sample_count;     // 1..80
    uint16_t sample_rate_hz;   // current ODR (Hz)
    uint16_t accel_fsr_g;      // 2 / 4 / 8 / 16
    uint16_t gyro_fsr_dps;     // 125 / 250 / 500 / 1000 / 2000
    uint16_t seq;              // monotonic per frame
    uint32_t first_sample_ts_us; // low 32 bits of CLOCK_BOOTTIME us
    uint16_t frame_len;        // = 18 + sample_count * 16
};

struct imu_sample {                   // 16 bytes
    int16_t  ax, ay, az;       // LSM6DSL accel chip-frame raw LSB
    int16_t  gx, gy, gz;       // LSM6DSL gyro  chip-frame raw LSB
    uint32_t ts_delta_us;      // sample.ts - first_sample_ts_us
                               // (sample[0] = 0)
};
/* Convert with the driver's startup FSR (default ±8 g / ±2000 dps):
 *   accel_ms2  = raw * fsr_g  * 9.80665 / 32768
 *   gyro_dps   = raw * fsr_dps / 32768
 * Issue #56 Commit E embeds the per-frame FSR in the header so the PC
 * can pick up FSR changes at runtime.
 */
```

## Key design decisions

### Why btstack

The NuttX upstream BT host stack is BLE only — no Classic RFCOMM / SDP
— so the project needed a different stack to deliver SPP to a PC.
btstack supports Classic + BLE simultaneously, ships a first-party
CC2564C chipset driver and offers `embedded` / `freertos` / `posix` /
`zephyr` run loops that make a NuttX port small.

### `/dev/ttyBT` chardev

btstack runs in user mode under BUILD_PROTECTED, so the HCI UART has
to cross the kernel/user boundary through a POSIX fd.  The chardev is
a thin wrapper over the existing `btuart_lowerhalf_s`; btstack's
`btstack_uart_t` adapter just opens the fd and uses it as a data
source.

### `btuart_read` `rxwork_pending` re-arm (critical fix)

`stm32_btuart.c`'s `btuart_notify_rx` uses an `rxwork_pending` latch to
debounce rxcb calls.  The latch was originally only cleared when
`btuart_read` was called on an already-empty ring.  btstack's
`hci_transport_h4` state machine reads *exactly* the byte count it
needs (1 byte packet type → 2 byte event header → N byte body), so it
can drain the ring to empty on the final read without triggering the
empty-read branch.  Result: after the first response burst the latch
stayed true, the IDLE ISR stopped firing the rxcb and subsequent
bursts only came through when the run loop's poll timeout kicked in —
a full second per command.

Fix in `btuart_read`: after copying out `copy` bytes, re-evaluate
producer==consumer inside a critical section and clear
`rxwork_pending` when the ring is empty.  See
`boards/spike-prime-hub/src/stm32_btuart.c`.

After this fix the ISR → poll-wake path is microsecond-scale and the
full CC2564C init script (~40 chunks) streams in ~200 ms.

### CC2564C init script eHCILL patch

The pybricks-baselined v1.4 service pack has the
`HCI_VS_Sleep_Mode_Configurations` (0xFD0C) eHCILL flag set to 0x01.
With eHCILL enabled the chip sends `GO_TO_SLEEP_IND (0x30)` when idle
and goes silent until the host replies `WAKE_UP_IND (0x31)`.  btstack's
`hci_transport_h4` only sends that ack when `ENABLE_EHCILL` is
defined, which we intentionally leave out.  The service pack was
patched once (commit 92817cb) to set the flag to zero.

### btstack run loop shape

`btstack_run_loop_nuttx.c` is a trimmed-down POSIX run loop — single
thread, `poll(2)` for fd waiting, `clock_gettime(CLOCK_MONOTONIC)` for
timers.  `timeout_ms` per `poll` call is `min(next_timer_ms, 1000)`.
`poll_data_sources_from_irq()` is a flag-setter only; the real
wake-up path is the chardev calling `poll_notify` on the UART fd when
RX data arrives.

## Kconfig

```
CONFIG_STM32_USART2=y
CONFIG_STM32_TIM8=y
CONFIG_SCHED_HPWORK=y          # uORB / btuart IRQ work queue
CONFIG_SENSORS_LSM6DSL=y       # IMU uORB publication
CONFIG_UORB=y
CONFIG_APP_BTSENSOR=y
CONFIG_APP_BTSENSOR_BATCH=8    # samples per SPP frame (default)
CONFIG_APP_BTSENSOR_RING_DEPTH=8
```

The Issue #47 flags (`CONFIG_WIRELESS_BLUETOOTH_HOST`,
`CONFIG_NET_BLUETOOTH`, `CONFIG_BLUETOOTH_UART_GENERIC`,
`CONFIG_BTSAK`, `CONFIG_NETDEV_LATEINIT`) were removed in Issue #52
Step A.

## NSH usage

```
nsh> ls /dev/ttyBT
/dev/ttyBT

nsh> dmesg                                # `grep` is not in this defconfig
... BT: CC2564C powered, /dev/ttyBT ready
...

nsh> btsensor start                       # spawn the daemon (BT/IMU both off)
btsensor: started (pid 6)

nsh> dmesg                                # banner is logged via syslog -> RAMLOG
... btsensor: bringing up btstack on /dev/ttyBT
... btsensor: HCI working, BD_ADDR F8:2E:0C:A0:3E:64 — adv off ("SPIKE-BT-Sensor" hidden until BT button)

nsh> btsensor bt on                       # turn BT advertising on (or short-press BT button)
OK
```

## Host adapter compatibility (sustained streaming)

The CC2564C side keeps credits available, but on some host adapters the
HCI `Number of Completed Packets` event stops arriving partway through
the stream, which prevents btstack from emitting
`RFCOMM_EVENT_CAN_SEND_NOW` again — and the Hub stalls.  Observed
behaviour:

| Host adapter | Chip | 30 s streaming test |
|--------------|------|---------------------|
| Generic USB dongle | **MediaTek** | Stops after ~1.75 s (`pending=1` never clears; the link supervision timeout closes the session ~30 s later) |
| RPi 5 built-in | **Broadcom/Cypress** (CYW43455) | Sustains for 30 s (sensor ODR 790 Hz / link ODR 662 Hz; ~16 % drop at the Hub-side ring) |

The Hub-side code is fine.  Root cause is MediaTek Classic-BT firmware
interop with the TI CC2564C: working around it on the Hub would need a
rate-limited send path or vendor-specific TX-queue flush.  In practice,
**pick a known-good adapter**:

- ✅ **Broadcom / Cypress** (RPi 5 built-in, Apple T2-class controllers)
- ✅ **Intel** (AX200 / AX210 generation — `iwlwifi` family, generally
  good but not yet verified on this stack)
- ❌ **MediaTek** (most cheap USB dongles and Logitech "Unifying"
  adapters)

To verify a given adapter run the semi-automated H-5 test
(`tests/test_bt_spp.py::test_bt_pc_pair_and_stream`); it opens
`BTPROTO_RFCOMM` directly, captures ~3 s of frames, and asserts that
the Hub-side `frames: sent` advances and `dropped` stays under 25 %
of `sent`.  An adapter that survives H-5 cleanly is OK.

## Host-side receive

See [PC receive guide](../development/pc-receive-spp.md) for the
Linux and macOS pairing + stream read procedures.

## Related docs

- [PC receive guide](../development/pc-receive-spp.md)
- [DMA / IRQ allocation](../hardware/dma-irq.md)
- [Pin map](../hardware/pin-mapping.md)
- [Test spec H. Bluetooth](../testing/test-spec.md#h-bluetooth-test_bt_spppy)

## References

- btstack upstream: [bluekitchen/btstack](https://github.com/bluekitchen/btstack) (libs/btstack submodule, v1.8.1-6)
- btstack `platform/posix/btstack_run_loop_posix.c` — starting point for the NuttX run loop
- btstack `platform/posix/btstack_uart_posix.c` — starting point for the UART wrapper
- btstack `chipset/cc256x/btstack_chipset_cc256x.c` — init-script streaming + baud switch
- btstack `example/spp_counter.c` — minimal SPP + SSP Just-Works reference
- pybricks `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c` — STM32 HAL-style UART reference
- TI: [CC256XC-BT-SP service pack](https://www.ti.com/tool/CC256XC-BT-SP) — source init script
- RM0430 Rev 9 §9.3.4 Figure 24 / Table 30 — DMA1 request mapping
