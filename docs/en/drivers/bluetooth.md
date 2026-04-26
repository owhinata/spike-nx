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
  and `ioctl(fd, BTUART_IOC_SETBAUD, baud)`.  `poll()` setup reports
  POLLIN when the RX ring is non-empty and always-POLLOUT because the
  lower-half write is blocking-DMA based.
- `stm32_bluetooth.c` — nSHUTD toggle, slow clock start and chardev
  registration only.  HCI reset / init script / baud switch are
  delegated to btstack.
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
- `imu_sampler.c` — opens `/dev/uorb/sensor_imu0`, registers the fd as
  a btstack data source, copies each `struct sensor_imu` (paired raw
  int16 accel + gyro + ISR-captured timestamp) into a wire-format
  sample slot and hands a Kconfig-tunable batch to
  `btsensor_tx_try_enqueue_frame()`.

## NSH commands

```
nsh> btsensor start [batch]   # launch daemon (BT advertising stays off)
nsh> btsensor status          # running / pid / bt state / rfcomm / stats
nsh> btsensor stop            # asynchronous teardown (HCI off completes
                              # within ~3 s; watchdog forces exit)
```

`start` accepts an optional batch override (1..80 samples per RFCOMM
frame).  Multiple-start is rejected with `already running`.  `stop`
posts a teardown_kick callback via
`btstack_run_loop_execute_on_main_thread()` so the FSM advances on the
btstack main thread.

## BT state machine

```
   OFF ──short/long press──▶ ADVERTISING ──pairing OK──▶ PAIRED
                                  │                          │
                                  │  pairing FAIL            │  long press
                                  ▼                          ▼
                             FAIL_BLINK ─auto──▶ OFF         OFF
                                                              ▲
                                  PAIRED ─link drop──▶ ADVERTISING
```

- States: `OFF`, `ADVERTISING`, `FAIL_BLINK`, `PAIRED`.  `status`
  prints `bt: <state>`.
- Short press: only OFF/FAIL_BLINK → ADVERTISING; no-op otherwise.
- Long press: OFF/FAIL_BLINK → ADVERTISING, ADVERTISING → OFF,
  PAIRED → RFCOMM disconnect + OFF.
- LED: OFF=off, ADVERTISING=blue 1 Hz blink, PAIRED=solid blue,
  FAIL_BLINK=`CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS` short blue
  pulses (~150 ms × 2N).
- Pairing completion routes through
  `HCI_EVENT_SIMPLE_PAIRING_COMPLETE` (status≠0 → FAIL_BLINK); SPP
  channel up routes through `RFCOMM_EVENT_CHANNEL_OPENED` → PAIRED.
- All transitions run on the BTstack main thread (worker / shell
  callers go through `btstack_run_loop_execute_on_main_thread()`).

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

Little-endian, 12-byte header + 8 samples × 12 bytes = 108 bytes per
frame at the Kconfig default; up to 80 samples per frame are supported.

```c
struct spp_frame_hdr {
    uint16_t magic;          // 0xA55A
    uint16_t seq;            // monotonic per frame
    uint32_t timestamp_us;   // first sample's hardware timestamp,
                             // microseconds since session start
    uint16_t sample_rate;    // 833 Hz, informational
    uint8_t  sample_count;   // typically 8 (Kconfig default), up to 80
    uint8_t  type;           // 0x01 = IMU
};

struct imu_sample {
    int16_t ax, ay, az;      // LSM6DSL accel chip-frame raw LSB
    int16_t gx, gy, gz;      // LSM6DSL gyro  chip-frame raw LSB
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

nsh> dmesg | grep BT
BT: CC2564C powered, /dev/ttyBT ready

nsh> btsensor &
btsensor [5:100]
btsensor: bringing up btstack on /dev/ttyBT
btsensor: HCI working, BD_ADDR F8:2E:0C:A0:3E:64 — advertising as "SPIKE-BT-Sensor"
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

To verify a given adapter run `tools/rfcomm_receive.py --duration 30
--decode`; if the trailing `link ODR` does not collapse to ~0 within a
few seconds, the adapter is OK.

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
