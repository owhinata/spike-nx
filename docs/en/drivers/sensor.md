# LEGO Sensor uORB Driver

## 1. Overview

A NuttX uORB sensor driver (Issue #45) layered on top of the LUMP UART
protocol engine (Issue #43).  Six topics — `/dev/uorb/sensor_lego[0..5]`
— are statically registered at boot and publish telemetry from any
LUMP-capable Powered Up device (LPF2 sensors and motor encoder
telemetry) as a fixed 56 byte `struct lump_sample_s` envelope.

Mode switching and writable-mode TX (e.g. Color sensor RGB LED control)
go through custom ioctls on the same fd via `sensor_lowerhalf_s::ops->control()`.
Multi-subscriber + single-control-owner arbitration is handled by the
`LEGOSENSOR_CLAIM` / `LEGOSENSOR_RELEASE` pair.

## 2. Architecture

```
[user]   apps/legosensor/legosensor_main.c (CLI)
         apps/system/sensortest, user apps
   |  open(/dev/uorb/sensor_legoN) + read() + ioctl()
   |
[kernel uORB upper-half]  nuttx/drivers/sensors/sensor.c
   |  push_event() -> ring buffer (nbuffer=16) -> poll() wake
   |
[kernel lower-half]  boards/spike-prime-hub/src/legosensor_uorb.c
   |    per-port struct legosensor_dev_s
   |    - sensor_lowerhalf_s, mutex, claim_owner, generation, ...
   |  ^ on_sync / on_data / on_error callbacks
   |
[kernel] LUMP engine (#43)  stm32_legoport_lump.c
   |
[kernel] DCM (#42)
```

### 2.1 Registration policy

- **Static register at boot**: all six `/dev/uorb/sensor_legoN` are
  registered by `sensor_custom_register()` in one bringup pass
- Empty ports publish nothing — subscribers wait via `poll()`
- `lump_attach` lifetime contract: physical disconnect does **not**
  detach the publisher; the LUMP engine handles ERR / backoff / re-SYNC
  internally and re-fires `on_sync` after the next handoff

### 2.2 Callback context

- `on_sync` / `on_data` / `on_error` run in the LUMP per-port kthread
  context **after** the LUMP-side per-port lock has been released
- The driver protects state with a per-port `nxmutex_t`; the mutex is
  always released before calling `push_event()` (lock-order discipline,
  deadlock avoidance)

## 3. Sample envelope

```c
struct lump_sample_s
{
  uint64_t timestamp;       /* µs (sensor framework convention) */
  uint32_t seq;             /* per-port monotonic counter */
  uint32_t generation;      /* +1 on attach / SELECT confirmation /
                             * disconnect */
  uint8_t  port;            /* 0..5 (A..F) */
  uint8_t  type_id;         /* LPF2 type (0 = disconnect sentinel) */
  uint8_t  mode_id;         /* mode reported in this DATA frame */
  uint8_t  data_type;       /* enum lump_data_type_e */
  uint8_t  num_values;      /* INFO_FORMAT[2] */
  uint8_t  len;             /* live byte count (0 = sentinel) */
  uint16_t reserved;
  union
  {
    uint8_t  raw [32];
    int8_t   i8  [32];
    int16_t  i16 [16];
    int32_t  i32 [8];
    float    f32 [8];
  } data;
};
_Static_assert(sizeof(struct lump_sample_s) == 56, "ABI");
_Static_assert(offsetof(struct lump_sample_s, data) == 24, "ABI");
```

56 bytes total, fixed.  Consumers `switch` on `data_type` and read
`data.i16[k]` etc. directly — no cast boilerplate.

### 3.1 Sentinel encoding

| Sentinel | Condition | Meaning |
|---|---|---|
| sync sentinel | `type_id != 0 && len == 0` | SYNC complete; pre-warms the upper-half lazy circbuf |
| disconnect sentinel | `type_id == 0 && len == 0` | DCM disconnect (`on_error`); no further samples until a new SYNC |

### 3.2 `generation` semantics

| Event | generation |
|---|---|
| First SYNC complete | +1 (carried in the sync sentinel) |
| `LEGOSENSOR_SELECT` confirmed | +1 (bumped in `on_data` once the device starts reporting the requested mode) |
| SELECT timeout (500 ms) | unchanged (subscribers don't drop valid samples) |
| Disconnect | +1 (carried in the disconnect sentinel) |

A subscriber that snapshots `generation` immediately before SELECT can
identify the new-mode samples by the combination of `generation` jump
and `mode_id` change.

## 4. Custom ioctls

| Cmd | Arg | Purpose | Claim |
|---|---|---|---|
| `LEGOSENSOR_GET_INFO` | `lump_device_info_s *` | per-mode schema dump | -- |
| `LEGOSENSOR_GET_STATUS` | `lump_status_full_s *` | engine status (state/baud/RX/TX/...) | -- |
| `LEGOSENSOR_CLAIM` | (none) | take control ownership (filep) | -- |
| `LEGOSENSOR_RELEASE` | (none) | release control ownership | owner only |
| `LEGOSENSOR_SELECT` | `uint8_t mode` | mode switch (`lump_select_mode`) | required |
| `LEGOSENSOR_SEND` | `legosensor_send_arg_s *` | writable-mode TX (`lump_send_data`) | required |

### 4.1 Standard sensor ioctls

- `SNIOC_SET_INTERVAL`: dispatched by the upper-half directly to
  `lower->ops->set_interval()`; this driver no-ops it because the LUMP
  cadence is dictated by the device firmware
- `SNIOC_ACTIVATE` does **not** exist in NuttX 12.13.0's sensor ioctl
  set.  Activation is driven through `nsubscribers` count crossings
  invoking `lower->ops->activate()` (no-op here — the LUMP engine is
  always running)

### 4.2 CLAIM / RELEASE arbitration

- Read (sample stream) and `GET_INFO` / `GET_STATUS` are unrestricted —
  multi-subscriber friendly
- SELECT and SEND require the calling fd to hold the per-port CLAIM
- A second CLAIM by another fd returns `-EBUSY`
- A re-CLAIM by the current owner is idempotent (no-op, OK)
- SELECT/SEND from a non-owner fd returns `-EACCES`
- **Auto-release**: per-fd `close()` triggers `sensor_ops_s::close(lower, filep)`
  in NuttX 12.13.0 (sensor.c:788, 814).  The driver clears
  `claim_owner` when it matches the closing `filep`, so a forgotten
  `LEGOSENSOR_RELEASE` is harmless
- DCM disconnect (`on_error`) also clears `claim_owner` immediately

### 4.3 SELECT API contract

`LEGOSENSOR_SELECT` only reports whether the request was queued at the
LUMP engine.  **Mode-change confirmation is observed by subscribers
through `seq` / `generation` / `mode_id`** because:

- `lump_select_mode` enqueues a CMD SELECT for the kthread to send
- The device takes ~10 ms to react and start emitting the new mode
- The only reliable signal is the next `on_data` whose `mode` matches

The driver tracks `pending_select_mode` plus a deadline (now + 500 ms)
under the per-port mutex.  In `on_data`:

```c
nxmutex_lock(&priv->lock);
now_ticks = clock_systime_ticks();
if (priv->pending_select_mode != UINT8_MAX)
{
    if ((int32_t)(now_ticks - priv->pending_select_deadline) >= 0)
        priv->pending_select_mode = UINT8_MAX;       /* expired */
    else if (frame->mode == priv->pending_select_mode)
    {
        priv->generation++;
        priv->pending_select_mode = UINT8_MAX;       /* confirmed */
    }
}
priv->mode_id = frame->mode;
priv->seq++;
sample = build_envelope(priv, frame);                /* local copy */
nxmutex_unlock(&priv->lock);
priv->lower.push_event(priv->lower.priv, &sample, sizeof(sample));
```

Comparing `(int32_t)(now - deadline) >= 0` keeps the test wraparound-safe
across the `uint32_t` tick counter.

## 5. CLI (`legosensor`)

```
legosensor                     6-port summary (type/state/mode/RX/TX)
legosensor list                same as above
legosensor info <N>            per-mode schema (name, num_values,
                               data_type, raw/pct/si min-max, units,
                               writable)
legosensor mode <N> <m>        CLAIM -> SELECT -> poll(2) 500 ms ->
                               RELEASE
legosensor send <N> <m> <hex>...  CLAIM -> SEND (writable-mode payload)
                                  -> RELEASE
legosensor watch <N> [ms]      open(O_NONBLOCK) -> poll(2) -> read ->
                               decoded print loop
legosensor claim <N>           explicit CLAIM (released on fd close)
legosensor release <N>         explicit RELEASE
```

`N` accepts `0..5` or `A..F`.

### 5.1 Why `dd` / `sensortest` don't work

- **`dd`** issues a single `read()` without `poll(2)`.  The upper-half
  non-fetch read path returns `-ENODATA` synchronously when the
  caller's user generation is up to date, so a race against
  `push_event()` produces `Unknown error 61` (ENODATA)
- **`sensortest`** validates the device name against
  `g_sensor_info[]`'s standard types (accel0, gyro0, ...) and rejects
  the custom `sensor_lego[N]` path with
  `The sensor node name:sensor_legoN is invalid`

`legosensor watch` is the canonical end-to-end verification tool.

## 6. Device support

### 6.1 Verified on hardware (SYNCED)

| Type | Name | num_modes | Default mode |
|---|---|---|---|
| 48 | SPIKE Medium Motor | 6 | POWER (mode 0) |
| 49 | SPIKE Large Motor | 6 | POWER (mode 0) |
| 61 | SPIKE Color Sensor | 8 | COLOR (mode 0) |
| 62 | SPIKE Ultrasonic Sensor | 8 | DISTL (mode 0) |
| 63 | SPIKE Force Sensor | 7 | FORCE (mode 0) |

### 6.2 LUMP-protocol-compatible (theoretically supported)

The exhaustive type-ID list lives in [port-detection.md](port-detection.md)
§5.  `/dev/uorb/sensor_lego[N]` works for any LUMP-capable device,
including Mindstorms EV3 family.

### 6.3 Dependency on the #44 motor driver (LED / motor actuation)

LED control on the SPIKE Color / Ultrasonic sensors (mode 3 / mode 5
LIGHT) and motor PWM drive both depend on the **#44 H-bridge motor
driver (TIM1/3/4)**.

- The pybricks reference (`pbio/drv/legodev/legodev_pup_uart.c:894-900`)
  drives the H-bridge to `-MAX_DUTY` (Pin 1 supply) or `+MAX_DUTY`
  (Pin 2 supply) right after SYNC to power the device's LEDs.
  `pbdrv_legodev_spec_basic_flags()` maps
  `SPIKE_COLOR_SENSOR / SPIKE_ULTRASONIC_SENSOR ->
  NEEDS_SUPPLY_PIN1` and
  `TECHNIC_COLOR_LIGHT_MATRIX -> NEEDS_SUPPLY_PIN2`
- In this build (#45 only, #44 not yet implemented) the
  `LEGOSENSOR_SEND` wire transmission succeeds (the TX byte counter
  increments and the wire format matches pybricks) but the physical
  LED does not light because the supply pin is not driven
- Motor `SEND POWER` similarly requires the H-bridge to actually
  spin the rotor
- The LUMP frames are reliably leaving the hub — the current limit
  is only the absent supply rail.  Once #44 lands, the same
  `LEGOSENSOR_SEND` ioctl will drive LEDs and motors

## 7. Design choices

### 7.1 uORB primary, not chardev (`/dev/legosensor[N]`)

Two rounds of Codex review settled on uORB for `/dev/uorb/sensor_lego[N]`:

- Same sensor framework as the IMU (`/dev/uorb/sensor_imu0`); the same
  `poll()` loop can fuse both sources
- Multi-subscriber is natural (control app + monitor app coexist)
- Ring buffer, batch, `poll(2)` come for free from the framework
- The dynamic LUMP mode / shape fits a fixed envelope + C union under
  `SENSOR_TYPE_CUSTOM`

### 7.2 nbuffer = 16 (per instance)

- LUMP cadence is ~10..100 Hz; subscribers typically read every 100 ms
- 16 frames covers a 100 Hz × 100 ms loop without overrun
- The circbuf is per-instance, not per-subscriber, so RAM does not
  grow with user count
- Memory: heap allocation `16 × 56 B × 6 ports = 5.4 KB` plus
  upper-half/user bookkeeping

### 7.3 Reject `O_WROK`

`sensor_ops_s::open` returns `-EACCES` if `(filep->f_oflags & O_WROK) != 0`.
This stops userspace from injecting fake samples through
`sensor_write()` and bypassing the LUMP transport.

### 7.4 Boundary with the future #44 motor driver

BOOST / Technic motors are used **simultaneously as encoder publishers
and as PWM actuators** in pybricks workflows.  The agreed split is:

- **#43 LUMP** holds one engine per port, but allows multiple
  publisher attaches
- **#45** is the telemetry publisher (attaches at boot, never detaches
  on physical disconnect)
- **#44** will be the writer / control owner (chardev driving PWM
  plus LUMP SELECT / SEND)

When #44 lands, #43 will gain
`lump_acquire_control(port, owner_token)` / `lump_release_control(port, owner_token)`,
SELECT/SEND will take a token argument, and the legacy token-less
`lump_select_mode` / `lump_send_data` will only succeed when the port
has no current owner (no NULL-bypass loophole).  **#45 does not take
the LUMP control owner at boot** — the user fd takes it on the first
`LEGOSENSOR_CLAIM`.

## 8. Tuning constants

| Constant | Value | Description |
|---|---|---|
| `LEGOSENSOR_NUM_PORTS` | 6 | `/dev/uorb/sensor_lego0..5` |
| `LEGOSENSOR_MAX_DATA_BYTES` | 32 | LUMP payload cap |
| `LEGOSENSOR_NBUFFER` | 16 | upper-half circbuf depth |
| `LEGOSENSOR_PENDING_TIMEOUT_MS` | 500 | SELECT-confirmation deadline |

## 9. References

- [lump-protocol.md](lump-protocol.md), [port-detection.md](port-detection.md)
- Public ABI: `boards/spike-prime-hub/include/board_legosensor.h`
- Driver: `boards/spike-prime-hub/src/legosensor_uorb.c`
- CLI: `apps/legosensor/legosensor_main.c`
- NuttX sensor framework: `nuttx/include/nuttx/sensors/sensor.h`,
  `nuttx/drivers/sensors/sensor.c`
