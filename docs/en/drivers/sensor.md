# LEGO Sensor uORB Driver

## 1. Overview

A NuttX uORB sensor driver (Issue #79) layered on top of the LUMP UART
protocol engine (Issue #43).  Topics are organised **by device class**,
so opening `/dev/uorb/sensor_color` always gives a colour sensor stream
and `/dev/uorb/sensor_motor_r` always gives the right-wheel motor.

| Topic | LPF2 type_id | Port restriction |
|---|---|---|
| `/dev/uorb/sensor_color` | 61 | any port |
| `/dev/uorb/sensor_ultrasonic` | 62 | any port |
| `/dev/uorb/sensor_force` | 63 | any port |
| `/dev/uorb/sensor_motor_m` | 49 (SPIKE Large Motor) | any port |
| `/dev/uorb/sensor_motor_r` | 48 (SPIKE Medium Motor) | port 0 / 2 / 4 (A / C / E) |
| `/dev/uorb/sensor_motor_l` | 48 (SPIKE Medium Motor) | port 1 / 3 / 5 (B / D / F) |

Suffixes: `_m` is the high-torque arm / manipulator motor (type 49 =
Large); `_r` / `_l` are right / left driving wheels (type 48 = Medium,
split by port parity).  Unmapped type IDs (38 / 46 / 47 / 65 / 75 /
76, …) are dropped.

Mode switching, writable-mode TX, and LED / PWM control go through the
custom ioctls below via `sensor_lowerhalf_s::ops->control()`.
Multi-subscriber + single-control-owner arbitration is handled by the
`LEGOSENSOR_CLAIM` / `LEGOSENSOR_RELEASE` pair.

## 2. Architecture

```
[user]   apps/sensor/sensor_main.c (CLI)
   |  open(/dev/uorb/sensor_<class>) + read() + ioctl()
   |
[kernel uORB upper-half]  nuttx/drivers/sensors/sensor.c
   |  push_event() -> ring buffer (nbuffer=16) -> poll() wake
   |
[kernel lower-half]  boards/spike-prime-hub/src/legosensor_uorb.c
   |    6 class topics (static) -+-> classifier(type_id, port)
   |    6 port bindings        --+      |
   |                                    v
   |                                 g_legosensor_class_state[class].bound_port
   |  ^ on_sync / on_data / on_error per port (LUMP kthread context)
   |
[kernel] LUMP engine (#43)  stm32_legoport_lump.c
   |
[kernel] DCM (#42)
```

### 2.1 Registration policy

- **Static register at boot**: six class topics under `/dev/uorb/sensor_*`
  are registered in one bringup pass before any `lump_attach`
- Empty ports publish nothing — subscribers wait via `poll()`
- Registration order is (1) all six class topics, (2) `lump_attach()`
  per port.  `lump_attach()` may fire `on_sync` synchronously when the
  port is already SYNCED, so the push_event targets must already exist

### 2.2 1-topic = 1-port rule

Each class topic is bound to **at most one port at any instant**.  When
two ports classify the same way, the lower-numbered port wins; the
loser's frames are dropped.

- `on_sync(port, info)` runs `classify(type_id, port)` to obtain the
  candidate class slot
- Under the class-global `g_bind_lock`:
  - slot empty → bind this port
  - slot owned by a port with a higher number → **takeover** (this
    port wins, the displaced port is demoted to frame-drop and the
    class topic emits a disconnect sentinel)
  - slot owned by a port with a lower number → this port loses,
    frames are dropped
- `on_data(port, mode, …)`: only the bound port pushes to the class
  lower-half; unbound ports are dropped silently
- `on_error(port)`: if bound, emits a disconnect sentinel and frees
  the class slot

> **No automatic rebind.** When a class is freed, the driver does
> not silently rebind another connected port of the same type.  The
> port has to disconnect/reconnect (re-firing `on_sync`) to take the
> slot.  This keeps the routing predictable.

### 2.3 Callback context · lock order

- `on_sync` / `on_data` / `on_error` run in the LUMP per-port kthread
  context after the LUMP-side per-port lock has been released
- `g_bind_lock` (class-global) and per-port locks are **never held
  together**.  Each lock is acquired, the relevant state read, the
  lock dropped, and only then is the other lock taken
- `push_event()` is always called outside both locks (avoids cycles
  with the upper-half `nxrmutex_t`)

### 2.4 Handling multiple sensors of the same class

This driver intentionally does not aggregate multiple ports of the
same class onto one topic.  When you need to handle two of the same
sensor:

- **LUMP direct**: use the `/dev/legoport[N]` chardev (Issue #43)
  with the `LEGOPORT_LUMP_*` ioctls — port-scoped, no class layer
- **Pair the class topics in userspace**: e.g. the drivebase
  daemon (Issue #77) reads `sensor_motor_l` and `sensor_motor_r`
  separately and pairs them logically

## 3. Sample envelope

```c
struct lump_sample_s
{
  uint64_t timestamp;       /* µs (sensor framework convention) */
  uint32_t seq;             /* per-port monotonic counter */
  uint32_t generation;      /* +1 on SYNC / SELECT confirmation /
                             * disconnect / takeover */
  uint8_t  port;            /* 0..5 — bound port at the time of emit */
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
| disconnect sentinel | `type_id == 0 && len == 0` | port disconnected (`on_error`) **or** port displaced by a takeover |

### 3.2 `generation` semantics

| Event | generation |
|---|---|
| First SYNC complete | +1 (carried in the sync sentinel) |
| `LEGOSENSOR_SELECT` confirmed | +1 (bumped in `on_data` once the device starts reporting the requested mode) |
| SELECT timeout (500 ms) | unchanged (subscribers don't drop valid samples) |
| Disconnect / takeover | +1 (carried in the disconnect sentinel) |

A subscriber that snapshots `generation` immediately before SELECT can
identify the new-mode samples by the combination of `generation` jump
and `mode_id` change.  A change in `port` field flags a takeover.

## 4. Custom ioctls

| Cmd | Arg | Purpose | Claim |
|---|---|---|---|
| `LEGOSENSOR_GET_INFO` | `legosensor_info_arg_s *` | bound port + per-mode schema | -- |
| `LEGOSENSOR_GET_STATUS` | `lump_status_full_s *` | engine status (state/baud/RX/TX/...) | -- |
| `LEGOSENSOR_CLAIM` | (none) | take control ownership (filep) | -- |
| `LEGOSENSOR_RELEASE` | (none) | release control ownership | owner only |
| `LEGOSENSOR_SELECT` | `legosensor_select_arg_s *` | mode switch (`lump_select_mode`) | required |
| `LEGOSENSOR_SEND` | `legosensor_send_arg_s *` | writable-mode TX (`lump_send_data`) | required |
| `LEGOSENSOR_SET_PWM` | `legosensor_pwm_arg_s *` | LED / motor PWM (class-aware routing) | required |

Reserved class-specific ioctl ranges (currently `-ENOTTY`, allocated
for future class-specific commands):

| Class | Range |
|---|---|
| COLOR | `_SNIOC(0x0100..0x010f)` |
| ULTRASONIC | `_SNIOC(0x0110..0x011f)` |
| FORCE | `_SNIOC(0x0120..0x012f)` |
| MOTOR_M | `_SNIOC(0x0130..0x013f)` |
| MOTOR_R | `_SNIOC(0x0140..0x014f)` |
| MOTOR_L | `_SNIOC(0x0150..0x015f)` |

### 4.1 errno surface

| ioctl | errno | When |
|---|---|---|
| CLAIM | `-ENODEV` | no port bound to this class |
|  | `-EBUSY` | another fd holds CLAIM (re-CLAIM by the existing owner is idempotent) |
| RELEASE | `-EACCES` | not the current owner / claim is stale |
| SELECT / SEND | `-EACCES` | caller does not hold CLAIM |
|  | `-ENODEV` | port disconnected or replaced since CLAIM (re-CLAIM required) |
| SET_PWM | `-EACCES` | caller does not hold CLAIM |
|  | `-ENODEV` | same as SELECT/SEND |
|  | `-ENOTSUP` | force / motor_* (this release) / firmware lacks LIGHT mode of the expected shape (3×INT8 for color, 4×INT8 for ultrasonic) |
|  | `-EINVAL` | `num_channels` mismatched, channel out of range (LED 0..10000, motor -10000..10000, negative LED is `-EINVAL`) |
| GET_INFO / GET_STATUS | `-ENODEV` | no port bound |

### 4.2 CLAIM / RELEASE arbitration

- Reads (sample stream) and `GET_INFO` / `GET_STATUS` are unrestricted —
  multi-subscriber friendly
- SELECT / SEND / SET_PWM require the calling fd to hold the CLAIM
- A second CLAIM by another fd returns `-EBUSY`
- A re-CLAIM by the current owner is idempotent (no-op, OK)
- Write ioctls from a non-owner fd return `-EACCES`
- **Stale claim**: when a port is taken over or disconnected after a
  CLAIM, the driver bumps an internal `bind_generation`.  The owner
  fd's next write ioctl returns `-ENODEV`, indicating the subscriber
  must re-CLAIM
- **Auto-release**: per-fd `close()` triggers `sensor_ops_s::close(lower, filep)`,
  which clears `claim_owner` if it matches the closing `filep` — a
  forgotten `LEGOSENSOR_RELEASE` is harmless

### 4.3 SELECT API contract

`LEGOSENSOR_SELECT` only reports whether the request was queued at the
LUMP engine.  **Mode-change confirmation is observed by subscribers
through `seq` / `generation` / `mode_id`** because:

- `lump_select_mode` enqueues a CMD SELECT for the kthread to send
- The device takes ~10 ms to react and start emitting the new mode
- The only reliable signal is the next `on_data` whose `mode` matches

The driver tracks `pending_select_mode` plus a deadline (now + 500 ms)
under the per-port mutex; in `on_data`, when `frame->mode == pending`
within the deadline, generation is bumped and pending cleared.
Expired requests are silently dropped.

### 4.4 SET_PWM (LED / motor PWM)

| Class | Implemented | Alternative / Backend | Channel meaning |
|---|---|---|---|
| COLOR | `-ENOTSUP` (Issue #92) | use `LEGOSENSOR_SEND mode=3` (LIGHT, 3×INT8 PCT) directly | — |
| ULTRASONIC | `-ENOTSUP` (Issue #92) | use `LEGOSENSOR_SEND mode=5` (LIGHT, 4×INT8 PCT) directly | — |
| FORCE | `-ENOTSUP` (permanent) | — | no actuator on the device |
| MOTOR_M / R / L | `-ENOTSUP` (this release) | STM32 TIM PWM (lands with Issue #80) | channels[0] = signed duty -10000..10000 |

**SET_PWM semantics**: reserved for **H-bridge physical PWM** (motors).
LIGHT-mode brightness on COLOR / ULTRASONIC is a writable-mode WRITE,
not PWM in the H-bridge sense, so it has been removed from SET_PWM in
Issue #92.  Callers should use `LEGOSENSOR_SEND` (mode=3 for COLOR,
mode=5 for ULTRASONIC) directly.  Brightness payload is INT8 PCT
(0..100) per channel.

LIGHT writes via `LEGOSENSOR_SEND` (Issue #92):

- The user app sends a `legosensor_send_arg_s` with `mode = 3` for
  COLOR or `mode = 5` for ULTRASONIC and one INT8 PCT byte per LED
  in `data[0..N-1]` (range 0..100, no scaling on the kernel side).
- `lump_send_data(port, mode, data, len)` is the engine entry point.
  When `current_mode != mode` it queues `CMD SELECT mode` ahead of
  the DATA frame in the same drain pass, so the engine's
  `current_mode` ends up aligned with LIGHT.  The earlier
  implementation skipped SELECT and relied on "writable-mode SEND is
  independent of the active SELECT" — but the SPIKE Color Sensor's
  firmware-driven auto-LED policy overrides any LIGHT value written
  in another mode, so pre-SELECTing is needed to make the
  brightness actually stick.  Behaviour now matches pybricks' send-
  thread (SELECT → wait → WRITE).
- (Historical note) The LUMP wire format itself does allow a
  writable-mode WRITE without a preceding SELECT — that path is what
  the engine was originally relying on.  Color/Ultrasonic firmware
  quirks are the reason we no longer use it.

> **Physical LED illumination depends on the H-bridge supply pin
> (lands with Issue #80).**
>
> The SPIKE Color Sensor (`NEEDS_SUPPLY_PIN1`) and Ultrasonic Sensor
> (`NEEDS_SUPPLY_PIN1`) require the H-bridge to be driven at
> `-MAX_DUTY` after SYNC to power the device's LEDs (see the pybricks
> reference at `pbio/drv/legodev/legodev_pup_uart.c:894-900` and the
> capability map at `legodev_spec.c:201-208`).  This release (#79)
> does not yet ship the H-bridge driver, so `LEGOSENSOR_SEND` for
> mode=LIGHT emits the LUMP frame onto the wire (visible as
> `tx_bytes` growth) but **the physical LED stays dark for lack of
> supply voltage**.  Once Issue #80 lands, the H-bridge will be driven
> automatically on SYNC and the brightness commanded here will
> also light the LEDs.

### 4.5 SPIKE Color Sensor firmware-autonomous LED policy (empirical)

> Not in any LEGO-published spec.  The pybricks reference
> implementation has always been built on top of this assumption, and
> we **re-verified it on hardware on 2026-05-01** (Hub firmware =
> `main`, sensor 45605).

The SPIKE Color Sensor (type_id=61) drives its three on-board LEDs
**autonomously based on the SELECTed LUMP mode**.  Neither
`stm32_legoport_lump.c` nor `legosensor_uorb.c` carries any mode → LED
table; we just forward the `CMD SELECT` and the sensor firmware
decides what to do with the LEDs.

| mode | name | observed LED default | physical meaning |
|---|---|---|---|
| 0 | COLOR | ON  | colour decision from reflected light → needs LEDs |
| 1 | REFLT | ON  | reflected-light intensity |
| 2 | AMBI  | OFF | ambient light — LEDs would corrupt the reading |
| 3 | LIGHT | OFF (writable can override) | dedicated LED-control mode, parked off until userspace writes |
| 4 | RREFL | ON  | raw reflected pair |
| 5 | RGB I | ON  | RGB + clear/IR — needs LEDs |
| 6 | HSV   | ON  | HSV under LED illumination (pybricks `surface=True`) |
| 7 | SHSV  | OFF | HSV under ambient light (pybricks `surface=False`) |

**Brightness written via LIGHT (mode 3) is local to mode 3 only**
(hypothesis A, confirmed empirically):

- While SELECT == 3, writes to LIGHT (`LEGOSENSOR_SET_PWM` /
  `LUMP_SEND`) drive the LEDs immediately
- As soon as SELECT moves to a different mode (e.g. mode 5 RGB_I), the
  firmware overwrites the LED state with **its own default brightness
  for that measurement mode** — the user-supplied value is discarded
- Going back to mode 3 leaves the LEDs OFF until you write again

So SET_PWM is useful for "drive the sensor as a controllable LED
array" but **not** for "dim the measurement-mode LEDs of mode 5/6/etc"
— firmware always wins for those modes.

### 4.6 kernel ↔ userspace responsibility split

| Layer | Responsibility | Example |
|---|---|---|
| **sensor firmware (LEGO)** | Owns per-mode LED ON/OFF policy autonomously; not reachable from the Hub | SELECT(5) → LED auto-ON, SELECT(7) → LED auto-OFF (table in 4.5) |
| **kernel driver (this code)** | Mechanism only.  Forwards SELECT / SEND / SET_PWM verbatim.  No mode-aware LED policy of its own | SELECT(0) issues a CMD SELECT and never touches LED state — the actual ON/OFF happens inside the sensor firmware |
| **userspace helper library (Issue #78)** | Optional override layer for callers that want to **defy** the firmware default.  Examples: "keep LEDs off in mode 0", "pin LIGHT to 50 % whenever in mode 3" | The firmware default is already correct for most use cases, so #78 is only needed when an app wants to subvert it |

Standard NuttX "mechanism in kernel, policy in userspace".
Subscribers may still call SELECT / SET_PWM directly if they want
finer control — the helper library is convenience, not enforcement.

### 4.7 SPIKE Ultrasonic Sensor modes (45604)

The LEGO marketing spec quotes 100 Hz sample rate but it isn't carried
in LUMP info; firmware output rate varies per mode (Issue #92 observed
~110 fps in motion, ~67 fps stationary on mode 5).

| mode | Name | Size | pybricks API | Purpose |
|---|---|---|---|---|
| 0 | DISTL | 1×INT16 | `distance` | distance (cm), long range |
| 1 | DISTS | 1×INT16 | — | short-range distance (pybricks unused) |
| 2 | SINGL | 1×INT16 | — | single-ping distance (pybricks unused) |
| 3 | LISTN | 1×INT8 | `presence` | other-ultrasonic-signal detection (0/1) |
| 4 | TRAW | 1×INT32 | — | raw time-of-flight (pybricks unused) |
| 5 | LIGHT | 4×INT8 (writable) | `lights` | 4 eye-LED brightness (0..100 %) |
| 6 | PING | 1×INT8 (writable, raw 0..1) | — | pybricks marks this "??" (`legodev.h:333`).  Hardware INFO confirms **writable + raw 0..1 + unit PCT** (`sensor ultrasonic info` 2026-05-02), strongly suggesting "fire one ping" trigger semantics (write 1 = fire, 0 = idle).  pybricks does not invoke it, side-effects unverified — leave alone unless explicitly needed |
| 7 | ADRAW | 1×INT16 | — | raw ADC value (pybricks unused) |
| 8 | CALIB | 7×INT16 | — | calibration (pybricks unused) |

Port supply: same as COLOR — `NEEDS_SUPPLY_PIN1`, the H-bridge must be
driven `-MAX_DUTY` after SYNC to power the LEDs (Issue #80).
LIGHT-mode brightness is assumed to follow the same firmware-gating
behaviour as COLOR (only effective while mode 5 is the active SELECT —
hypothesis A in §4.5); ULTRASONIC re-verification is still TODO.

## 5. CLI (`sensor`)

```
sensor                                 list all class topics (status one-liner each)
sensor list                            same as above
sensor <class>                         status one-liner
sensor <class> info                    bound port + per-mode schema
sensor <class> status                  engine + traffic counters
sensor <class> watch [ms]              poll -> read decoded samples (default 1000)
sensor <class> fps [ms]                rate-only count, no per-sample print
sensor <class> select <mode>           open -> CLAIM -> SELECT -> close (auto-RELEASE)
sensor <class> send <mode> <hex>...    open -> CLAIM -> SEND -> close
sensor <class> pwm <ch0> [ch1 ch2 ch3] open -> CLAIM -> SET_PWM -> close
```

`<class>` is `color | ultrasonic | force | motor_m | motor_r | motor_l`.

Each write-side command is **self-contained**: open → CLAIM → operate
→ close (auto-RELEASE).  When you need to hold a claim across
multiple operations, write a long-running daemon (see Issue #77's
drivebase for an example).

### 5.1 Examples

```sh
# Plug a Color sensor into port A
sensor color info                    # bound port=A, modes listed
sensor color watch                   # 1 s of decoded samples
sensor color select 1                # switch to REFLT (mode 1)
sensor color pwm 5000 0 0            # LED0 at 50 %
sensor color pwm 0 0 0               # all LEDs off

# Plug an Ultrasonic sensor elsewhere
sensor ultrasonic pwm 5000 5000 0 0  # top two eye LEDs at 50 %

# Force / motor SET_PWM is unsupported in this release
sensor force pwm 0                   # -ENOTSUP
sensor motor_m pwm 0                 # -ENOTSUP
```

### 5.2 Why `dd` / `sensortest` don't work

- **`dd`** issues a single `read()` without `poll(2)`.  The upper-half
  non-fetch read path returns `-ENODATA` synchronously if there is no
  newer sample, so a race against `push_event()` produces
  `Unknown error 61` (ENODATA)
- **`sensortest`** validates the device name against
  `g_sensor_info[]`'s standard types (accel0, gyro0, …) and rejects
  the custom `sensor_<class>` paths

`sensor watch` is the canonical end-to-end verification tool.

## 6. Device support

### 6.1 Verified on hardware (SYNCED)

| Type | Name | num_modes | Default mode | Class topic |
|---|---|---|---|---|
| 48 | SPIKE Medium Motor | 6 | POWER (mode 0) | `sensor_motor_r` (port A/C/E) / `sensor_motor_l` (port B/D/F) |
| 49 | SPIKE Large Motor | 6 | POWER (mode 0) | `sensor_motor_m` |
| 61 | SPIKE Color Sensor | 8 | COLOR (mode 0) | `sensor_color` |
| 62 | SPIKE Ultrasonic Sensor | 8 | DISTL (mode 0) | `sensor_ultrasonic` |
| 63 | SPIKE Force Sensor | 7 | FORCE (mode 0) | `sensor_force` |

### 6.2 Unmapped type IDs

Powered Up devices with type ID 38 / 46 / 47 / 65 / 75 / 76 (and
similar) are dropped by the classifier.  They remain accessible
through the LUMP-direct `/dev/legoport[N]` chardev with port-scoped
`LEGOPORT_LUMP_*` ioctls.

## 7. Follow-up issues

| Issue | Scope |
|---|---|
| #80 | Motor PWM H-bridge driver (STM32 TIM).  MOTOR_M / R / L SET_PWM moves from `-ENOTSUP` to a real implementation |
| #78 | Userspace helper library `apps/legolib/` (per-class policy: mode → LED automation, …) |
| #77 | drivebase userspace daemon (`apps/drivebase/`, paired left/right wheel control; depends on #78 + #80) |

## 8. Tuning constants

| Constant | Value | Description |
|---|---|---|
| `LEGOSENSOR_NUM_PORTS` | 6 | physical ports (A..F) |
| `LEGOSENSOR_CLASS_NUM` | 6 | class topics |
| `LEGOSENSOR_MAX_DATA_BYTES` | 32 | LUMP payload cap |
| `LEGOSENSOR_NBUFFER` | 16 | upper-half circbuf depth (per class topic) |
| `LEGOSENSOR_PENDING_TIMEOUT_MS` | 500 | SELECT-confirmation deadline |

## 9. References

- [lump-protocol.md](lump-protocol.md), [port-detection.md](port-detection.md)
- Public ABI: `boards/spike-prime-hub/include/board_legosensor.h`
- Driver: `boards/spike-prime-hub/src/legosensor_uorb.c`
- CLI: `apps/sensor/sensor_main.c`
- LUMP API: `boards/spike-prime-hub/include/board_lump.h`
- NuttX sensor framework: `nuttx/include/nuttx/sensors/sensor.h`,
  `nuttx/drivers/sensors/sensor.c`
