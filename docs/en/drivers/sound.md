# Sound Driver

Board-local drivers that drive the SPIKE Prime Hub's built-in speaker from NuttX. Waveforms generated on DAC1 CH1 are routed through an external amplifier to the speaker.

## Hardware

| Signal | Pin / Peripheral | Notes |
|--------|------------------|-------|
| DAC output | PA4 (analog, DAC1 OUT1) | 12-bit left-aligned samples |
| Amplifier enable | PC10 (push-pull, active high) | LOW mutes the speaker |
| DMA | DMA1 Stream 5 Channel 7 | Circular transfer into `DAC1_DHR12L1` |
| Sample timer | TIM6 (basic timer) | TRGO = update, input 96 MHz |
| Waveform center | `0x8000` | Symmetric midpoint for 12-bit left-aligned DAC |
| Max sample rate | 100 kHz | Enforced by the driver |

The owhinata/nuttx fork now includes `STM32_HAVE_DAC1` select and `GPIO_DAC1_OUT1_0` pinmap macro for the F413 ([#27](https://github.com/owhinata/spike-nx/issues/27), [#28](https://github.com/owhinata/spike-nx/issues/28)).  DMA1 and TIM6 are managed through NuttX abstraction APIs (`stm32_dmachannel()` / `stm32_tim_init()`); only the DAC1 RCC clock enable remains as direct register access.

## Layered architecture

```
apps/sound (NSH builtin)
  sound beep / tone / notes / volume / off / selftest
        |           |
        v           v
  /dev/pcm0   /dev/tone0
   (raw PCM)   (tune string)
        \         /
         v       v
  stm32_sound_play_pcm / stop_pcm / set_volume
    DAC1 (register direct) / DMA1 S5 (NuttX DMA API) / TIM6 (NuttX TIM API)
```

- **Low-level layer (`stm32_sound.c`)** — provides `stm32_sound_play_pcm(data, length, sample_rate)` and `stm32_sound_stop_pcm()`.  Callers must hold `g_sound.lock`.
- **`/dev/tone0` (`stm32_tone.c`)** — char device with an in-kernel parser for tune strings.
- **`/dev/pcm0` (`stm32_pcm.c`)** — char device with a single-call raw PCM ABI (semantically equivalent to pybricks `pbdrv_sound_start`).
- **`apps/sound`** — NSH utility that drives both devices from the shell.

The low-level layer is a thin 1:1 wrapper around pybricks `pbdrv_sound_start(data, length, sample_rate)` — it has no notion of music or notes.

## Shared state and synchronization

```c
struct sound_state_s
{
    mutex_t           lock;         /* board-wide HW mutex */
    int               open_count;   /* /dev/tone0 + /dev/pcm0 combined */
    FAR struct file  *owner;        /* current playback owner */
    enum sound_mode_e mode;         /* IDLE / TONE / PCM */
    uint8_t           volume;       /* 0..100 */
    atomic_bool       stop_flag;    /* tone_write interrupt request */
};
```

- **Hardware access always runs under `g_sound.lock`**.  `stm32_sound_play_pcm` / `stm32_sound_stop_pcm` themselves do **not** take the lock — the caller must.
- **`stop_flag`** is `atomic_bool` (memory_order_relaxed).  `tone_write` clears it at entry; `pcm_write`, `TONEIOC_STOP`, and self-owner `close()` set it to true.  `pcm_write` does not clear it — doing so would wipe out a concurrent stop request aimed at tone playback.
- **`close()`** is ownership-based: if the caller is the current owner, stop and clear the owner; otherwise only stop when `open_count` reaches zero.  This avoids fork/dup accidents where closing one fd would silence another process's playback.
- **`stm32_sound_stop_pcm()` is idempotent** — calling it on an already-stopped pipeline is a no-op.

## Start / stop sequence

**start** (`stm32_sound_play_pcm`):

1. Enter critical section.
2. `STM32_TIM_DISABLE(g_tim6)` — stop timer.
3. `stm32_dmastop()` + `stm32_dmasetup()` + `stm32_dmastart()` — reconfigure DMA1 S5 (circular, 16-bit, M2P).
4. DAC1 CR: `DMAEN1 | TSEL1_TIM6 | TEN1 | EN1` (direct register).
5. `STM32_TIM_SETPERIOD()` + `STM32_TIM_ENABLE()` — set ARR + UG + CEN.
6. Leave critical section.
7. **2 ms settling delay** via `nxsig_usleep` so the DAC can reach its midpoint before the amplifier is unmuted.
8. AMP_EN = HIGH.

**stop** (`stm32_sound_stop_pcm`):

1. AMP_EN = LOW — mute first to avoid pops.
2. Enter critical section.
3. `STM32_TIM_DISABLE(g_tim6)` — stop timer.
4. `stm32_dmastop()` — stop DMA.
5. DAC1 `CR = 0` (direct register).
6. Leave critical section.

## `/dev/tone0` tune string format

pybricks-compatible syntax:

```
[T<bpm>] [O<octave>] [L<fraction>] <note>...

<note>    := (<letter>[#|b][<octave>]) | R
             [ / <fraction> ] [.] [_]

<letter>  := C | D | E | F | G | A | B
<fraction>:= 1 | 2 | 4 | 8 | 16 | 32
```

- Leading directives override defaults: `T<bpm>` (10..400, default 120), `O<octave>` (2..8), `L<fraction>`.
- `R` is a rest.
- `.` marks a dotted note (×1.5 duration).
- `_` marks legato (no release gap after the note).
- Tokens can be separated by spaces, commas, or newlines.

### Timing

- Whole-note duration = `240_000_000 / bpm` µs (= 4 × 60 s / bpm).
- Audible portion = `whole_us / fraction` × (1.5 if dotted) − release_us.
- release_us = 1/8 of the audible portion (0 for legato).
- `nxsig_usleep` runs in 20 ms slices and polls `stop_flag` at the start of each slice.

### Examples

```
nsh> echo "C4/4 D4/4 E4/4 F4/4 G4/2" > /dev/tone0
nsh> echo "T240 L8 C4 D4 E4 F4 G4 A4 B4 C5" > /dev/tone0
nsh> echo "D#5/8. F#5/16 A5/4_" > /dev/tone0   # dotted + legato
nsh> echo "C4/4 R/4 G4/4" > /dev/tone0          # rest
```

### Interruption

- `Ctrl-C` (SIGINT) cancels `nxsig_usleep`; `write()` returns `-EINTR`.
- `ioctl(fd, TONEIOC_STOP, 0)` stops immediately.
- A concurrent write to `/dev/pcm0` preempts tone playback.

## `/dev/pcm0` raw PCM ABI (v1)

Single-call interface semantically equivalent to pybricks `pbdrv_sound_start(data, length, sample_rate)`: one `write()` carries the header and all samples together.

### Header

```c
#define PCM_WRITE_MAGIC    0x304D4350u  /* "PCM0" little-endian */
#define PCM_WRITE_VERSION  0x0001u

struct pcm_write_hdr_s
{
    uint32_t magic;         /* PCM_WRITE_MAGIC */
    uint16_t version;       /* ABI version, currently 1 */
    uint16_t hdr_size;      /* sizeof(struct) or larger (future extensions) */
    uint32_t flags;         /* 0 only for now (reserved) */
    uint32_t sample_rate;   /* Hz, [1000, 100000] */
    uint32_t sample_count;  /* number of uint16_t samples after the header */
};
```

| offset | size | field |
|-------:|-----:|-------|
| 0  | 4 | `magic` |
| 4  | 2 | `version` |
| 6  | 2 | `hdr_size` |
| 8  | 4 | `flags` |
| 12 | 4 | `sample_rate` |
| 16 | 4 | `sample_count` |
| **total** | **20** | |

### write() validation order

1. `nbytes >= sizeof(struct pcm_write_hdr_s)` → otherwise `-EINVAL`.
2. `version <= PCM_WRITE_VERSION` (forward compatibility) → `-EINVAL`.
3. `magic == PCM_WRITE_MAGIC` → `-EINVAL`.
4. `hdr_size >= sizeof(struct)` and `hdr_size <= nbytes` → `-EINVAL`.
5. `1000 <= sample_rate <= 100000` → `-EINVAL`.
6. `sample_count <= UINT32_MAX / 2` → `-EOVERFLOW`.
7. `payload_bytes == nbytes - hdr_size` → `-EINVAL`.
8. `payload_bytes <= CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES * 2` → `-E2BIG`.

Validation, `memcpy` into the kernel BSS buffer, `stm32_sound_play_pcm`, and the `owner = filep` update all run atomically under `g_sound.lock`.  Under `CONFIG_BUILD_PROTECTED` the entire user buffer is range-checked with `board_user_in_ok(ubuf, nbytes)` (from `boards/spike-prime-hub/src/board_usercheck.h`) before any dereference, then the header is copied into a kernel-local struct (closes the TOCTOU window between the magic/size/count checks and the payload copy), then the payload is `memcpy`'d into the kernel BSS buffer `g_pcm_buf`.  Under `CONFIG_BUILD_FLAT` the range check degenerates to `true`.

### Cyclic playback

`/dev/pcm0` is designed for **cyclic single-period playback** — the uploaded buffer represents one waveform period that loops until stop, exactly like pybricks.  Streaming arbitrary-length WAV content is not supported.  Stop via `ioctl(TONEIOC_STOP, 0)` or `close()`.

### Buffer size

`CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES` (Kconfig, default 1024 samples = 2 KB) is tunable in the range [256, 8192].

## `TONEIOC_*` ioctls

Both `/dev/tone0` and `/dev/pcm0` share the same ioctl space defined in `arch/board/board_sound.h` via `_BOARDIOC()`.  The codes do not collide with upstream `AUDIOIOC_*` or `SNDIOC_*`.

| ioctl | arg | Description |
|-------|-----|-------------|
| `TONEIOC_VOLUME_SET` | `int` (0..100) | Set volume; applies to the next note. |
| `TONEIOC_VOLUME_GET` | `int *`        | Read the current volume. |
| `TONEIOC_STOP`       | 0              | Stop playback immediately. |

Volume uses a pybricks-compatible exponential curve for amplitude scaling (0 = silent, 100 = full scale 0x7fff).

## `apps/sound` NSH utility

Enabled with `CONFIG_APP_SOUND=y`.

| Command | Description |
|---------|-------------|
| `sound beep [freq_hz] [dur_ms]` | Build a square wave, write the blob to `/dev/pcm0`, `usleep`, then `TONEIOC_STOP` (defaults 500 Hz / 200 ms). |
| `sound tone <freq> <dur_ms>` | Alias for `beep`. |
| `sound notes <tune>` | Write the tune string to `/dev/tone0`. |
| `sound volume [0-100]` | `TONEIOC_VOLUME_SET` / `VOLUME_GET`. |
| `sound off` | `TONEIOC_STOP`. |
| `sound selftest` | Play 500 Hz for 200 ms through `/dev/pcm0` (replaces the earlier bringup-time POST so init context never `usleep`s). |

`sound beep` is the canonical diagnostic path for `/dev/pcm0` since the raw PCM device cannot be driven from `echo` (it expects a binary header).

## Feature comparison with pybricks

The low-level layer of this driver is a 1:1 equivalent of pybricks `pbdrv_sound_start(data, length, sample_rate)`.  pybricks' high-level API (`Speaker.beep` / `Speaker.play_notes` / `Speaker.volume`) lives in MicroPython; this project re-implements the equivalent functionality in the kernel and an NSH builtin.

### Shared behavior

- **Audio chain**: DAC1 CH1 (PA4) → external amplifier (PC10) → speaker.  Identical to pybricks.
- **Cyclic DMA playback**: one period of the waveform loops until stop (buffer = single period, stop is explicit).
- **Waveform**: 128–256 sample square wave, centered on the midpoint.
- **Volume curve**: pybricks' `(10^(v/100) − 1) / 9 * INT16_MAX` precomputed in a 10 %-step lookup table (same exponential curve).
- **TIM6 as the sample trigger**: same basic timer as pybricks; no general-purpose timer is consumed.
- **Volume changes apply on the next note**: the currently playing waveform keeps its amplitude.
- **No file playback**: neither pybricks nor this driver supports streaming long WAV/RSF content.

### Features this driver has that pybricks does not

| Feature | This driver | pybricks |
|---------|-------------|----------|
| Tune playback from a shell | `echo "C4/4" > /dev/tone0` drives the speaker directly | Requires a MicroPython runtime |
| In-kernel tune parser | `stm32_tone.c` parses `"C4/4 D#5/8. R/4 G4/4_"` inside the kernel | Parsed in Python on the MicroPython side |
| Single-call binary PCM ABI | `struct pcm_write_hdr_s` (version/hdr_size/flags) delivered in one `write()` | Direct C function call (internal API only, no ABI) |
| Forward-compatible version field | `version` / `hdr_size` allow new fields to be appended later | None (internal C API only) |
| Multi-process ownership tracking | `filep`-based owner + `open_count` gate takeover and cleanup | Single MicroPython runtime, no multi-process concept |
| Mid-playback interruption | 20 ms `nxsig_usleep` slices + `atomic_bool stop_flag` driven by `TONEIOC_STOP` / `Ctrl-C` / a concurrent `pcm0` write | `beep()` blocks for the full duration (no mid-note stop) |
| Startup pop suppression | DAC enabled → 2 ms settling delay → AMP_EN HIGH | `HAL_DAC_Start_DMA` followed immediately by AMP HIGH (no delay) |
| Automatic `close()` stop | Stops only when owner matches; safe across fork/dup | Not applicable (single-process MicroPython) |
| `TONEIOC_*` ioctl space | Board-local `_BOARDIOC()` codes, no clash with upstream ioctl spaces | N/A |
| NSH diagnostic commands | `sound beep`, `sound notes`, `sound volume`, `sound off`, `sound selftest` | Needs a pybricks-micropython script |

### Features pybricks has that this driver does not

| Feature | pybricks | This driver |
|---------|----------|-------------|
| MicroPython bindings | `hub.speaker.beep(500, 100)` and friends | No Python runtime on NuttX |
| Non-blocking beep completion | `pb_type_Speaker_beep_test_completion` integrates with the cooperative scheduler | `write()` + `usleep` is synchronous (Ctrl-C and ioctl still interrupt) |
| Python `play_notes()` generator | Runs as a `notes_generator` and can continue alongside other tasks | `/dev/tone0` write is a single blocking call |
| Always-on sample timer | `pbdrv_sound_init` starts TIM6 once and leaves it running | `TIM6.CEN` is toggled 0→1 for every playback |
| DMA zero-copy buffer | The caller's buffer is handed straight to the DMA engine | `/dev/pcm0` copies the user payload into a kernel BSS buffer (under BUILD_PROTECTED the user range is validated by `board_user_in_ok` before the copy) |

### Keeping compatibility in mind

- **The low-level API is 1:1 with pybricks `pbdrv_sound_start`**, so future interoperability (running pybricks on top of this driver, or porting pybricks' tune generator on top of this one) stays straightforward.
- **The waveform format and DAC register layout are identical**, so sending a pybricks sample buffer (`INT16_MAX` center) through `/dev/pcm0` only needs a `0x8000` offset.
- **`TONEIOC_*` lives in a board-local ioctl space**, which leaves room for a future pybricks-compatible higher-level API in a separate space (e.g. `SNDIOC_*`) without collisions.

## Known limitations

- **PCM buffer plays one period on loop**.  Streaming arbitrary-length WAV content is not supported.
- **Volume changes apply on the next note**.  The currently playing waveform keeps the old amplitude (same as pybricks).
- **Interruption granularity is ~20 ms** — the slice size of the `stop_flag` polling loop.

## Design notes

### First attempt: NuttX `tone_register()` over a oneshot timer

The initial design reused NuttX's stock `drivers/audio/tone.c` (`tone_register()`) by presenting DAC1 as a PWM lower-half and delegating note-length timing to a `oneshot_lowerhalf_s` built on `stm32_oneshot`.

That approach was abandoned after discovering that **no oneshot timer is available on the F413**:

- `stm32_oneshot` drives one-pulse mode by writing `CR1 = CEN | ARPE | OPM`, but the STM32F413 `TIM10/11/13/14` do not support the `CR1.OPM` bit at all (see RM0430 §19.5.1).  The code compiles but never fires.
- Every OPM-capable timer (`TIM1/2/3/4/5/8/9/12`) is already reserved for pybricks-equivalent functionality:
    - `TIM1/3/4` — motor PWM (three pairs)
    - `TIM2` — ADC DMA trigger (1 kHz)
    - `TIM5` — battery charger ISET PWM
    - `TIM8` — Bluetooth 32 kHz clock
    - `TIM9` — NuttX `CONFIG_STM32_TICKLESS_TIMER`
    - `TIM12` — TLC5955 GSCLK (9.6 MHz)
- That forced note-length timing **off of any additional oneshot timer**.

Related issues: [#27](https://github.com/owhinata/spike-nx/issues/27), [#28](https://github.com/owhinata/spike-nx/issues/28), [#30](https://github.com/owhinata/spike-nx/issues/30).

### Converging on the current design

Following pybricks' approach, the kernel layer was trimmed down to "take a waveform buffer plus a sample rate and stream it via circular DMA", and note-length timing was moved to synchronous blocking inside `nxsig_usleep`.  That eliminated the oneshot dependency entirely and let the whole sound path run off `TIM6` alone.

Several Codex review rounds then settled the remaining details:

1. **Two parallel char devices** — `/dev/tone0` for tune strings and `/dev/pcm0` for pybricks-compatible raw PCM, so NSH `echo` diagnostics and a single-call binary ABI are both available.
2. **Single-call PCM ABI** (`struct pcm_write_hdr_s`) — removes the order dependency of a hypothetical `SETRATE` + `write()` pair and leaves room for extension via `version` / `hdr_size`.
3. **Ownership-based `close()`** — the current `filep` is tracked as the owner; playback only stops when the same fd closes, so fork/dup cannot silence another process by accident.
4. **Per-note mutex release + `atomic_bool stop_flag`** — `tone_write` drops the lock between slices so `TONEIOC_STOP`, `Ctrl-C`, and a concurrent `pcm0` write all interrupt playback within 20 ms.
5. **Restricted `stop_flag` clearing** — only `tone_write` clears it at entry; `pcm_write` does not, so it cannot accidentally cancel a pending tone interrupt.
6. **Start order: `TIM6 + DAC enable` → 2 ms settling delay → `AMP_EN HIGH`** — lets the DAC output reach its midpoint before the amplifier is unmuted, suppressing startup pops.

## Source references

- [`boards/spike-prime-hub/src/stm32_sound.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_sound.c) — low-level PCM playback
- [`boards/spike-prime-hub/src/stm32_tone.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_tone.c) — `/dev/tone0`
- [`boards/spike-prime-hub/src/stm32_pcm.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_pcm.c) — `/dev/pcm0`
- [`boards/spike-prime-hub/include/board_sound.h`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/include/board_sound.h) — public ABI
- [`apps/sound/sound_main.c`](https://github.com/owhinata/spike-nx/blob/5f439034e32c2bc0e18e9c16f1c728739c0b47e1/apps/sound/sound_main.c) — NSH builtin
- pybricks reference: `pybricks/lib/pbio/drv/sound/sound_stm32_hal_dac.c`
