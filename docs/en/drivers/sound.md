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

Upstream NuttX Kconfig for the STM32F413 does **not** `select STM32_HAVE_DAC1`, and `stm32f413xx_pinmap.h` has no DAC1 OUT1 macros.  The board layer therefore:

- enables the RCC `DAC1EN` / `TIM6EN` / `DMA1EN` bits directly, and
- defines its own `GPIO_DAC1_OUT1_F413` (`GPIO_ANALOG | PORTA | PIN4`).

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
    DAC1 / DMA1 S5 / TIM6 (direct register access)
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
2. TIM6 `CEN = 0`.
3. Disable DMA1 Stream 5, wait for `EN = 0`, clear HIFCR flags.
4. Reconfigure DMA1 S5 (`PAR = DHR12L1`, `M0AR = data`, `NDTR = length`, `SCR = CHSEL7 | PRIHI | MSIZE16 | PSIZE16 | MINC | CIRC | DIR_M2P | EN`).
5. DAC1 CR: `DMAEN1 | TSEL1_TIM6 | TEN1 | EN1`.
6. TIM6 `PSC = 0`, `ARR = 96000000 / sample_rate - 1`, `EGR = UG`, `CEN = 1`.
7. Leave critical section.
8. **2 ms settling delay** via `nxsig_usleep` so the DAC can reach its midpoint before the amplifier is unmuted.
9. AMP_EN = HIGH.

**stop** (`stm32_sound_stop_pcm`):

1. AMP_EN = LOW — mute first to avoid pops.
2. Enter critical section.
3. TIM6 `CEN = 0`.
4. Disable DMA1 S5 and clear flags.
5. DAC1 `CR = 0`.
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

Validation, `memcpy` into the kernel BSS buffer, `stm32_sound_play_pcm`, and the `owner = filep` update all run atomically under `g_sound.lock`.  User pointers are copied with `memcpy` on FLAT builds; `CONFIG_BUILD_PROTECTED` would need a `copyin()` helper.

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

## Known limitations

- **FLAT build only**.  `CONFIG_BUILD_PROTECTED` would need a `copyin()` helper for user pointers.
- **PCM buffer plays one period on loop**.  Streaming arbitrary-length WAV content is not supported.
- **Volume changes apply on the next note**.  The currently playing waveform keeps the old amplitude (same as pybricks).
- **Interruption granularity is ~20 ms** — the slice size of the `stop_flag` polling loop.

## Design notes

The original v3 design layered NuttX `tone_register()` on top of `stm32_oneshot` + TIM10, but it failed once we discovered that STM32F413 `TIM10/11/13/14` do not support one-pulse mode (`CR1.OPM`); see [issue #27](https://github.com/owhinata/spike-nx/issues/27), [#28](https://github.com/owhinata/spike-nx/issues/28), and [#30](https://github.com/owhinata/spike-nx/issues/30).  Every OPM-capable timer (TIM1/2–5/8/9/12) is already reserved for pybricks-equivalent functionality.

Starting from v4 Option B (in-kernel tune parser + synchronous `nxsig_usleep`), the design converged through four Codex review rounds on: dual char devices, a single-call PCM ABI, ownership-based `close()`, per-note mutex release, and an atomic `stop_flag`.

## Source references

- [`boards/spike-prime-hub/src/stm32_sound.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_sound.c) — low-level PCM playback
- [`boards/spike-prime-hub/src/stm32_tone.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_tone.c) — `/dev/tone0`
- [`boards/spike-prime-hub/src/stm32_pcm.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_pcm.c) — `/dev/pcm0`
- [`boards/spike-prime-hub/include/board_sound.h`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/include/board_sound.h) — public ABI
- [`apps/sound/sound_main.c`](https://github.com/owhinata/spike-nx/blob/5f439034e32c2bc0e18e9c16f1c728739c0b47e1/apps/sound/sound_main.c) — NSH builtin
- pybricks reference: `pybricks/lib/pbio/drv/sound/sound_stm32_hal_dac.c`
