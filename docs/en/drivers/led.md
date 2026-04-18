# LED Driver (TLC5955)

All LEDs on the SPIKE Prime Hub are controlled by a TI TLC5955 (48-channel SPI PWM LED driver). There are no GPIO-direct LEDs.

## Hardware Configuration

| Item | Setting |
|------|---------|
| LED driver IC | TI TLC5955 (48ch, 16-bit PWM) |
| SPI | SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5), 24 MHz |
| LAT (latch) | PA15 — GPIO output, latches on HIGH→LOW edge |
| GSCLK | TIM12 CH2 = PB15 (AF9), 9.6 MHz PWM |
| Shift register | 769 bits (97 bytes) |

### Power

LEDs are powered from the battery rail. **LEDs do not light up with USB power alone.**

### Control Latch Settings

Same settings as pybricks:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Dot Correction (DC) | 127 | 100% |
| Max Current (MC) | 3.2 mA | Minimum setting |
| Global Brightness (BC) | 127 | 100% |
| Auto Display Repeat | ON | |
| Display Timing Reset | OFF | |
| Auto Refresh | OFF | |
| ES-PWM | ON | Enhanced Spectrum PWM |
| LSD Detection | 90% | |

The control latch is sent twice during initialization (TLC5955 hardware requirement for max current to take effect).

## LED Channel Mapping

Channels are mapped to GS registers in reverse order (CH0→GSB15, CH47→GSR0).

### Status LEDs (RGB)

| LED | R | G | B |
|-----|---|---|---|
| Status Top (center button upper) | CH5 | CH4 | CH3 |
| Status Bottom (center button lower) | CH8 | CH7 | CH6 |
| Battery | CH2 | CH1 | CH0 |
| Bluetooth | CH20 | CH19 | CH18 |

### 5x5 LED Matrix

| Row | Col0 | Col1 | Col2 | Col3 | Col4 |
|-----|------|------|------|------|------|
| 0 | CH38 | CH36 | CH41 | CH46 | CH33 |
| 1 | CH37 | CH28 | CH39 | CH47 | CH21 |
| 2 | CH24 | CH29 | CH31 | CH45 | CH23 |
| 3 | CH26 | CH27 | CH32 | CH34 | CH22 |
| 4 | CH25 | CH40 | CH30 | CH35 | CH9 |

## Userspace API (`/dev/rgbled0`)

Under `CONFIG_BUILD_PROTECTED=y`, user blobs cannot resolve kernel symbols directly. TLC5955 is accessed via the char device `/dev/rgbled0` using ioctl (Issue #39).

```c
#include <fcntl.h>
#include <sys/ioctl.h>
#include <arch/board/board_rgbled.h>

int fd = open("/dev/rgbled0", O_RDWR);

/* Set one channel's duty */
struct rgbled_duty_s d = { .channel = TLC5955_CH_STATUS_TOP_G,
                           .value   = 0xffff };
ioctl(fd, RGBLEDIOC_SETDUTY, (unsigned long)&d);

/* Set all 48 channels at once (arg is the uint16_t duty value) */
ioctl(fd, RGBLEDIOC_SETALL, 0);         /* all off */

/* Force immediate SPI sync (bypass HPWORK deferred batch) */
ioctl(fd, RGBLEDIOC_UPDATE, 0);
```

| ioctl | arg type | Description |
|-------|----------|-------------|
| `RGBLEDIOC_SETDUTY` | `struct rgbled_duty_s *` | Set `channel` (0..47) to `value` (0..0xffff) |
| `RGBLEDIOC_SETALL` | `uint16_t` (arg itself) | Apply the same duty to all 48 channels |
| `RGBLEDIOC_UPDATE` | 0 | Call `tlc5955_update_sync()` for immediate flush |

`board_rgbled.h` also exports `TLC5955_NUM_CHANNELS` and the `TLC5955_CH_*` constants (previously defined in `spike_prime_hub.h`).

## Kernel-Internal API

`stm32_bringup` and other in-kernel code may call the TLC5955 functions directly:

```c
#include "spike_prime_hub.h"

/* Initialize (called automatically in stm32_bringup) */
int tlc5955_initialize(void);

/* Set PWM duty for a channel (0=OFF, 0xFFFF=full brightness).
 * Automatically schedules SPI transfer on HPWORK queue.
 * Multiple set_duty calls are batched into one SPI transfer.
 */
void tlc5955_set_duty(uint8_t ch, uint16_t value);

/* Immediate update: for init/shutdown use (when HPWORK not running) */
int tlc5955_update_sync(void);

/* Register /dev/rgbled0 (called automatically in stm32_bringup) */
int stm32_rgbled_register(void);
```

## defconfig

```
CONFIG_STM32_SPI1=y
CONFIG_STM32_SPI1_DMA=y
CONFIG_STM32_DMA2=y
```

## Update Method

`tlc5955_set_duty()` only writes data to the buffer and sets the `changed` flag without performing an SPI transfer. Calling `tlc5955_update()` schedules a deferred transfer on the HPWORK queue, batching multiple `set_duty` calls into a single SPI transfer.

Use `tlc5955_update_sync()` when immediate update is needed (during initialization or shutdown).

## Comparison with pybricks

| Item | pybricks | NuttX |
|------|----------|-------|
| SPI transfer | HAL SPI + DMA (async) | NuttX SPI driver + DMA (sync) |
| GSCLK | TIM12 CH2 (HAL PWM) | TIM12 CH2 (NuttX TIM API) |
| LAT | HAL GPIO | stm32_gpiowrite() |
| Update method | Contiki protothread + changed flag | HPWORK queue + changed flag |
| Control latch | Same parameters | Same parameters |

The SPI1 DMA channel mapping (`DMACHAN_SPI1_RX/TX`) was not defined in NuttX for STM32F413, so it is defined in `board.h`:

- RX: DMA2 Stream2 Channel 3 (`DMAMAP_SPI1_RX_2`)
- TX: DMA2 Stream3 Channel 3 (`DMAMAP_SPI1_TX_1`)

## Test App

The `led` NSH command tests all LED features:

```
led green     - Status LED green (boot default)
led status    - Cycle status LED: R → G → B → white → off
led battery   - Cycle battery LED colors
led bluetooth - Cycle bluetooth LED colors
led rainbow   - Rainbow animation (HSV hue sweep)
led blink     - Blink green
led breathe   - Breathing effect (fade in/out)
led matrix    - 5x5 matrix: all on → scan → digits 0-9
led all       - Run all tests
led off       - All LEDs off
```

## Source Files

- `boards/spike-prime-hub/src/stm32_tlc5955.c` — Driver implementation (SPI + HPWORK)
- `boards/spike-prime-hub/src/stm32_rgbled.c` — `/dev/rgbled0` char driver (thin ioctl wrapper)
- `boards/spike-prime-hub/include/board_rgbled.h` — Channel constants + ioctl ABI (shared by user/kernel)
- `apps/led/led_main.c` — LED test app (ioctl-based)
