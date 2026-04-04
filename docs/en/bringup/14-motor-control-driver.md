# Motor Control Driver

## 1. H-Bridge Hardware Configuration

Each I/O port has an H-bridge motor driver controlled by 2 PWM channels (M1, M2).

### Port → Timer Channel Mapping

| Port | M1 Pin | M1 Timer/Ch | M2 Pin | M2 Timer/Ch | AF |
|---|---|---|---|---|---|
| A | PE9 | TIM1 CH1 | PE11 | TIM1 CH2 | AF1 |
| B | PE13 | TIM1 CH3 | PE14 | TIM1 CH4 | AF1 |
| C | PB6 | TIM4 CH1 | PB7 | TIM4 CH2 | AF2 |
| D | PB8 | TIM4 CH3 | PB9 | TIM4 CH4 | AF2 |
| E | PC6 | TIM3 CH1 | PC7 | TIM3 CH2 | AF2 |
| F | PC8 | TIM3 CH3 | PB1 | TIM3 CH4 | AF2 |

### Timer Configuration

| Timer | Ports | Clock | Prescaler | Period | PWM Freq |
|---|---|---|---|---|---|
| TIM1 | A, B | 96 MHz (APB2) | 8 | 1000 | 12 kHz |
| TIM3 | E, F | 96 MHz (APB1×2) | 8 | 1000 | 12 kHz |
| TIM4 | C, D | 96 MHz (APB1×2) | 8 | 1000 | 12 kHz |

**Note**: APB1 timers clock is doubled when APB1 prescaler > 1 (48 MHz × 2 = 96 MHz).
All channels configured with INVERT (inverted PWM). Pybricks uses 12 kHz (LEGO official uses 1.2 kHz).

---

## 2. H-Bridge Control Patterns

Four states via GPIO/AF mode switching:

| State | M1 (Pin1) | M2 (Pin2) | Effect |
|---|---|---|---|
| **Coast** | GPIO LOW | GPIO LOW | Both LOW, motor free-spins |
| **Brake** | GPIO HIGH | GPIO HIGH | Both HIGH, short-circuit braking |
| **Forward** | PWM (AF) + duty | GPIO HIGH | M1 PWMs, M2 held HIGH |
| **Reverse** | GPIO HIGH | PWM (AF) + duty | M1 held HIGH, M2 PWMs |

### Duty Cycle

- Range: [-1000, +1000] (±100%)
- Positive → Forward, Negative → Reverse, Zero → Brake
- Period = 1000, so duty value writes directly to CCR register
- Inverted PWM: duty=0 → always HIGH, duty=1000 → always LOW

### Voltage Compensation

Pybricks compensates for battery voltage variation:
```
duty = target_voltage × MAX_DUTY / battery_voltage
```

---

## 3. Motor Encoder (via LUMP UART)

### Data Flow

```
Motor built-in encoder → UART TX → Hub UART RX (LUMP)
  → legodev_pup_uart → pbio_tacho → pbio_servo (PID control)
```

### Encoder Modes

**Absolute encoder motors** (SPIKE M/L/S, Technic Angular):
- Mode 3 (APOS): int16, tenths of degrees
- Mode 4 (CALIB): absolute position + calibration data
- Software tracks full rotations by detecting 360° boundary crossing

**Relative encoder motors** (BOOST Interactive):
- Mode 2 (POS): int32, cumulative degrees

### PID Control Loop

Pybricks PID control:

| Parameter | Value |
|---|---|
| Loop rate | 5 ms (200 Hz) |
| Position control | PID with anti-windup |
| Speed control | PI (P on speed error + D on position) |
| Speed estimation | 100ms window (20 samples) differentiator |
| State estimation | Luenberger observer (angle, speed, current) |
| Stall detection | Observer feedback voltage ratio + min stall time |

---

## 4. NuttX Driver Design

### NuttX PWM Driver

NuttX provides upper-half/lower-half PWM architecture:
- `/dev/pwmN` character device
- `ioctl(PWMIOC_SETCHARACTERISTICS)` for control
- `CONFIG_PWM_MULTICHAN` for multi-channel support
- STM32 TIM1 advanced timer (complementary output, dead-time) supported

**NuttX has NO motor control framework.**

### Recommended Architecture

```
/dev/legomotor[N]  (custom motor driver)
  ├─ H-bridge control (GPIO/AF mode switch + PWM duty)
  ├─ Encoder reading (via LUMP UART)
  └─ PID control loop (optional, can be in app layer)
```

### ioctl Interface

```c
#define LEGOMOTOR_SET_DUTY      _LEGOMOTORIOC(0)  // int16: -1000 to +1000
#define LEGOMOTOR_COAST         _LEGOMOTORIOC(1)  // Coast (free-spin)
#define LEGOMOTOR_BRAKE         _LEGOMOTORIOC(2)  // Brake (short-circuit)
#define LEGOMOTOR_GET_POSITION  _LEGOMOTORIOC(3)  // int32: degrees
#define LEGOMOTOR_GET_SPEED     _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS   _LEGOMOTORIOC(5)  // int16: absolute position
#define LEGOMOTOR_RESET_POS     _LEGOMOTORIOC(6)  // Reset position counter
```

### GPIO/AF Mode Switching Implementation

Core pybricks pattern: **dynamically switch pins between GPIO mode (Digital HIGH/LOW) and AF mode (Timer PWM output)**.

NuttX implementation:
```c
// Forward: M1=PWM, M2=HIGH
stm32_configgpio(GPIO_TIM1_CH1OUT);                // M1 → AF1 (PWM)
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_SET |    // M2 → GPIO HIGH
                 GPIO_PORTE | GPIO_PIN11);

// Coast: M1=LOW, M2=LOW
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M1 → GPIO LOW
                 GPIO_PORTE | GPIO_PIN9);
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M2 → GPIO LOW
                 GPIO_PORTE | GPIO_PIN11);
```

### Alternative: PWM-Only Control

Avoid GPIO/AF switching by leveraging inverted PWM:
- Inverted PWM with duty=0 → always HIGH (equivalent to GPIO HIGH)
- Inverted PWM with duty=period → always LOW (equivalent to GPIO LOW)
- Forward: M1=duty_N, M2=duty_0
- Coast: M1=duty_period, M2=duty_period

This keeps PWM channels always active, better compatibility with NuttX standard PWM driver.

---

## 5. PID Control Implementation Strategy

### Initial Phase

- Simple duty control only (`LEGOMOTOR_SET_DUTY`)
- Encoder position reading (`LEGOMOTOR_GET_POSITION`)
- PID implemented in application layer

### Future

- Kernel-space PID loop (5ms work queue)
- Luenberger observer based on pybricks reference
- High-level ioctls: `LEGOMOTOR_RUN_TO_POSITION`, `LEGOMOTOR_RUN_AT_SPEED`
