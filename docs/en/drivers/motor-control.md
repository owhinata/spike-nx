# Motor Control Driver

Implementation status (Issue #80, 2026-05): the H-bridge PWM HAL lives in
`boards/spike-prime-hub/src/stm32_legoport_pwm.c` and is exposed through
the `/dev/legoport[N]` chardev (`CONFIG_LEGO_PORT=y`) via the
`LEGOPORT_PWM_SET_DUTY / COAST / BRAKE / GET_STATUS` ioctls.  The
userspace CLI is `legoport pwm <port> {set|coast|brake|status}`
(`apps/legoport/`).  Full ABI lives in
`boards/spike-prime-hub/include/board_legoport.h`.

Phase B (auto-driving the SUPPLY pin on LUMP SYNC for sensors that
advertise NEEDS_SUPPLY_PIN1 / PIN2 — needed to power Color / Ultrasonic
sensor LEDs) lands in a follow-up commit.

## 1. H-Bridge Hardware Configuration

Each I/O port has an H-bridge motor driver with 2 PWM channels (M1, M2) for direction control.

### Port to Timer Channel Mapping

| Port | M1 Pin | M1 Timer/Ch | M2 Pin | M2 Timer/Ch | AF |
|---|---|---|---|---|---|
| A | PE9 | TIM1 CH1 | PE11 | TIM1 CH2 | AF1 |
| B | PE13 | TIM1 CH3 | PE14 | TIM1 CH4 | AF1 |
| C | PB6 | TIM4 CH1 | PB7 | TIM4 CH2 | AF2 |
| D | PB8 | TIM4 CH3 | PB9 | TIM4 CH4 | AF2 |
| E | PC6 | TIM3 CH1 | PC7 | TIM3 CH2 | AF2 |
| F | PC8 | TIM3 CH3 | PB1 | TIM3 CH4 | AF2 |

### Timer Configuration

| Timer | Ports | Clock | Prescaler | Period | PWM Frequency |
|---|---|---|---|---|---|
| TIM1 | A, B | 96 MHz (APB2) | 8 | 1000 | 12 kHz |
| TIM3 | E, F | 96 MHz (APB1x2) | 8 | 1000 | 12 kHz |
| TIM4 | C, D | 96 MHz (APB1x2) | 8 | 1000 | 12 kHz |

APB1 timers have their clock doubled when APB1 prescaler > 1 (48 MHz x 2 = 96 MHz). All channels use inverted PWM. pybricks uses 12 kHz (LEGO official uses 1.2 kHz).

## 2. H-Bridge Control Patterns

Four states are achieved by switching pins between GPIO/AF modes:

| State | M1 (Pin1) | M2 (Pin2) | Effect |
|---|---|---|---|
| **Coast** | GPIO LOW | GPIO LOW | Both sides LOW, motor free-spins |
| **Brake** | GPIO HIGH | GPIO HIGH | Both sides HIGH, short-circuit braking |
| **Forward** | PWM (AF) + duty | GPIO HIGH | M1 is PWM, M2 is HIGH |
| **Reverse** | GPIO HIGH | PWM (AF) + duty | M1 is HIGH, M2 is PWM |

### Duty Cycle

- Range: [-1000, +1000] (+-100%)
- Positive -> Forward, Negative -> Reverse, 0 -> Brake
- Period = 1000 so the duty value is written directly to the CCR register
- Due to inverted PWM, duty=0 -> always HIGH, duty=1000 -> always LOW

### Voltage Compensation

pybricks compensates for battery voltage fluctuation:
```
duty = target_voltage * MAX_DUTY / battery_voltage
```

### GPIO/AF Mode Switching in NuttX

```c
// Forward: M1=PWM, M2=HIGH
stm32_configgpio(GPIO_TIM1_CH1OUT);                // M1 -> AF1 (PWM)
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_SET |    // M2 -> GPIO HIGH
                 GPIO_PORTE | GPIO_PIN11);

// Coast: M1=LOW, M2=LOW
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M1 -> GPIO LOW
                 GPIO_PORTE | GPIO_PIN9);
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M2 -> GPIO LOW
                 GPIO_PORTE | GPIO_PIN11);
```

### Alternative Approach: PWM-Only Control

A method to avoid GPIO/AF switching:
- With inverted PWM, duty=0 -> always HIGH, duty=period -> always LOW
- Forward: M1=duty_N, M2=duty_0 (equivalent to GPIO HIGH)
- Coast: M1=duty_period, M2=duty_period (equivalent to GPIO LOW)

This keeps PWM channels always enabled and has better compatibility with the NuttX standard PWM driver.

## 3. Motor Encoder (via LUMP UART)

### Data Flow

```
Motor built-in encoder -> UART TX -> Hub UART RX (LUMP)
  -> legodev_pup_uart -> pbio_tacho -> pbio_servo (PID control)
```

### Encoder Modes

**Absolute encoder motors** (SPIKE M/L/S Motor, Technic Angular Motor):
- Mode 3 (APOS): int16, 1/10 degree units
- Mode 4 (CALIB): absolute position + calibration data
- Software detects 360-degree crossings and tracks rotations

**Relative encoder motors** (BOOST Interactive Motor):
- Mode 2 (POS): int32, cumulative degrees

## 4. PID Control Loop

pybricks PID control parameters:

| Parameter | Value |
|---|---|
| Loop rate | 5 ms (200 Hz) |
| Position control | PID (proportional + integral + derivative) + anti-windup |
| Speed control | PI (P on speed error + D on position error) |
| Speed estimation | 100ms window (20 samples) differentiator |
| State estimation | Luenberger observer (angle, speed, current) |
| Stall detection | Observer feedback voltage ratio + minimum stall time |

## 5. NuttX Driver Design

### Device Node

```
/dev/legomotor[N]  (Custom motor driver)
  |- H-bridge control (GPIO/AF mode switching + PWM duty setting)
  |- Encoder reading (via LUMP UART)
  +- PID control loop (optional, can also be in application layer)
```

NuttX provides a PWM driver (`/dev/pwmN`) but does not have a motor control framework.

### ioctl Interface

```c
#define LEGOMOTOR_SET_DUTY      _LEGOMOTORIOC(0)  // int16: -1000 to +1000
#define LEGOMOTOR_COAST         _LEGOMOTORIOC(1)  // Coast
#define LEGOMOTOR_BRAKE         _LEGOMOTORIOC(2)  // Brake
#define LEGOMOTOR_GET_POSITION  _LEGOMOTORIOC(3)  // int32: degrees
#define LEGOMOTOR_GET_SPEED     _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS   _LEGOMOTORIOC(5)  // int16: absolute position
#define LEGOMOTOR_RESET_POS     _LEGOMOTORIOC(6)  // Reset position
```

## 6. PID Control Implementation Strategy

### Initial Stage

- Simple duty control only (`LEGOMOTOR_SET_DUTY`)
- Encoder value reading (`LEGOMOTOR_GET_POSITION`)
- PID implemented in the application layer

### Future

- PID loop in kernel space (5ms work queue)
- Implementation referencing pybricks Luenberger observer
- High-level ioctls such as `LEGOMOTOR_RUN_TO_POSITION`, `LEGOMOTOR_RUN_AT_SPEED`

## 7. Reference Files

- `pybricks/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c` -- H-bridge driver
- `pybricks/lib/pbio/src/servo.c` -- PID control loop
- `pybricks/lib/pbio/src/observer.c` -- Luenberger observer
