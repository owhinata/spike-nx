# Debugging Strategy

## 1. Overview

The SPIKE Prime Hub repurposes SWD pins (PA13/PA14) for power control, making traditional SWD debugging difficult. A layered debugging strategy is needed.

### Debug Method Priority

| Priority | Method | Use Case |
|---|---|---|
| **Primary** | USB CDC/ACM NSH + syslog + `dmesg` | Day-to-day development |
| **Secondary** | NuttX coredump → backup SRAM persistence | Post-crash analysis |
| **Tertiary** | Debug build + SWD (USB power only) + OpenOCD | Hard-to-diagnose issues |
| **Diagnostic** | NSH commands (`ps`, `free`, `top`, `/proc`) | Runtime monitoring |
| **Emergency** | Connect-Under-Reset (NRST access required) | Bricked firmware recovery |

---

## 2. NuttX Logging System

### Hierarchical Debug Architecture

NuttX debug output is syslog-based with a two-dimensional hierarchy:

**Priority levels** (independently configurable):

| Config | Level | Description |
|---|---|---|
| `CONFIG_DEBUG_ERROR` | Error | Most critical |
| `CONFIG_DEBUG_WARN` | Warning | Warnings |
| `CONFIG_DEBUG_INFO` | Info | Verbose |

**Subsystems** (each has `_ERROR`, `_WARN`, `_INFO` variants):

| Config | Target |
|---|---|
| `CONFIG_DEBUG_USB` | USB stack |
| `CONFIG_DEBUG_SCHED` | Scheduler |
| `CONFIG_DEBUG_MM` | Memory management |
| `CONFIG_DEBUG_GPIO` | GPIO |
| `CONFIG_DEBUG_I2C` | I2C bus |
| `CONFIG_DEBUG_SPI` | SPI bus |

**Master switch**: `CONFIG_DEBUG_FEATURES=y` enables all subsystem options.

### syslog Output Targets

| Config | Target | Use Case |
|---|---|---|
| `CONFIG_SYSLOG_SERIAL` | UART | Via I/O port UART |
| `CONFIG_SYSLOG_CHARDEV` | Character device | USB CDC/ACM etc. |
| `CONFIG_RAMLOG` | RAM ring buffer | Read via `dmesg`. Preserved when USB disconnects |
| `CONFIG_SYSLOG_FILE` | File | To mounted filesystem |

**Recommendation**: Enable `CONFIG_RAMLOG` as secondary syslog target. Preserves logs when USB CDC/ACM is down; read via `dmesg` after reboot.

---

## 3. Crash Dump / Hard Fault

### Hard Fault Handler

NuttX captures exception frame on Cortex-M hard faults and dumps registers.

**Kconfig**:
```
CONFIG_DEBUG_HARDFAULTS=y        # Verbose fault register output (CFSR, HFSR, MMFAR, BFAR)
CONFIG_STACK_COLORATION=y        # Dump all task stack usage
CONFIG_DEBUG_ASSERTIONS=y        # Register dump on ASSERT/PANIC
```

**Note**: When NSH console is USB CDC/ACM, hard fault output may not reach USB. Configure a UART as secondary output.

### Coredump (Post-Mortem Analysis)

```
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_BOARD_COREDUMP_COMPRESSION=y    # LZF compression (default)
CONFIG_BOARD_COREDUMP_FULL=y
```

Output is hex-encoded + LZF-compressed. Use `tools/coredump.py` to convert to ELF core file → analyze with GDB.

### Crash Data Persistence

**Challenge**: Current NuttX outputs crash dumps to serial/syslog, lost on reset.

**Recommendation for SPIKE Hub**:
- Save crash info to STM32F413's 4KB backup SRAM (battery-backed with VBAT)
- Or write to reserved flash sector
- Read back via NSH command after reboot

---

## 4. NSH Debug Commands

### Process/Task Monitoring

| Command | Description | Required Config |
|---|---|---|
| `ps` | Task list (PID, priority, state, stack size) | Default |
| `free` | Memory stats (total, used, free, largest block) | Default |
| `top` | Dynamic CPU usage | `CONFIG_SYSTEM_TOP` |
| `dmesg` | Dump syslog buffer | `CONFIG_RAMLOG` |

### /proc Filesystem

| Path | Content |
|---|---|
| `/proc/<pid>/status` | Task state |
| `/proc/<pid>/stack` | Stack usage (StackAlloc, StackBase, MaxStackUsed) |
| `/proc/<pid>/group/fd` | Open file descriptors |
| `/proc/meminfo` | System memory info |
| `/proc/uptime` | System uptime |

### Stack Overflow Detection

| Method | Config | Detection Timing | Overhead |
|---|---|---|---|
| Stack coloring | `CONFIG_STACK_COLORATION` | Task exit/fault | Low |
| Context-switch check | Default | Every context switch | Low |
| Per-function check | `CONFIG_ARMV7M_STACKCHECK` | Every function call | High |

---

## 5. SWD Debugging (USB-Powered)

### Principle

After reset, STM32 automatically configures PA13/PA14 as SWDIO/SWCLK. SWD works in the brief window before firmware GPIO initialization.

### Debug Build Strategy

When USB-powered, battery power hold circuit is unnecessary — skip PA13 GPIO reconfiguration:

```c
void stm32_boardinitialize(void)
{
#ifndef CONFIG_BOARD_SWD_DEBUG
    // Production: PA13 = BAT_PWR_EN (HIGH)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET |
                     GPIO_PORTA | GPIO_PIN13);
#endif
    // Debug: PA13 stays as SWDIO (USB power only)
}
```

**Note**: SWD requires **both** PA13 (SWDIO) and PA14 (SWCLK). One pin alone is insufficient.

### OpenOCD Connection

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c '$_TARGETNAME configure -rtos nuttx' \
  -c 'init; reset halt'

arm-none-eabi-gdb --tui nuttx -ex 'target extended-remote localhost:3333'
```

Use [sony/openocd-nuttx](https://github.com/sony/openocd-nuttx) for NuttX thread awareness. `info threads` shows all NuttX tasks.

### Connect-Under-Reset

Even with production builds, SWD works if NRST is accessible:

1. Hold NRST LOW (reset asserted)
2. Connect ST-Link/J-Link in "connect under reset" mode
3. PA13/PA14 function as SWD during reset
4. Halt CPU before GPIO init code
5. Requires physical access to NRST pad on SPIKE Hub PCB

---

## 6. GDB Remote Debugging

### NuttX GDB Server

NuttX provides application-level GDB server over serial.

### GDB via USB CDC/ACM

Possible in principle, but shares port with NSH console. Options:
1. Second CDC/ACM instance dedicated to GDB
2. Use I/O port UART for GDB
3. Mode switching (NSH ↔ GDB)

**Recommendation**: Reserve Port A (UART7) as debug UART during development. Use USB CDC/ACM for NSH console.

---

## 7. Recommended Debug Configuration

### Development Build

```
CONFIG_DEBUG_FEATURES=y
CONFIG_DEBUG_ERROR=y
CONFIG_DEBUG_WARN=y
CONFIG_DEBUG_INFO=y
CONFIG_DEBUG_ASSERTIONS=y
CONFIG_DEBUG_HARDFAULTS=y
CONFIG_DEBUG_SYMBOLS=y
CONFIG_STACK_COLORATION=y
CONFIG_RAMLOG=y
CONFIG_RAMLOG_SYSLOG=y
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_SYSTEM_TOP=y
```

### Release Build

```
CONFIG_DEBUG_ERROR=y
CONFIG_DEBUG_HARDFAULTS=y
CONFIG_STACK_COLORATION=y
CONFIG_RAMLOG=y
CONFIG_COREDUMP=y
```
