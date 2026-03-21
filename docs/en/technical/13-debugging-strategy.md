# Debugging Strategy

## 1. Overview

The SPIKE Prime Hub repurposes SWD pins (PA13/PA14) for power control, and extracting them externally is impractical, so SWD debugging is not used. USB CDC/ACM NSH console is the primary debug method.

### Debug Method Priority

| Priority | Method | Use Case |
|---|---|---|
| **Primary** | USB CDC/ACM NSH + syslog + `dmesg` | Day-to-day development |
| **Secondary** | NuttX coredump → backup SRAM persistence | Post-crash analysis |
| **Diagnostic** | NSH commands (`ps`, `free`, `top`, `/proc`) | Runtime monitoring |
| **Last resort** | Temporarily use I/O port UART for debug output | Initial bring-up when USB not working |

### Methods NOT Used

- **SWD**: PA13/PA14 pin extraction impractical — not used
- **GDB**: Not used by default. May use via I/O port UART if absolutely necessary

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
| `CONFIG_DEBUG_NET` | Networking |
| `CONFIG_DEBUG_FS` | Filesystem |

**Master switch**: `CONFIG_DEBUG_FEATURES=y` enables all subsystem options.

### syslog Output Targets

| Config | Target | Use Case |
|---|---|---|
| `CONFIG_SYSLOG_CHARDEV` | Character device | USB CDC/ACM (primary output) |
| `CONFIG_RAMLOG` | RAM ring buffer | Read via `dmesg`. Preserved when USB disconnects |
| `CONFIG_SYSLOG_FILE` | File | To mounted filesystem |

**Recommendation**: Always enable `CONFIG_RAMLOG`. Preserves logs when USB CDC/ACM is down; read via `dmesg` after reconnect. Particularly important for early boot messages emitted before the USB driver is active — RAMLOG is the only way to see them.

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

**Note**: When NSH console is USB CDC/ACM, the USB driver itself may be non-functional during a hard fault. If crash info is written to RAMLOG, it can be checked via `dmesg` after re-flashing firmware via DFU (if backup SRAM persistence is implemented).

### Coredump (Post-Mortem Analysis)

```
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_BOARD_COREDUMP_COMPRESSION=y    # LZF compression (default)
CONFIG_BOARD_COREDUMP_FULL=y
```

Output is hex-encoded + LZF-compressed. Use `tools/coredump.py` to convert to ELF core file → analyze with GDB.

### Crash Data Persistence

**Challenge**: Crash dumps are output to serial/syslog, lost on reset.

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

## 5. Recommended Debug Configuration

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
