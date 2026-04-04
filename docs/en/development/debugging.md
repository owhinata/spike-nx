# Debugging

Debugging techniques for the SPIKE Prime Hub. Since the SWD pins (PA13/PA14) are repurposed for power control, SWD debugging is not used.

## Debugging Methods

| Priority | Method | Use Case |
|---|---|---|
| Primary | USB CDC/ACM NSH + syslog + `dmesg` | Day-to-day development |
| Secondary | NuttX coredump | Post-crash analysis |
| Diagnostic | NSH commands (`ps`, `free`, `top`, `/proc`) | Runtime monitoring |
| Last resort | Temporarily use an I/O port UART for debug output | When USB is non-functional |

## RAMLOG and dmesg

Enabling `CONFIG_RAMLOG` saves syslog output to a RAM ring buffer. Logs are retained even if USB CDC/ACM goes down, and can be reviewed with `dmesg` after reconnection.

Messages from the early boot stage are output before the USB driver is operational, so they can only be viewed via RAMLOG.

```
CONFIG_RAMLOG=y
CONFIG_RAMLOG_SYSLOG=y
```

## DEBUG Subsystem

`CONFIG_DEBUG_FEATURES=y` is the master switch. Once enabled, ERROR / WARN / INFO levels can be configured individually per subsystem.

### Priority Levels

| Setting | Level | Description |
|---|---|---|
| `CONFIG_DEBUG_ERROR` | Error | Most critical |
| `CONFIG_DEBUG_WARN` | Warning | Warnings |
| `CONFIG_DEBUG_INFO` | Info | Detailed (most verbose) |

### Subsystems

| Setting | Target |
|---|---|
| `CONFIG_DEBUG_USB` | USB stack |
| `CONFIG_DEBUG_SCHED` | Scheduler |
| `CONFIG_DEBUG_MM` | Memory management |
| `CONFIG_DEBUG_GPIO` | GPIO |
| `CONFIG_DEBUG_I2C` | I2C bus |
| `CONFIG_DEBUG_SPI` | SPI bus |

## Recommended Kconfig Settings

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

## NSH Debug Commands

### Process/Task Monitoring

| Command | Description | Required Setting |
|---|---|---|
| `ps` | Task list (PID, priority, state, stack size) | Default |
| `free` | Memory statistics (total, used, free, largest free block) | Default |
| `top` | Dynamic CPU usage display | `CONFIG_SYSTEM_TOP` |
| `dmesg` | Read syslog buffer | `CONFIG_RAMLOG` |

### /proc Filesystem

| Path | Contents |
|---|---|
| `/proc/<pid>/status` | Task status |
| `/proc/<pid>/stack` | Stack usage (StackAlloc, StackBase, MaxStackUsed) |
| `/proc/<pid>/group/fd` | Open file descriptors |
| `/proc/meminfo` | System memory information |
| `/proc/uptime` | System uptime |

## HardFault Analysis

NuttX outputs a register dump from the exception frame when a Cortex-M HardFault occurs.

```
CONFIG_DEBUG_HARDFAULTS=y        # Fault register output (CFSR, HFSR, MMFAR, BFAR)
CONFIG_STACK_COLORATION=y        # Dump stack usage for all tasks
CONFIG_DEBUG_ASSERTIONS=y        # Register dump on ASSERT/PANIC paths
```

### Fault Registers

| Register | Description |
|---|---|
| CFSR (Configurable Fault Status Register) | Details of UsageFault / BusFault / MemManage |
| HFSR (HardFault Status Register) | Cause of HardFault |
| MMFAR (MemManage Fault Address Register) | Address of memory management violation |
| BFAR (BusFault Address Register) | Address of bus fault |

### Panic Syslog Output

Crash dumps are output through the panic syslog channel (`stm32_panic_syslog.c`) to **USB CDC ACM (stdout)**. This works for ASSERT paths, but may not work during HardFault as the USB driver may be inoperable.

Use the `crashtest assert` command to verify panic syslog operation. During HardFault (bus fault, etc.), the Cortex-M enters a lockup state and does not go through the panic path, so no dump is output. In this case, the IWDG (3-second timeout) triggers a reset.

### Coredump

Post-mortem analysis is possible using the NuttX coredump subsystem:

```
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_BOARD_COREDUMP_COMPRESSION=y    # LZF compression
CONFIG_BOARD_COREDUMP_FULL=y           # Full task information
```

The output is hex-encoded + LZF-compressed. Convert it to an ELF core file with `tools/coredump.py` and analyze with GDB.

## Stack Overflow Detection

| Method | Setting | Detection Timing | Overhead |
|---|---|---|---|
| Stack coloring | `CONFIG_STACK_COLORATION` | On task exit / fault | Low |
| Context switch check | Default | On context switch | Low |
| Function-level check | `CONFIG_ARMV7M_STACKCHECK` | On every function call | High |
