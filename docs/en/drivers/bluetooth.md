# CC2564C Bluetooth (HCI over USART2)

!!! warning "Migration to btstack in progress (Issue #52)"
    The NuttX standard BT host stack finalised in Issue #47 (`CONFIG_WIRELESS_BLUETOOTH_HOST`, `bnep0`, `btsak`) was removed in Issue #52 Step A.  `stm32_bluetooth.c` now only runs the power-on path (32.768 kHz slow clock, nSHUTD toggle) and hands back the instantiated USART2 lower-half.  HCI reset / init script / baud switch and the upper stack (Classic BT SPP over RFCOMM) are moving to the btstack port under `apps/btsensor/` in Steps B–F.

    Everything below this banner documents the Issue #47 architecture.  Step F will rewrite the page for the new stack.

Board-local driver exposing the SPIKE Prime Hub's **TI CC2564C** BR/EDR + BLE dual-mode Bluetooth controller through NuttX.  Bring-up streams a ~6.6 KB TI service pack (init script) over USART2 at 115,200 bps, switches the link to 3 Mbps and registers the controller through the `CONFIG_NET_BLUETOOTH` path as `bnep0` so `btsak` can drive scans and connections.

## Hardware wiring

| Signal | Pin / peripheral | Notes |
|--------|-----------------|-------|
| TX | PD5 (AF7) | USART2 TX |
| RX | PD6 (AF7) | USART2 RX |
| CTS | PD3 (AF7) | HW flow control required |
| RTS | PD4 (AF7) | HW flow control required |
| nSHUTD | PA2 (GPIO output) | CC2564C chip enable (active HIGH, LOW at boot) |
| SLOWCLK | PC9 (AF3, TIM8 CH4) | 32.768 kHz 50% duty sleep clock, stable before nSHUTD HIGH |
| DMA RX | DMA1 Stream 7 Channel 6 | RM0430 Rev 9 Table 30, F413-specific multiplexed mapping #2 (VERY_HIGH) |
| DMA TX | DMA1 Stream 6 Channel 4 | VERY_HIGH |
| NVIC priority | 0xA0 (USART2, DMA1 S6, DMA1 S7) | Issue #50 reserved slot (LUMP=0x90 > BT=0xA0 > IMU=0xB0) |

Pin and DMA allocations mirror pybricks (`lib/pbio/platform/prime_hub/platform.c:80-96`).  TIM8 CH4 delivers 32.768 kHz with PSC=0, ARR=2929, CCR4=1465 (APB2 = 96 MHz → 96 MHz / 2930 ≈ 32.765 kHz, -0.01 % error).

## Software layout

Two board-local drivers plus the NuttX generic upper half:

| Layer | Implementation | Role |
|-------|----------------|------|
| Lower half | `boards/spike-prime-hub/src/stm32_btuart.c` | Implements `struct btuart_lowerhalf_s` (USART2 + DMA1 S6/S7, circular RX DMA + IDLE notification, blocking TX DMA, `setbaud`) |
| Slow clock | `boards/spike-prime-hub/src/stm32_bt_slowclk.c` | TIM8 CH4 32.768 kHz PWM |
| Bring-up | `boards/spike-prime-hub/src/stm32_bluetooth.c` | nSHUTD toggle → HCI_Reset → baud switch → init-script load → `btuart_register()` |
| Init script | `boards/spike-prime-hub/src/cc256x_init_script.c` | TI CC256XC v1.4 firmware patch (derived from pybricks, eHCILL flag patched) |
| Upper half | NuttX `drivers/wireless/bluetooth/bt_uart.c` + `bt_uart_generic.c` | Via `CONFIG_BLUETOOTH_UART_OTHER` + `CONFIG_BLUETOOTH_UART_GENERIC` |
| BT stack | NuttX `wireless/bluetooth/bt_hcicore.c`, etc. | HCI / host layer, `CONFIG_WIRELESS_BLUETOOTH_HOST` |
| netdev | `wireless/bluetooth/bt_netdev.c` | Registers `/dev/bnep0` (`CONFIG_NET_BLUETOOTH`) |

## Bring-up sequence (matches pybricks / btstack ordering)

`stm32_bluetooth_initialize()` follows btstack's canonical HCI init order (`pybricks/lib/btstack/src/hci.h:744` substate enum):

1. **nSHUTD LOW** — held low until the slow clock is stable.
2. **Start TIM8 CH4 slow clock** (32.768 kHz, stable before nSHUTD HIGH).
3. **Instantiate USART2 lower half** (`stm32_btuart_instantiate()`) at 115 200 bps with HW flow control and circular RX DMA armed.
4. **nSHUTD LOW 50 ms → HIGH → wait 150 ms** for the CC2564C ROM boot to settle.
5. **HCI_Reset @ 115,200** — put the controller in a clean state.  This step is essential — performing the bring-up in any order other than HCI_Reset-first caused the chip to emit Hardware Error 0x06 after the baud switch.
6. **HCI_VS_Update_UART_HCI_Baud_Rate (0xFF36) @ 115,200**.  Upon Command Complete, **immediately** call `lower->setbaud(3_000_000)` (no delay — pybricks does the same).
7. **Stream the init script (firmware patch, ~6.6 KB) @ 3 Mbps**.  Each H4-framed HCI command chunk in `cc256x_init_script[]` is sent and verified with a Command Complete.
8. **`btuart_register(lower)`** → NuttX generic upper half → `bt_driver_register_with_id()` → `bt_netdev_register()` → controller appears as `/dev/bnep0`.

Upon success the syslog prints `BT: CC2564C ready at 3000000 bps` at roughly T+0.56 s after boot.

## Key design decisions

### Why pybricks ordering

The initial implementation loaded the init script at 115 200 bps before the baud switch and handed off to NuttX's BT stack, which then issued HCI_Reset at 3 Mbps — and timed out.  Root cause was a combination of three issues:

1. **eHCILL enabled in the init script.** The second `HCI_VS_Sleep_Mode_Configurations` command (opcode 0xFD0C) in the pybricks-derived 1.4 script shipped with its eHCILL flag set to `0x01`.  With eHCILL on the chip sends `GO_TO_SLEEP_IND (0x30)` when the UART is idle and stays asleep until the host replies with `0x31` — something NuttX's generic upper half does not implement.  btstack patches that byte to 0 dynamically when `ENABLE_EHCILL` is undefined (`pybricks/lib/btstack/chipset/cc256x/btstack_chipset_cc256x.c:280`); we replicate that by patching the static blob.
2. **Wrong bring-up order.** pybricks/btstack always issue HCI_Reset, then 0xFF36 + local baud switch, then the init script at the new baud.  Running the init script first and switching baud afterwards left the chip in a state where it emitted Hardware Error 0x06 (`Event_Not_Served_Time_Out`) 10 ms after the switch.
3. **UE toggle + DMA stop/restart during setbaud.** The RM0430 "canonical" sequence (UE=0 → BRR → UE=1) plus rearming the RX DMA opens a multi-microsecond window during which the CC2564C's first post-switch byte is lost.  pybricks just writes BRR directly (`pybricks/lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c:129`); we now do the same.

!!! tip "Porting lesson"
    When a working reference implementation exists (pybricks here), the fastest path is to **copy it verbatim, confirm it works, then refactor**.  Rewriting to be "more conservative" or "more idiomatic NuttX" up front can conflict with chip-specific timing requirements.

### IDLE-driven RX notification

Relying solely on DMA HT/TC callbacks means a 7-byte HCI Command Complete has to wait for the 256-byte half-transfer boundary before the upper half sees it.  Instead the ISR clears the USART2 IDLE flag and reports the producer index directly; DMA HT/TC stays silent (no callback registered) but keeps updating the circular ring.

### rxcallback coalescing

The upper half's `btuart_rxcallback()` queues into a single `work_s` (`bt_uart.c:190`).  Spamming it with notifications returns `-EBUSY`.  The lower half holds a `rxwork_pending` latch: set when a callback is dispatched, cleared when `btuart_read()` drains the ring to empty.

### Known issues in NuttX upstream

- `drivers/wireless/bluetooth/bt_uart_cc2564.c` ships with a zero-filled firmware array, so `btuart_create()` always returns `-EINVAL` — hence the board-local bring-up.
- `arch/arm/src/stm32/stm32_hciuart.c` has multiple compile blockers in the current submodule (L1095/L2409/L2434/L2661) and is unusable without patches.

See [NuttX upstream issues](../nuttx/upstream-issues.md) for the full list.

## Relevant Kconfig

Added to `boards/spike-prime-hub/configs/usbnsh/defconfig`:

```
CONFIG_ALLOW_BSD_COMPONENTS=y
CONFIG_WIRELESS=y
CONFIG_WIRELESS_BLUETOOTH=y
CONFIG_DRIVERS_WIRELESS=y
CONFIG_DRIVERS_BLUETOOTH=y
CONFIG_BLUETOOTH_UART=y
CONFIG_BLUETOOTH_UART_OTHER=y
CONFIG_BLUETOOTH_UART_GENERIC=y
CONFIG_BLUETOOTH_UART_RXBUFSIZE=2048
CONFIG_NET=y
CONFIG_NET_BLUETOOTH=y
CONFIG_NET_SOCKOPTS=y
CONFIG_NETDEV_LATEINIT=y
CONFIG_STM32_USART2=y
CONFIG_STM32_TIM8=y
CONFIG_BTSAK=y
```

`CONFIG_NETDEV_LATEINIT=y` suppresses the unresolved `arm_netinitialize()` reference that appears when `CONFIG_NET=y` is set but no ethernet/wifi driver provides the symbol — our Bluetooth netdev registers itself much later from `btuart_register()` instead.

## NSH examples

```
nsh> ifconfig                 # bnep0 should be listed
bnep0    Link encap:UNSPEC at DOWN mtu -9
    inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0

nsh> bt bnep0 info            # BD address via HCI_Read_BD_ADDR
Device: bnep0
BDAddr: f8:2e:0c:a0:3e:64
...

nsh> bt bnep0 scan start
nsh> bt bnep0 scan get        # List of nearby BT peers
nsh> bt bnep0 scan stop
```

## Currently out of scope

- BR/EDR profiles (A2DP / HFP / SPP)
- GATT server (BLE peripheral role)
- Persisting pairing info (requires Flash support wiring)
- Low-power modes / eHCILL / deep sleep

## Related documents

- [Test spec — H. Bluetooth](../testing/test-spec.md#h-bluetooth-test_bluetoothpy) — pytest automated + interactive tests
- [DMA / IRQ allocation ledger](../hardware/dma-irq.md) — the project-wide DMA stream and NVIC-priority design
- [Pin mapping](../hardware/pin-mapping.md) — Bluetooth section
- [NuttX upstream issues](../nuttx/upstream-issues.md) — why we rolled our own instead of using the upstream CC2564 / HCI UART drivers

## Source references

- pybricks: `lib/pbio/platform/prime_hub/platform.c:80-96` (pin / DMA allocation)
- pybricks: `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c:49-194` (HAL wrapper)
- pybricks: `lib/btstack/chipset/cc256x/btstack_chipset_cc256x.c:280` (eHCILL flag patch)
- pybricks: `lib/btstack/src/hci.h:744-759` (HCI init substate order)
- pybricks: `lib/btstack/src/hci.c:1847-1901` (baud change + custom init)
- TI: [CC256XC-BT-SP service pack](https://www.ti.com/tool/CC256XC-BT-SP) (init script source)
- RM0430 Rev 9 §9.3.4 Figure 24 / Table 30 (DMA1 request mapping, 4-bit CHSEL extension)
