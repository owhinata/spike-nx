# CC2564C Bluetooth (HCI over USART2)

!!! warning "btstack への移行中 (Issue #52)"
    Issue #47 で完成した NuttX 標準 BT スタック (`CONFIG_WIRELESS_BLUETOOTH_HOST` / `bnep0` / `btsak`) は Issue #52 Step A で撤去しました。`stm32_bluetooth.c` は電源と 32.768 kHz slow clock の起動、USART2 lower-half の instantiate までに縮退しています。HCI reset / init script / baud 切替 / 上位スタック (Classic BT SPP over RFCOMM) はすべて `apps/btsensor/` の btstack port に移管中 (Step B〜F)。

    このページの「ソフトウェア構成」以降の記述は Issue #47 時点の設計メモです。新アーキテクチャは Step F で全面書き直しの予定です。

SPIKE Prime Hub の **TI CC2564C** (BR/EDR + BLE デュアルモード Bluetooth コントローラ) を NuttX から使えるようにする board-local ドライバ。USART2 HCI-UART 接続で、起動時に約 6.6 KB の TI service pack (init script) を送信 → 115200 から 3 Mbps にボーレート切替 → NuttX の `CONFIG_NET_BLUETOOTH` 経由で `bnep0` netdev として登録、`btsak` からスキャン/接続可能にする。

## ハードウェア構成

| 信号 | ピン / 周辺機能 | 説明 |
|------|----------------|------|
| TX | PD5 (AF7) | USART2 TX |
| RX | PD6 (AF7) | USART2 RX |
| CTS | PD3 (AF7) | HW flow control 必須 |
| RTS | PD4 (AF7) | HW flow control 必須 |
| nSHUTD | PA2 (GPIO output) | CC2564C chip enable (Active HIGH、初期 LOW) |
| SLOWCLK | PC9 (AF3, TIM8 CH4) | 32.768 kHz 50% duty sleep clock (nSHUTD HIGH 前に安定化) |
| DMA RX | DMA1 Stream 7 Channel 6 | RM0430 Rev 9 Table 30 の F413 固有多重マッピング #2 (VERY_HIGH) |
| DMA TX | DMA1 Stream 6 Channel 4 | VERY_HIGH |
| NVIC 優先度 | 0xA0 (USART2, DMA1 S6, DMA1 S7) | Issue #50 予約枠 (LUMP=0x90 > BT=0xA0 > IMU=0xB0) |

ピン配線と DMA 割当は pybricks `lib/pbio/platform/prime_hub/platform.c:80-96` と一致。TIM8 CH4 の 32.768 kHz 生成は PSC=0, ARR=2929, CCR4=1465 (APB2 96 MHz / 2930 = 32.765 kHz, 誤差 -0.01%)。

## ソフトウェア構成

2 つの board-local driver + NuttX generic upper-half:

| レイヤ | 実装 | 役割 |
|---|---|---|
| Lower-half | `boards/spike-prime-hub/src/stm32_btuart.c` | `struct btuart_lowerhalf_s` 実装 (USART2 + DMA1 S6/S7、RX circular DMA + IDLE 通知、blocking TX DMA、setbaud) |
| Slow clock | `boards/spike-prime-hub/src/stm32_bt_slowclk.c` | TIM8 CH4 PWM (32.768 kHz) |
| Bring-up | `boards/spike-prime-hub/src/stm32_bluetooth.c` | nSHUTD toggle → HCI_Reset → baud 切替 → init script ロード → `btuart_register()` |
| Init script | `boards/spike-prime-hub/src/cc256x_init_script.c` | TI CC256XC v1.4 の firmware patch (pybricks 由来、eHCILL 無効化パッチ済) |
| Upper-half | NuttX `drivers/wireless/bluetooth/bt_uart.c` + `bt_uart_generic.c` | `CONFIG_BLUETOOTH_UART_OTHER` + `CONFIG_BLUETOOTH_UART_GENERIC` 経由 |
| BT stack | NuttX `wireless/bluetooth/bt_hcicore.c` 等 | HCI 層、`CONFIG_WIRELESS_BLUETOOTH_HOST` |
| netdev | `wireless/bluetooth/bt_netdev.c` | `/dev/bnep0` として netdev 登録 (`CONFIG_NET_BLUETOOTH`) |

## Bring-up シーケンス (pybricks/btstack 順序)

pybricks の btstack HCI init (`pybricks/lib/btstack/src/hci.h` L744 の substate enum) に合わせて、`stm32_bluetooth_initialize()` は以下の順序で実行する:

1. **nSHUTD LOW** (初期状態、chip を reset で保持)
2. **TIM8 CH4 slow clock 起動** (32.768 kHz、nSHUTD HIGH 前に安定化)
3. **USART2 lower-half instantiate** (`stm32_btuart_instantiate()`) — 115200 bps、HW flow control、RX 循環 DMA 起動
4. **nSHUTD LOW → 50 ms → HIGH → 150 ms 待機** (CC2564C ROM boot 完了)
5. **HCI_Reset @ 115200** — チップをクリーン状態にする (この順序は重要、pybricks/btstack と一致)
6. **HCI_VS_Update_UART_HCI_Baud_Rate (0xFF36) @ 115200** → CC 受信後 **即座に `lower->setbaud(3000000)`**
7. **init script (firmware patch, 約 6.6 KB) @ 3 Mbps** — `cc256x_init_script[]` を H4 framed HCI command として順送信、各チャンクで Command Complete を確認
8. **`btuart_register(lower)`** → NuttX 汎用 upper-half → `bt_driver_register_with_id()` → `bt_netdev_register()` → `/dev/bnep0` として netdev 登録

完了時 syslog `BT: CC2564C ready at 3000000 bps` が出力される (約 0.56 秒)。

## 重要な設計決定

### pybricks 順序の採用経緯

当初実装では「init script @ 115200 → baud 切替 → NuttX BT スタックへ hand off」の順序だった。3 Mbps 切替直後に chip が Hardware Error 0x06 (`Event_Not_Served_Time_Out`) を送信し、以降の HCI_Reset に無応答となる現象が発生。原因は以下 3 つの複合要因:

1. **eHCILL 有効状態**: init script (pybricks 1.4 由来) の 2 番目の `HCI_VS_Sleep_Mode_Configurations` コマンド (opcode 0xFD0C) の eHCILL flag が `0x01` (enabled) だった。eHCILL が有効だと chip が idle 時に `GO_TO_SLEEP_IND (0x30)` を送信し、host が `0x31` で ack しない限り chip はスリープで無応答になる。btstack は `ENABLE_EHCILL` 未定義なら flag を 0 に動的パッチする (`pybricks/lib/btstack/chipset/cc256x/btstack_chipset_cc256x.c:280`)。我々は静的ファイルにパッチを入れて同等化。
2. **実装順序が pybricks と逆**: pybricks/btstack は `HCI_Reset → 0xFF36 → baud 切替 → init script @ 新 baud` の順。init script が適用された chip に baud 切替は正常に動作しないケースが発生。pybricks 順序に合わせることで解決。
3. **setbaud の UE トグル + DMA 再開**: 元々 RM0430 推奨手順の `CR1.UE=0 → BRR 更新 → CR1.UE=1` + RX DMA stop/restart を使っていた。このシーケンスは一般 UART peripheral には正しいが、CC2564C の baud 切替後 timing 要件に合わず、切替直後の byte を取りこぼす。pybricks は `LL_USART_SetBaudRate()` で BRR を直接書くだけ (`pybricks/lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c:129`)。同じ挙動にすることで解決。

!!! tip "移植プロジェクトでの教訓"
    実機で動作実績のある参照実装 (pybricks) がある場合、まず **verbatim にコピーして動作確認してから差分を加える** のが最短ルート。NuttX 慣習や RM 推奨手順に合わせて "より保守的に" リライトすると、チップ固有の timing 要件と噛み合わないことがある。

### RX 通知源に USART IDLE を使う

DMA HT/TC だけだと 512 バイトリングの半分まで到達しないと通知されず、7 バイトの HCI Command Complete が長時間待たされる。USART2 IDLE 割込を主通知源にし、DMA HT/TC は producer index 更新の補助として使う設計にしている (`btuart_usart_isr()` 参照)。

### ring buffer coalescing

上位半の `btuart_rxcallback()` は 1 つの `work_s` に対して `work_queue(HPWORK, ...)` を投げるだけ (`bt_uart.c:190`)。過剰通知で `-EBUSY` を避けるため、lower 側に `rxwork_pending` latch を持ち、ring が drain された時点で読み側 (`btuart_read()`) が clear する。

### NuttX upstream の既知問題

- `drivers/wireless/bluetooth/bt_uart_cc2564.c`: firmware blob が `{ 0 }` 固定で、`btuart_create()` が `-EINVAL` で fail。Board-local 実装にした理由
- `arch/arm/src/stm32/stm32_hciuart.c`: compile blocker 複数 (L1095/L2409/L2434/L2661)。使用不可

詳細は [NuttX upstream issues](../nuttx/upstream-issues.md) を参照。

## 関連 Kconfig

defconfig に追加 (`boards/spike-prime-hub/configs/usbnsh/defconfig`):

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

`CONFIG_NETDEV_LATEINIT=y` は `arm_netinitialize()` の未解決参照を抑制するため必須 (ether/wifi ドライバ無しで BT netdev のみ late register するため)。

## NSH 操作例

```
nsh> ifconfig                  # bnep0 が表示される
bnep0    Link encap:UNSPEC at DOWN mtu -9
    inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0

nsh> bt bnep0 info             # BD address 取得
Device: bnep0
BDAddr: f8:2e:0c:a0:3e:64
...

nsh> bt bnep0 scan start
nsh> bt bnep0 scan get         # 周囲 BT デバイスリスト
nsh> bt bnep0 scan stop
```

## 現時点での対象外 (将来拡張)

- BR/EDR プロファイル (A2DP / HFP / SPP)
- GATT サーバ (BLE peripheral role)
- ペアリング情報の永続化 (Flash 実装待ち)
- 省電力 / eHCILL / Deep Sleep

## 関連ドキュメント

- [テスト仕様書 H. Bluetooth](../testing/test-spec.md#h-bluetooth-test_bluetoothpy) — pytest 自動/対話テスト
- [DMA / IRQ 割当台帳](../hardware/dma-irq.md) — DMA ストリーム・NVIC 優先度の全体設計
- [ピンマップ](../hardware/pin-mapping.md) — Bluetooth 節
- [NuttX upstream の既知問題](../nuttx/upstream-issues.md) — board-local 実装を選んだ理由

## 参照ソース

- pybricks: `lib/pbio/platform/prime_hub/platform.c:80-96` (pin/DMA 割当)
- pybricks: `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c:49-194` (HAL wrapper)
- pybricks: `lib/btstack/chipset/cc256x/btstack_chipset_cc256x.c:280` (eHCILL flag patch)
- pybricks: `lib/btstack/src/hci.h:744-759` (HCI init substate 順序)
- pybricks: `lib/btstack/src/hci.c:1847-1901` (baud change + custom init)
- TI: [CC256XC-BT-SP service pack](https://www.ti.com/tool/CC256XC-BT-SP) (init script 元ファイル)
- RM0430 Rev 9 §9.3.4 Figure 24 / Table 30 (DMA1 request mapping, CHSEL 4-bit 拡張)
