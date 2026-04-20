# NuttX upstream の既知問題

本プロジェクトが NuttX submodule (owhinata fork of 12.13.0) を読んだ際に発見した既知の問題を記録する。apache/nuttx への起票はせず、本リポジトリ内の記録に留める (memory `feedback_no_nuttx_upstream_issues.md` 方針)。必要になった場合は owhinata/nuttx fork で修正する。

## Bluetooth UART: `stm32_hciuart.c` の compile blocker

**対象**: `nuttx/arch/arm/src/stm32/stm32_hciuart.c`

`CONFIG_STM32_USART*_HCIUART` + `CONFIG_STM32_HCIUART_RXDMA` を有効化すると本ファイルがビルドに含まれるが、静的読解で複数の compile blocker が確認された:

| 行 | 内容 |
|---|---|
| L1095 | `rxnext = 0` — 末尾セミコロン欠落 |
| L2409 | `config.state->rxdmastream` — `config` がポインタなので `config->state->...` が正しい |
| L2434 | `handled = true;` — `handled` が未宣言 |
| L2661-2666 | `g_hciusart1_config.state`, `g_hciusart2_config.state` 同類 |

これらはいずれもファイル内部のタイプミスで、CONFIG を有効化しない限り顕在化しないため upstream では未検出のまま残っている。

**本プロジェクトでの対応**: Issue #47 (CC2564C Bluetooth) では `stm32_hciuart.c` を使用せず、board-local の `boards/spike-prime-hub/src/stm32_btuart.c` に `struct btuart_lowerhalf_s` を自前実装した。詳細は [Bluetooth ドライバ](../drivers/bluetooth.md) を参照。

## Bluetooth UART: `bt_uart_cc2564.c` の firmware 未同梱

**対象**: `nuttx/drivers/wireless/bluetooth/bt_uart_cc2564.c`

L48-58 にて `cc256x_firmware` / `ble_firmware` 配列が `#warning` 付きで `{ 0 }` 固定定義されており、ユーザーが firmware blob を `.c` ファイル内に手で書き込まない限り動作しない (TI から `.bts` を取得して変換するのは SPDX / ライセンス整理の観点で upstream では難しい)。

`btuart_create()` は L163-209 で必ず `load_cc2564_firmware()` を実行し、サイズ検査で `-EINVAL` を返す (L135-138):

```c
if (sizeof(cc256x_firmware) < 10 || sizeof(ble_firmware) < 10)
  {
    return -EINVAL;
  }
```

**本プロジェクトでの対応**: `CONFIG_BLUETOOTH_UART_CC2564=n` とし、代わりに汎用 upper-half (`CONFIG_BLUETOOTH_UART_OTHER` + `CONFIG_BLUETOOTH_UART_GENERIC`) を使用。firmware blob は pybricks 由来 (TI Text File License、CC2564C と組合せて再配布可) を `boards/spike-prime-hub/src/cc256x_init_script.c` に配置し、bring-up コードが自前でチップに転送する。

## USART2 Kconfig choice の排他性

**対象**: `nuttx/arch/arm/src/stm32/Kconfig` L9825-9846

`CONFIG_STM32_USART2` が選択されたときの USART2 ドライバ選択は Kconfig の `choice` で 3 択 (SERIALDRIVER / 1WIREDRIVER / HCIUART) になっており、デフォルトが `STM32_USART2_SERIALDRIVER`。BT UART として USART2 を専有したい場合、

- HCIUART を選択すると上記の compile blocker で build fail
- SERIALDRIVER を残すと `stm32_serial.c` が USART2 を自動 config し `/dev/ttyS2` を登録 (board-local driver と USART2 レジスタを奪い合う)
- 1WIREDRIVER は 1-wire 用、本用途に不適

**本プロジェクトでの対応**: SERIALDRIVER を残したまま、board-local `stm32_btuart.c` が後から USART2 レジスタを上書きする形で共存。`/dev/ttyS2` は /dev に残るが誰も使わない想定。詳細は `stm32_btuart.c` の file header コメント参照。

## 参照

- memory: `feedback_no_nuttx_upstream_issues.md` — apache/nuttx への起票禁止、必要なら fork で修正
- memory: `feedback_nuttx_minimal_changes.md` — submodule への変更は必要最小限
