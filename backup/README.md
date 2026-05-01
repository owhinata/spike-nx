# SPIKE Prime Hub フラッシュダンプ

dfu-util で吸い出した実機フラッシュのバックアップ置き場。`.gz` 圧縮で git 管理。

展開:

```bash
gunzip -k backup/*.gz
```

展開後 MD5 で整合性確認できる (下記参照)。

## ファイル

取得日: 2026-05-01。Hub は NuttX 未書き込みで、**LEGO 純正 MicroPython firmware が入っている工場出荷に近い状態** のスナップショット (`MicroPython v1.14-876-gfbecba865 on 2021-05-04; LEGO Technic Large Hub with STM32F413xx`)。

### `spike_0x08008000.bin.gz` — 内蔵フラッシュ (kernel slot 以降)

- **アドレス範囲**: `0x08008000` – `0x080FFFFF` (STM32F413 内蔵フラッシュ)
- **展開後サイズ**: 1,015,808 bytes (992 KB = 0xF8000)
- **展開後 MD5**: `c6cf8af3a8a60a41f8cb03ea4b91baab`
- **中身**: LEGO MicroPython firmware 本体 (uasyncio/* 等の Python ソースも焼き込み済)
- **取得**: `dfu-util -d 0694:0008 -a 0 -s 0x08008000:0xF8000 -U spike_0x08008000.bin`

### `ext_flash_low.bin.gz` — 外部 SPI フラッシュ low region

- **アドレス範囲**: `0x10000000` – `0x100FFFFF` (W25Q256 の先頭 1MB)
- **展開後サイズ**: 1,048,576 bytes (1 MB)
- **展開後 MD5**: `d8daec9cdc85431834a48f512264c9e8`
- **中身**: ARM コードらしきバイナリ (Cortex-M ベクタ風の構造 + 実コード)。LEGO firmware の追加コード or リソースの可能性
- **取得**: `dfu-util -d 0694:0008 -a 0 -s 0x10000000:0x100000 -U ext_flash_low.bin`

### `ext_flash_high.bin.gz` — 外部 SPI フラッシュ high region

- **アドレス範囲**: `0x10100000` – `0x11FFFFFF` (W25Q256 の残り 31MB)
- **展開後サイズ**: 32,505,856 bytes (31 MB = 0x1F00000)
- **展開後 MD5**: `370352d175a318061132cbd83fe98b30`
- **中身**: **littlefs ファイルシステム** (オフセット 0x28 に `littlefs` magic 文字列確認)。LEGO 公式 firmware がユーザプログラム / データを置くストレージ領域。実使用 ~1.1MB、残りは未使用 (0xFF)
- **取得**: `dfu-util -d 0694:0008 -a 0 -s 0x10100000:0x1F00000 -U ext_flash_high.bin`

合計 W25Q256 (32MB) フル。STM32F413 内蔵 1MB と合わせて Hub のフラッシュは全部取れている。

## DFU メモリレイアウト

`dfu-util -l` で得られる alt=0 のメモリマップ:

```
@LEGO LES HUB
  /0x08000000/02*016Ka,02*016Kg,01*064Kg,07*128Kg   ← STM32F413 内蔵フラッシュ 1MB
  /0x10000000/01*1Ma                                 ← W25Q256 先頭 1MB
  /0x10100000/31*1Ma                                 ← W25Q256 残り 31MB
```

末尾文字: `a` = readable のみ / `g` = read+erase+write。

## メモ

- `0x08000000` – `0x08007FFF` (32KB の bootloader 領域) は descriptor では `a` (readable) だが、実際には全 `0xFF` でマスクされる (LEGO ブートローダが自分自身を読み出し保護)
- 内蔵フラッシュは合計 1MB しか DFU で露出していない (STM32F413 は本来 1.5MB)。残り 0.5MB は LEGO ブートローダの descriptor に含まれず
- 外部 SPI フラッシュ (`0x10000000` 起点) は CPU から memory-mapped で見える領域。32MB 全域 readable (`a`)。書き込み不可なので NuttX 側で書き換えたい場合は QSPI ドライバ経由で直接叩く必要あり
- DFU upload は deterministic — 同条件で再取得すれば同じ MD5 が出る (一度確認済)

## リストア (LEGO 純正に戻したい時)

```bash
# まず .gz を展開
gunzip -k backup/*.gz

# Hub を DFU モードに入れた上で、内蔵フラッシュ (kernel slot) を書き戻し
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D backup/spike_0x08008000.bin

# 外部 SPI は DFU 上 'a' (readable のみ) フラグなので書き戻し不可。
# 必要なら NuttX の QSPI/W25Q ドライバ経由で直接書く。
```

bootloader 領域 (`0x08000000`–`0x08007FFF`) は元から書き換え不可なので、内蔵フラッシュの書き戻しだけで実用上は工場出荷状態に戻せる。
