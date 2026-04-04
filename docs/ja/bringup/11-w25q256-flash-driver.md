# W25Q256 Flash ドライバ

## 1. 概要

W25Q256 は Winbond の 256 Mbit (32 MB) SPI NOR Flash。32MB のため 4byte アドレッシングが必要。

| 項目 | 値 |
|---|---|
| 容量 | 32 MB (256 Mbit) |
| インターフェース | SPI (標準/Dual/Quad) |
| ページサイズ | 256 bytes |
| セクタサイズ | 4 KB |
| ブロックサイズ | 64 KB |
| デバイス ID | 0xEF 0x40 0x19 |

---

## 2. SPIKE Hub 接続

| 信号 | 接続先 | 備考 |
|---|---|---|
| SPI | SPI2 | DMA1 Stream4 (TX, ch0) / Stream3 (RX, ch0) |
| CS | PB12 | ソフトウェア NSS (Active Low) |
| SPI モード | CPOL=0, CPHA=0 | MSB first, prescaler /2 |

---

## 3. 4byte アドレッシング

### 2つのアプローチ

| 方式 | 説明 | コマンド例 |
|---|---|---|
| A: アドレスモード切替 | `0xB7` で 4byte モードに入る | 標準コマンド (0x03, 0x02, 0x20) が 4byte アドレスを使用 |
| **B: 4byte 専用コマンド** | 常に 4byte アドレスのコマンドを使用 | 0x13/0x0C (read), 0x12 (write), 0x21 (erase) |

### pybricks の実装: 方式 B (4byte 専用コマンド)

```c
FLASH_CMD_READ_DATA  = 0x0C  // Fast Read with 4-Byte Address
FLASH_CMD_WRITE_DATA = 0x12  // Page Program with 4-Byte Address
FLASH_CMD_ERASE_BLOCK = 0x21 // Sector Erase with 4-Byte Address
```

アドレスモードレジスタの状態に依存しないため、より堅牢。

### アドレスフォーマット

```c
buf[0] = address >> 24;  // 最上位バイト
buf[1] = address >> 16;
buf[2] = address >> 8;
buf[3] = address;        // 最下位バイト
```

### SPI コマンドフォーマット (Read 例)

```
[0x0C] [addr3] [addr2] [addr1] [addr0] [dummy] [data...]
```

---

## 4. フラッシュメモリレイアウト

| 領域 | アドレス (SPI Flash) | サイズ | 用途 |
|---|---|---|---|
| ブートローダーデータ | 0x000000 - 0x07FFFF | 512 KB | LEGO ブートローダー領域 |
| pybricks ブロックデバイス | 0x080000 - 0x0BFFFF | 256 KB | ユーザープログラムバックアップ |
| 更新キー | 0x0FF000 - 0x0FFFFF | 4 KB | mboot FS-load キー |
| ファイルシステム | 0x100000 - 0x1FFFFFF | 31 MB | FAT ファイルシステム |

NuttX では先頭 1 MB を予約域として扱い、1 MB 以降をファイルシステムに使用するのが安全。

---

## 5. NuttX ドライバ対応

### 既存ドライバの課題

| ドライバ | Kconfig | 問題 |
|---|---|---|
| `w25.c` (SPI) | `MTD_W25` | 3byte アドレスのみ (W25Q128 まで) |
| `w25qxxxjv.c` (QSPI) | `W25QXXXJV` | QSPI 専用 (標準 SPI 非対応) |

### 推奨アプローチ

**`w25.c` を拡張して 4byte コマンド対応を追加**:

1. W25Q256 のデバイス ID (0xEF4019) を JEDEC ID テーブルに追加
2. 4byte アドレスコマンド (0x0C, 0x12, 0x21) のサポートを追加
3. アドレス送信部分を 3byte/4byte で分岐

変更量は中程度。pybricks のドライバ (`block_device_w25qxx_stm32.c`) を参考に実装可能。

### 代替アプローチ

- 先頭 16 MB のみ使用 (3byte アドレスで十分) → ファイルシステム用に 15 MB 利用可能
- 独立した MTD ドライバを新規作成 (pybricks ドライバをベースに)

---

## 6. 参照ファイル

- `pybricks/lib/pbio/drv/block_device/block_device_w25qxx_stm32.c` — pybricks W25Q256 ドライバ
- `pybricks/micropython/ports/stm32/boards/LEGO_HUB_NO6/mpconfigboard.h` — 32bit アドレス設定
