# /mnt/flash (W25Q256 + LittleFS)

SPIKE Prime Hub の外付け W25Q256 (32 MB SPI NOR) に NuttX LittleFS でアクセスし、設定データ・プログラム・ログ等の永続化を行う。NSH の `rz` / `sz` で PC ⇔ Hub のファイル転送ができる (picocom 経由の操作手順は [Zmodem ファイル転送](../development/file-transfer.md))。

## マウントと容量

起動後、`/mnt/flash` に LittleFS が自動マウントされる:

```
nsh> mount
  /mnt/flash type littlefs
  /proc type procfs
nsh> df -h /mnt/flash
```

## 通常のファイル操作

POSIX ライクな NSH コマンドが使える:

```
nsh> echo "hello" > /mnt/flash/note.txt
nsh> cat /mnt/flash/note.txt
hello
nsh> ls -l /mnt/flash
nsh> rm /mnt/flash/note.txt
```

reboot しても内容は保持される (LittleFS の電源断耐性 + NOR Flash の不揮発性)。

## 整合性確認 (md5)

ファイルが転送中に壊れていないかを確認する:

```
nsh> md5 -f /mnt/flash/file.bin
e3b0c44298fc1c149afbf4c8996fb924
```

NSH の `md5` は 32-char hex digest を **末尾改行なし**で出力する点に注意 (スクリプトから扱う場合は `; echo` で改行を補う)。

## パーティションとデバイス

| 領域 | アドレス | サイズ | 公開デバイス |
|---|---|---|---|
| LEGO bootloader | `0x000000-0x07FFFF` | 512 KB | (非公開) |
| pybricks block device | `0x080000-0x0BFFFF` | 256 KB | (非公開) |
| Update key | `0x0FF000-0x0FFFFF` | 4 KB | (非公開) |
| LittleFS partition | `0x100000-0x1FFFFFF` | 31 MB | `/dev/mtdblock0` → `/mnt/flash` |

full chip raw (`/dev/mtd0`) は user 空間に公開していない (LEGO 領域の誤消去防止)。デバッグで raw access が必要な場合は [W25Q256 ドライバ](../drivers/w25q256.md) の "dev-only raw access" 節を参照。

!!! warning "LittleFS 専用領域"
    `0x100000-0x1FFFFFF` (31 MB) は LittleFS が完全管理する。raw 書込みされた non-LittleFS データは初回起動時の自動フォーマットで消去される。

## トラブルシューティング

### `/mnt/flash` がマウントされない

`dmesg` で `W25Q256:` 行を確認する:

- `JEDEC ID 0xef4019, 32MB detected` が出ていること (出ていなければ SPI 配線 / SPI2 設定不良)
- `LittleFS mounted at /mnt/flash` が出ていること

mount 失敗時は手動 recovery:

```
nsh> mount -t littlefs -o autoformat /dev/mtdblock0 /mnt/flash
```

それでもダメなら electrically partition 全消去 (現状ユーティリティ未実装)。再フラッシュ後 boot 時の forceformat フォールバックで復旧する場合がある。

### ファイル転送関連のトラブル

[Zmodem ファイル転送](../development/file-transfer.md) の "オプションについての注意" を参照 (ZNAK retry、`-e` 禁止、`-L 256` 必須 等)。

## 関連ドキュメント

- ドライバ仕様: [W25Q256 ドライバ](../drivers/w25q256.md)
- ファイル転送手順: [Zmodem ファイル転送](../development/file-transfer.md)
- ハードウェア: [W25Q256 ペリフェラル](../hardware/peripherals.md), [DMA / IRQ 割当](../hardware/dma-irq.md)
- テスト: [test-spec G カテゴリ](../testing/test-spec.md)
