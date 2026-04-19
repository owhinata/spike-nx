# Zmodem ファイル転送 (PC ⇔ Hub)

picocom + lrzsz の `sz` / `rz` で USB CDC コンソール経由のファイル転送を行う。Hub 側で受信したファイルは `/mnt/flash` (W25Q256 + LittleFS) に保存される。

## 前提条件

### ホスト PC

```bash
# macOS
brew install lrzsz picocom

# Ubuntu/Debian
sudo apt install lrzsz picocom
```

### NuttX ファームウェア (usbnsh defconfig)

| 設定 | 用途 |
|---|---|
| `CONFIG_SYSTEM_ZMODEM=y` | Zmodem (`sz` / `rz`) |
| `CONFIG_SYSTEM_ZMODEM_DEVNAME="/dev/console"` | USB CDC コンソール経由で転送 |
| `CONFIG_SYSTEM_ZMODEM_MOUNTPOINT="/mnt/flash"` | `rz` 受信先のデフォルト |
| `CONFIG_CDCACM_RXBUFSIZE=2048` | 取りこぼし軽減 (HW フロー制御なし対策) |
| `CONFIG_FS_LITTLEFS=y` + `CONFIG_MTD=y` | `/mnt/flash` の永続ストレージ |
| `CONFIG_NETUTILS_CODECS=y` + `CONFIG_CODECS_HASH_MD5=y` | `md5 -f` で整合性検証 |

詳細は [W25Q256 ドライバ](../drivers/w25q256.md) と [/mnt/flash の使い方](../usage/01-flash-storage.md)。

## PC → Hub (アップロード)

```bash
picocom --send-cmd 'sz -vv -L 256' /dev/tty.usbmodem01
```

picocom 接続後:

1. NSH で `rz` を入力 (Hub を Zmodem 受信モードに)
2. `Ctrl-A Ctrl-S` を押す → ローカルファイルパス (例: `/path/to/file.bin`) を入力 → Enter
3. "Transfer complete" を待つ (1 MB ≒ 45 秒)
4. `Ctrl-A Ctrl-X` で picocom 終了

ファイルは `/mnt/flash/<basename>` に保存される。

```
nsh> ls -l /mnt/flash/file.bin
nsh> md5 -f /mnt/flash/file.bin   # 32-char hex (改行なし)
```

## Hub → PC (ダウンロード)

```bash
cd <受信先ディレクトリ>
picocom --receive-cmd 'rz -vv -y' /dev/tty.usbmodem01
```

picocom 接続後:

1. NSH で `sz /mnt/flash/file.bin` を入力 (Hub を Zmodem 送信モードに)
2. `Ctrl-A Ctrl-R` を押す → 引数なしで Enter (`rz` がプロトコルから受信ファイル名を取得)
3. "Transfer complete" を待つ
4. `Ctrl-A Ctrl-X` で picocom 終了

PC 側で md5 比較:

```bash
md5sum file.bin
```

## オプションについての注意

| オプション | 役割 | 備考 |
|---|---|---|
| `-vv` | 進捗表示 | 必須ではないが転送中の状態が見える |
| `-L 256` (sz) | subpacket size を 256 byte に制限 | **必須** — USB CDC は HW フロー制御がなく、デフォルト 1024 byte だと取りこぼしで ZNAK retry になる |
| `-y` (rz) | 同名ファイルを上書き | 検証サイクルで便利 |
| `-e` (sz/rz) | 制御文字を escape | **付けない** — 8-bit clean な USB CDC で escape は不要かつ ZNAK の原因になる |

## 関連

- [W25Q256 ドライバ仕様](../drivers/w25q256.md)
- [/mnt/flash の使い方 (recovery 含む)](../usage/01-flash-storage.md)
- [テスト仕様 G カテゴリ (pytest 自動化)](../testing/test-spec.md)
