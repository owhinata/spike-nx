# ファイル転送と ELF 実行

picocom + Zmodem でファイルを転送し、ELF バイナリを実行する方法。

## 前提条件

### ホスト PC

```bash
# macOS
brew install lrzsz picocom

# Ubuntu/Debian
sudo apt install lrzsz picocom
```

### NuttX ファームウェア

以下が有効化されたファームウェアを書き込み済みであること。

| 設定 | 用途 |
|---|---|
| `CONFIG_SYSTEM_ZMODEM=y` | Zmodem (sz/rz) |
| `CONFIG_ELF=y` | ELF ローダー |
| `CONFIG_NSH_FILE_APPS=y` | NSH からファイル実行 (フルパス指定) |

## picocom 接続

```bash
picocom /dev/tty.usbmodem01 -b 115200 \
  --send-cmd "sz -vv -w 256" \
  --receive-cmd "rz -vv"
```

| オプション | 説明 |
|---|---|
| `--send-cmd` | `Ctrl-A Ctrl-S` でファイル送信時に使用されるコマンド |
| `--receive-cmd` | `Ctrl-A Ctrl-R` でファイル受信時に使用されるコマンド |
| `-w 256` | ウィンドウサイズ制限 (フロー制御なし環境向け) |

## ファイル転送

### PC からデバイスへ (アップロード)

1. NSH で `rz` を入力
2. `Ctrl-A Ctrl-S` を押す
3. ファイルパスを入力 (例: `data/imu`)
4. 転送完了を待つ

```
nsh> rz
```

ファイルは `/data/` (Zmodem マウントポイント) に保存される。

### デバイスから PC へ (ダウンロード)

1. NSH で `sz <ファイルパス>` を入力
2. picocom が自動的に `rz` を起動して受信

```
nsh> sz /data/test.txt
```

ファイルはホスト PC のカレントディレクトリに保存される。

## ELF バイナリのビルド

```bash
# エクスポートパッケージ作成 + ELF ビルド (export がなければ自動生成)
make nuttx-elf APP=<app>

# ELF のみ再ビルド (export 済みの場合、高速)
make nuttx-elf APP=<app>

# ELF ビルド成果物クリーン
make nuttx-elf-clean
```

ELF バイナリは `./data/<app>` に出力される。

各アプリの ELF ビルド定義は `apps/<app>/elf.mk` に記載:

```makefile
# apps/imu/elf.mk の例
ELF_BIN  = imu
ELF_SRCS = imu_main.c imu_geometry.c imu_stationary.c imu_fusion.c imu_calibration.c
```

## ELF の転送と実行

```bash
# NSH で rz → Ctrl-A Ctrl-S → data/<app> を選択
nsh> rz

# フルパスで実行
nsh> /data/imu start
nsh> /data/imu status
nsh> /data/imu stop
```

### 注意事項

- ELF 実行は必ずフルパスで指定する (例: `/data/imu`)
- `CONFIG_LIBC_ENVPATH` は有効にしない (NSH の PATH 検索で CPU オーバーヘッドが発生するため)
- ELF が使用するシンボルはカーネルのシンボルテーブル (`g_symtab`) に含まれている必要がある
- シンボルテーブルは `libc.csv` + `syscall.csv` + `libm.csv` + libgcc ヘルパーから生成
- ELF ロード時にコード + データが RAM に配置される
