# ファイル転送と ELF 実行

B-L4S5I-IOT01A 上の LittleFS (`/data`) に対して、picocom + Zmodem でファイルを転送し、ELF バイナリを実行する方法。

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

- `CONFIG_SYSTEM_ZMODEM=y` — Zmodem (sz/rz)
- `CONFIG_ELF=y` — ELF ローダー
- `CONFIG_NSH_FILE_APPS=y` — NSH からファイル実行 (フルパス指定)

## picocom 接続

```bash
picocom /dev/cu.usbmodem1103 -b 115200 \
  --send-cmd "sz -vv -w 256" \
  --receive-cmd "rz -vv"
```

- `--send-cmd`: `Ctrl-A Ctrl-S` でファイル送信時に使用されるコマンド
- `--receive-cmd`: `Ctrl-A Ctrl-R` でファイル受信時に使用されるコマンド
- `-w 256`: ウィンドウサイズ制限 (フロー制御なし環境向け)

## ファイル転送

### PC → デバイス (アップロード)

1. NSH で `rz` を入力
2. `Ctrl-A Ctrl-S` を押す
3. ファイルパスを入力 (例: `data/imu`)
4. 転送完了を待つ

```
nsh> rz
```

ファイルは `/data/` (Zmodem マウントポイント) に保存される。

### デバイス → PC (ダウンロード)

1. NSH で `sz <ファイルパス>` を入力
2. picocom が自動的に `rz` を起動して受信

```
nsh> sz /data/test.txt
```

ファイルはカレントディレクトリに保存される。

## ELF 実行

### ELF バイナリのビルド

```bash
# エクスポートパッケージ作成 + ELF ビルド (export がなければ自動生成)
make nuttx-elf APP=imu BOARD=b-l4s5i-iot01a
```

ELF バイナリは `./data/imu` に出力される。

各アプリの ELF ビルド定義は `apps/<app>/elf.mk` に記載:

```makefile
# apps/imu/elf.mk
ELF_BIN  = imu
ELF_SRCS = imu_main.c imu_geometry.c imu_stationary.c imu_fusion.c imu_calibration.c
```

### 転送と実行

```bash
# NSH で rz → Ctrl-A Ctrl-S → data/imu を選択
nsh> rz

# フルパスで実行
nsh> /data/imu start
nsh> /data/imu status
nsh> /data/imu stop
```

### ビルドコマンド一覧

```bash
# ELF ビルド (export がなければ自動生成)
make nuttx-elf APP=imu BOARD=b-l4s5i-iot01a

# ELF のみ再ビルド (export 済みの場合、高速)
make nuttx-elf APP=imu

# ELF ビルド成果物クリーン
make nuttx-elf-clean
```

### 注意事項

- ELF 実行は必ずフルパスで指定する (例: `/data/imu`)
- `CONFIG_LIBC_ENVPATH` は有効にしない (NSH の PATH 検索で CPU オーバーヘッドが発生するため)
- ELF が使用するシンボルはカーネルのシンボルテーブル (`g_symtab`) に含まれている必要がある
- シンボルテーブルは `libc.csv` + `syscall.csv` + `libm.csv` + libgcc ヘルパーから生成
- ELF ロード時にコード + データが RAM に配置される (~10KB for imu)
