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
3. ファイルパスを入力 (例: `/tmp/elf-build/imu`)
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
# 1. エクスポートパッケージの作成 (NuttX ビルド後)
make -f scripts/nuttx.mk export

# 2. エクスポートパッケージの展開
mkdir -p /tmp/elf-build && cd /tmp/elf-build
tar xzf /path/to/spike-nx/nuttx/nuttx-export-*.tar.gz
mv nuttx-export-* nuttx-export

# 3. Makefile 作成 (例: imu アプリ)
cat > Makefile << 'EOF'
include nuttx-export/scripts/Make.defs

ARCHCFLAGS += -mlong-calls
ARCHWARNINGS = -Wall -Wstrict-prototypes -Wshadow -Wundef
ARCHOPTIMIZATION = -Os -fno-strict-aliasing -fomit-frame-pointer
ARCHINCLUDES = -I. -isystem nuttx-export/include

CFLAGS = $(ARCHCFLAGS) $(ARCHWARNINGS) $(ARCHOPTIMIZATION) \
         $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES)

LDELFFLAGS = --relocatable -e main
LDELFFLAGS += -T nuttx-export/scripts/gnu-elf.ld

IMUDIR = /path/to/spike-nx/apps/imu

BIN = imu
SRCS = $(IMUDIR)/imu_main.c $(IMUDIR)/imu_geometry.c \
       $(IMUDIR)/imu_stationary.c $(IMUDIR)/imu_fusion.c \
       $(IMUDIR)/imu_calibration.c
OBJS = $(notdir $(SRCS:.c=$(OBJEXT)))

all: $(BIN)

%$(OBJEXT): $(IMUDIR)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

$(BIN): $(OBJS)
	$(LD) $(LDELFFLAGS) -o $@ $^
	$(STRIP) $@

clean:
	rm -f $(BIN) $(OBJS)
EOF

# 4. ビルド
make
```

### 転送と実行

```bash
# NSH で rz → Ctrl-A Ctrl-S → /tmp/elf-build/imu を選択
nsh> rz

# フルパスで実行
nsh> /data/imu status
nsh> /data/imu start
```

### 注意事項

- ELF はリロケータブル形式 (`--relocatable`) でビルドする
- `-mlong-calls` が必要 (RAM → Flash 間の関数呼び出しのため)
- ELF が使用するシンボルはカーネルのシンボルテーブル (`g_symtab`) に含まれている必要がある
- 現在のシンボルテーブルは `libs/libc/libc.csv` から生成 (標準 C ライブラリ関数)
- ELF ロード時に RAM を消費する (コード + データがRAM上に配置される)
- `CONFIG_LIBC_ENVPATH` は有効にしない (NSH の PATH 検索で CPU オーバーヘッドが発生するため)
- ELF 実行は必ずフルパスで指定する (例: `/data/imu`)
