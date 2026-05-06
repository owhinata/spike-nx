# NSH 起動スクリプト (`/etc/init.d/rcS`)

USB-NSH が起動した直後に自動実行されるスクリプト。`btsensor start` などの常駐デーモンを毎回手で叩かずに済ませる。

## 動作フロー

1. `CONFIG_ETC_ROMFS=y` のとき、`nuttx/sched/init/nx_bringup.c::nx_romfsetc()` がカーネル起動時に baked-in ROMFS を `/etc` にマウント
2. `nuttx-apps/nshlib/nsh_init.c::nsh_initialize()` が `BOARDCTL_INIT` / `BOARDCTL_FINALINIT` 完了後に `/etc/init.d/rc.sysinit` → `/etc/init.d/rcS` を実行 (それぞれ `nsh_sysinitscript()` / `nsh_initscript()`)
3. `nsh_initscript()` は static フラグ `g_nsh_script_initialized` で **idempotent**。複数の NSH セッションが並行起動しても rcS は 1 回しか走らない

## ビルド機構

`boards/Board.mk` (lines 21–42) が `RCSRCS` 変数を見つけると、以下を自動実行する:

1. C プリプロセッサで各 RC ファイルを処理 (`#include <nuttx/config.h>` / `#ifdef CONFIG_*` が使える)
2. `genromfs -V "NSHInitVol"` で ROMFS イメージを作る
3. `xxd -i romfs.img | sed "s/.../const unsigned char aligned_data(4)/"` で `etctmp.c` を生成
4. `etctmp.c` を libboard.a にリンクし、`nx_romfsetc()` がそれを `romdisk_register()` で `/dev/ram0` 経由で `/etc` にマウント

Docker ビルド環境には `genromfs` / `xxd` が同梱済 (`docker/Dockerfile.nuttx`)。

## SPIKE Prime Hub の構成

### `defconfig`

```
CONFIG_ETC_ROMFS=y
CONFIG_FS_ROMFS=y
```

`CONFIG_ETC_ROMFSMOUNTPT` (デフォルト `/etc`) と `CONFIG_ETC_ROMFSDEVNO` (デフォルト 0 → `/dev/ram0`) はそのまま使う。

### `boards/spike-prime-hub/src/Make.defs`

```make
ifeq ($(CONFIG_ETC_ROMFS),y)
RCSRCS = etc/init.d/rc.sysinit etc/init.d/rcS
endif
```

### `boards/spike-prime-hub/src/etc/init.d/rcS`

```c
#include <nuttx/config.h>

#ifdef CONFIG_APP_BTSENSOR
btsensor start
#endif
```

C プリプロセッサで処理されるので NSH に渡るのは `btsensor start` の 1 行のみ。

### `boards/spike-prime-hub/src/etc/init.d/rc.sysinit`

`nsh_sysinitscript()` が探しに行くため、空でも置いておく。FS マウントなど `rcS` より前に走らせたい処理を書きたくなったらここに追加する。

## 編集の流れ

1. `boards/spike-prime-hub/src/etc/init.d/rcS` を編集
2. `make` (差分ビルドで OK; `etctmp.c` だけ再生成される)
3. DFU で書き込み

## 制約

- スクリプトは BUILD_PROTECTED の userspace で実行されるので、起動するコマンドは Builtin App として userspace に登録されている必要がある (`apps/<name>/Makefile` の `MODULE = $(CONFIG_APP_<NAME>)` 形式)
- `nsh_initscript()` は idempotent なので、btnsh (BT NSH shell mode) 接続側では rcS は **再実行されない** ([apps/btsensor/btnsh_main.c](https://github.com/owhinata/spike-nx/blob/main/apps/btsensor/btnsh_main.c) は `nsh_session()` を直接呼んでおり、そもそも `nsh_initialize()` のコードパスを通らない)
- ROMFS は read-only。永続化が必要なら別途 SPIFFS / FAT を `/data` 等にマウントする
