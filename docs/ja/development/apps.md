# アプリケーションの追加

カスタムアプリケーションは `apps/` ディレクトリに配置する。NuttX ビルドシステムの External Application として統合され、NSH のビルトインコマンドとして実行できる。

## ディレクトリ構成

```
apps/
├── Kconfig          # 自動生成: 各アプリの Kconfig を source
├── Make.defs        # 自動検出: external/*/Make.defs を include
├── Makefile         # External Applications メニュー
├── crash/           # アプリ例: crash test
│   ├── Kconfig
│   ├── Makefile
│   ├── Make.defs
│   └── crash_main.c
├── imu/             # アプリ例: IMU sensor fusion
│   ├── Kconfig
│   ├── Makefile
│   ├── Make.defs
│   └── imu_main.c
└── led/             # アプリ例: LED テスト (ボードドライバ API 使用)
    ├── Kconfig
    ├── Makefile
    ├── Make.defs
    └── led_main.c
```

## 必要ファイル

アプリ `myapp` を追加する場合、以下の 4 ファイルを `apps/myapp/` に作成する。

### Kconfig

```kconfig
config APP_MYAPP
	tristate "My application"
	default n
	---help---
		Description of the application.
```

- `CONFIG_APP_<NAME>` の命名規則で定義
- `tristate` で静的リンク / モジュール / 無効を選択可能

### Makefile

```makefile
include $(APPDIR)/Make.defs

PROGNAME = myapp
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_APP_MYAPP)

MAINSRC = myapp_main.c

include $(APPDIR)/Application.mk
```

- `PROGNAME` — NSH で実行するコマンド名
- `MODULE` — Kconfig の値に連動
- `MAINSRC` — エントリポイントを含むソースファイル
- 複数ファイルの場合は `CSRCS = file1.c file2.c` を追加

### Make.defs

```makefile
ifneq ($(CONFIG_APP_MYAPP),)
CONFIGURED_APPS += $(APPDIR)/external/myapp
endif
```

NuttX ビルドシステムが `apps/` を `$(APPDIR)/external/` として参照するため、パスは `$(APPDIR)/external/<name>` となる。

### エントリポイント

```c
int myapp_main(int argc, FAR char *argv[])
{
  /* Application code */
  return 0;
}
```

関数名は `<PROGNAME>_main` とする（NuttX の規約）。

## ビルドシステムへの登録

### apps/Kconfig

`apps/Kconfig` に source 行を追加:

```kconfig
menu "External Applications"
source "/path/to/apps/myapp/Kconfig"
endmenu
```

!!! note
    `apps/Kconfig` のパスは絶対パスで記述する。ビルド環境に依存するため、このファイルは `.gitignore` で管理してもよい（現状はリポジトリに含めている）。

### defconfig

`boards/spike-prime-hub/configs/usbnsh/defconfig` に追加:

```
CONFIG_APP_MYAPP=y
```

## ビルドシステムの仕組み

1. NuttX は `$(APPDIR)/external/` 以下の `Make.defs` を自動検出
2. `apps/Make.defs` が `$(APPDIR)/external/*/Make.defs` をワイルドカードで include
3. 各アプリの `Make.defs` が `CONFIGURED_APPS` に自身を登録
4. NuttX ビルドシステムが `CONFIGURED_APPS` のアプリをビルド・リンク

```
Makefile (top-level)
  └── nuttx/Makefile
       └── $(APPDIR)/Makefile        ← apps/Makefile
            └── $(APPDIR)/Make.defs   ← apps/Make.defs (wildcard include)
                 ├── external/crash/Make.defs
                 ├── external/imu/Make.defs
                 └── external/myapp/Make.defs
```

## ボードドライバ API の使用

アプリからボードドライバの関数（例: `tlc5955_set_duty()`）を呼ぶ場合、ヘッダをインクルードする:

```c
#include "spike_prime_hub.h"
```

ボードドライバはカーネルにリンクされているため、アプリから直接呼び出し可能（Flat ビルドモードの場合）。

アプリの `Makefile` にボードソースのインクルードパスを追加する:

```makefile
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)boards$(DELIM)spike-prime-hub$(DELIM)src
```

実例は `led` アプリを参照。
