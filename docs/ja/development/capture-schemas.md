# キャプチャスキーマリファレンス

`apps/capture` のスキーマは、`/dev/btcap` を流れる 1 record の **type-safe な layout 仕様** を定義する。Hub 側 C struct と PC 側 C# class が同じ source-of-truth (`.c` instance ファイル) から派生し、record_size / offset / 型タグが drift しないことを `_Static_assert` + Python codegen で保証する。

[キャプチャ運用ガイド](capture.md) も併せて参照。

## ソース構成

| パス | 内容 |
|---|---|
| `apps/capture/include/capture_format.h` | `.cap` ファイル先頭ヘッダ + 各フィールド descriptor (48 B 各) の wire layout、type tag enum |
| `apps/capture/include/capture_field.h` | C 型 token (`u8`/`i32`/`f64` 等) → C 型 / 数値タグ / バイトサイズの 3 種マッピング |
| `apps/capture/include/capture_schema_init.h` | `CAPTURE_FIELD_INIT(<schema>, <field>, <type_tok>, <unit>, <scale>)` ヘルパマクロ |
| `apps/capture/include/capture_schema_<name>.h` | スキーマ毎: `__attribute__((packed))` record struct + `_Static_assert` で `sizeof` 確認 |
| `apps/capture/schemas_<name>.c` | スキーマ毎: `g_capture_schema_<name>` const インスタンス (magic / rate_hz_hint / fields[]) |

## wire 上の record layout

各 record は **little-endian / packed**。fields は宣言順。先頭は `ts_us: u32` 必須 (`CLOCK_BOOTTIME` 下位 32-bit、capture 開始からの μs 経過)。

| 型 token | C 型 | C# 型 | 数値タグ | size |
|---|---|---|---:|---:|
| `u8`  | `uint8_t`  | `byte`   | 0 | 1 |
| `i8`  | `int8_t`   | `sbyte`  | 1 | 1 |
| `u16` | `uint16_t` | `ushort` | 2 | 2 |
| `i16` | `int16_t`  | `short`  | 3 | 2 |
| `u32` | `uint32_t` | `uint`   | 4 | 4 |
| `i32` | `int32_t`  | `int`    | 5 | 4 |
| `u64` | `uint64_t` | `ulong`  | 6 | 8 |
| `i64` | `int64_t`  | `long`   | 7 | 8 |
| `f32` | `float`    | `float`  | 8 | 4 |
| `f64` | `double`   | `double` | 9 | 8 |

## 既知スキーマ (codegen 出力)

<!-- 以下は `apps/capture/tools/gen_pc_schema.py --out-doc` の出力。手で編集しない -->

## `color_reflection_run` (magic 0x0010)

- record size: 5 bytes
- rate hint: 100 Hz

| offset | name | type | unit | scale 10^n |
|-------:|------|------|------|-----------:|
| 0 | `ts_us` | `u32` | `us` | 0 |
| 4 | `reflection_pct` | `u8` | `%` | 0 |

## `color_rgbi_run` (magic 0x0011)

- record size: 12 bytes
- rate hint: 100 Hz

| offset | name | type | unit | scale 10^n |
|-------:|------|------|------|-----------:|
| 0 | `ts_us` | `u32` | `us` | 0 |
| 4 | `red` | `u16` | `raw` | 0 |
| 6 | `green` | `u16` | `raw` | 0 |
| 8 | `blue` | `u16` | `raw` | 0 |
| 10 | `intensity` | `u16` | `raw` | 0 |

## `linetrace_lap_run` (magic 0x0012)

`linetrace` PID デーモンが走行中に 1 tick = 1 record でライントレースの
ラップを記録するスキーマ (Issue #166、親 #163 LQG ロードマップ)。
`linetrace cap arm/stop/export` で扱う。

- record size: 19 bytes
- rate hint: 100 Hz

| offset | name | type | unit | scale 10^n |
|-------:|------|------|------|-----------:|
| 0 | `ts_us` | `u32` | `us` | 0 |
| 4 | `intensity` | `u16` | `raw` | 0 |
| 6 | `target` | `u16` | `raw` | 0 |
| 8 | `turn_cmd_dps` | `i16` | `dps` | 0 |
| 10 | `heading_mdeg` | `i32` | `mdeg` | 0 |
| 14 | `turn_rate_dps` | `i16` | `dps` | 0 |
| 16 | `speed_mmps` | `i16` | `mmps` | 0 |
| 18 | `edge` | `u8` | `` | 0 |

`edge` バイトはライン上のどちらのエッジを走ったかを表す:

- `0` = **UNKNOWN/UNSET** — P0b は常にこの値を書く (PID デーモンに
  まだ edge の概念がないため)。`0` は予約済みのセンチネルで、有効な
  LEFT/RIGHT ではない。
- `1` = **LEFT** (P1a で設定)
- `2` = **RIGHT** (P1a で設定)

P0c のオフライン fitter は **`edge == 0` を実エッジとして解釈しては
ならない**。P0b キャプチャでは `c` の符号 (どちらのエッジか) はオペレータ
が帯域外で指定する。

## 新規スキーマの追加手順

例: 慣性センサ (IMU) 6-axis の生サンプル `imu_raw_run` を追加する場合。

### 1. Header — packed record struct

`apps/capture/include/capture_schema_imu_raw_run.h`:

```c
#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_IMU_RAW_RUN_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_IMU_RAW_RUN_H

#include <stddef.h>
#include <stdint.h>

#include "capture.h"

#ifdef __cplusplus
extern "C" {
#endif

struct capture_imu_raw_run_record_s
{
  uint32_t ts_us;
  int16_t  ax, ay, az;       /* accel raw (mg unit, scale_log10 = -3) */
  int16_t  gx, gy, gz;       /* gyro raw  (mdps unit)                  */
} __attribute__((packed));

_Static_assert(sizeof(struct capture_imu_raw_run_record_s) == 16,
               "capture_imu_raw_run_record_s wire size");
_Static_assert(offsetof(struct capture_imu_raw_run_record_s, ts_us) == 0,
               "ts_us must be the first field");

extern const struct capture_schema_s g_capture_schema_imu_raw_run;

#ifdef __cplusplus
}
#endif

#endif
```

### 2. C — schema instance

`apps/capture/schemas_imu_raw_run.c`:

```c
#include <nuttx/config.h>

#include "capture.h"
#include "capture_field.h"
#include "capture_format.h"
#include "capture_schema_imu_raw_run.h"
#include "capture_schema_init.h"

const struct capture_schema_s g_capture_schema_imu_raw_run =
{
  .magic        = 0x0020,
  .rate_hz_hint = 416,
  .record_size  = sizeof(struct capture_imu_raw_run_record_s),
  .field_count  = 7,
  .name         = "imu_raw_run",
  .fields       =
  {
    CAPTURE_FIELD_INIT(imu_raw_run, ts_us, u32, "us",   0),
    CAPTURE_FIELD_INIT(imu_raw_run, ax,    i16, "mg",  -3),
    CAPTURE_FIELD_INIT(imu_raw_run, ay,    i16, "mg",  -3),
    CAPTURE_FIELD_INIT(imu_raw_run, az,    i16, "mg",  -3),
    CAPTURE_FIELD_INIT(imu_raw_run, gx,    i16, "mdps", 0),
    CAPTURE_FIELD_INIT(imu_raw_run, gy,    i16, "mdps", 0),
    CAPTURE_FIELD_INIT(imu_raw_run, gz,    i16, "mdps", 0),
  },
};
```

ルール:

- **magic は global で unique**。既存スキーマと衝突したら codegen が `duplicate schema magic` で abort する。0x0010..0x001F は color sensor、0x0020..0x002F は IMU、のような領域分けを推奨。
- **第 1 フィールドは `ts_us: u32`**。`_Static_assert(offsetof(..., ts_us) == 0)` で強制。
- 型 token は表内のもののみ。`f128` などは X-macro 表に無いので追加できない。
- フィールド名 / 単位文字列は `[A-Za-z0-9_]` で 16 文字以内 (`CAPTURE_FIELD_NAME_MAX` / `CAPTURE_FIELD_UNIT_MAX`)。
- `record_size` / `field_count` は省略可 (codegen が `_Static_assert` で cross-check)。

### 3. Build 統合

`apps/capture/Makefile` の `CSRCS` に `schemas_imu_raw_run.c` を追加。Make.defs / Kconfig は変更不要。

### 4. apps/sensor の resolve table 登録

`apps/sensor/sensor_main.c`:

```c
#  include "capture_schema_imu_raw_run.h"
...
static const struct capture_schema_entry_s g_capture_schemas[] =
{
  { LEGOSENSOR_CLASS_COLOR, 1, &g_capture_schema_color_reflection_run },
  { LEGOSENSOR_CLASS_COLOR, 5, &g_capture_schema_color_rgbi_run        },
  { LEGOSENSOR_CLASS_IMU,   0, &g_capture_schema_imu_raw_run           },  /* new */
};
```

`convert_sample()` にも対応 case を追加 (uORB sample → packed record の詰め直し)。

### 5. PC 側 codegen 実行

```bash
python3 apps/capture/tools/gen_pc_schema.py \
    apps/capture/schemas_color_reflection_run.c \
    apps/capture/schemas_color_rgbi_run.c \
    apps/capture/schemas_imu_raw_run.c \
    --out-cs       host/CaptureViewer/src/CaptureViewer.Core/Generated \
    --out-registry host/CaptureViewer/src/CaptureViewer.Core/Generated/KnownSchemas.cs \
    --out-doc      docs/ja/development/capture-schemas-generated.md  # 任意
```

出力:

- `Schema_ImuRawRun.cs` — `ushort Magic` / `string Name` / `int RecordSize` / `int FieldCount` / `int RateHzHint` 定数 + typed `Record` struct + `static Parse(ReadOnlySpan<byte>)` パーサ
- `KnownSchemas.cs` — `Magic` 検索ディクショナリ更新

`dotnet test host/CaptureViewer/CaptureViewer.slnx` を再実行。`KnownSchemas` の test ケースを 3 件目に拡張するのを忘れない。

### 6. 動作確認

```
nsh> sensor imu select 0
nsh> sensor imu capture 1000 &
nsh> btsensor mode capture
```

PC 側で `BTCS + meta(name="imu_raw_run", magic=0x0020) + .cap + BTCE` が届けば OK。

## codegen 設計について

`gen_pc_schema.py` は `.c` インスタンスを **line-oriented パース** している (regex)。理由:

- v1 は 2 schema、`CAPTURE_FIELD_INIT(...)` の format が strict
- preprocessor 経由 (`CAPTURE_TOOL_GEN` ifdef) は schema 数が増えてフォーマットが揺れだしてから移行 (capture_field.h の comment に hook 記述あり)

Python 側で正当性チェック (型 token の whitelist、ts_us 第一強制、magic 重複検出) を行うので、source 側の typo は codegen 段階で検出される (`SystemExit`)。

## 関連 Issue / commit

- Issue #122 (キャプチャパイプライン親 Issue)
- `apps/capture/include/capture_format.h` (wire layout の正典)
- `apps/capture/tools/gen_pc_schema.py` (codegen 実装)
