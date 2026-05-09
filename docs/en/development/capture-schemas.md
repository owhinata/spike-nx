# Capture schema reference

A capture schema defines the **type-safe layout of one record** on
the `/dev/btcap` wire.  The Hub-side packed C struct and the host-side
C# parser both derive from a single source-of-truth (`schemas_*.c`),
and `_Static_assert` + Python codegen guarantee that `record_size`,
field offsets, and type tags cannot drift between them.

See also the [capture operation guide](capture.md).

## Source layout

| Path | Contents |
|---|---|
| `apps/capture/include/capture_format.h` | `.cap` file header + per-field descriptor (48 B each) wire layout, type-tag enum |
| `apps/capture/include/capture_field.h` | Token (`u8`/`i32`/`f64` ...) → C type / numeric tag / byte size mappings |
| `apps/capture/include/capture_schema_init.h` | `CAPTURE_FIELD_INIT(<schema>, <field>, <type_tok>, <unit>, <scale>)` helper macro |
| `apps/capture/include/capture_schema_<name>.h` | Per-schema: `__attribute__((packed))` record struct + `_Static_assert` size lock |
| `apps/capture/schemas_<name>.c` | Per-schema: `g_capture_schema_<name>` const instance (magic / rate_hz_hint / fields[]) |

## On-the-wire record layout

Every record is **little-endian / packed**, fields appear in declared
order, and the first field must be `ts_us: u32` (`CLOCK_BOOTTIME`'s
low 32 bits, microseconds since the start of the capture).

| Token | C type | C# type | Numeric tag | Size |
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

## Registered schemas (codegen output)

<!-- Section below is the output of `apps/capture/tools/gen_pc_schema.py --out-doc`.  Do not hand-edit. -->

## `color_reflection_run` (magic 0x0010)

- record size: 9 bytes
- rate hint: 100 Hz

| offset | name | type | unit | scale 10^n |
|-------:|------|------|------|-----------:|
| 0 | `ts_us` | `u32` | `us` | 0 |
| 4 | `distance_mm` | `i32` | `mm` | 0 |
| 8 | `reflection_pct` | `u8` | `%` | 0 |

## `color_rgbi_run` (magic 0x0011)

- record size: 12 bytes
- rate hint: 100 Hz

| offset | name | type | unit | scale 10^n |
|-------:|------|------|------|-----------:|
| 0 | `ts_us` | `u32` | `us` | 0 |
| 4 | `distance_mm` | `i32` | `mm` | 0 |
| 8 | `red` | `u8` | `raw` | 0 |
| 9 | `green` | `u8` | `raw` | 0 |
| 10 | `blue` | `u8` | `raw` | 0 |
| 11 | `intensity` | `u8` | `raw` | 0 |

`distance_mm` is reserved (always 0 in v1 — slated for cross-correlation
with drivebase encoder state in a future revision).

## Adding a new schema

Worked example: a 6-axis IMU raw schema `imu_raw_run`.

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
  int16_t  ax, ay, az;       /* accel raw (mg, scale_log10 = -3) */
  int16_t  gx, gy, gz;       /* gyro  raw (mdps)                 */
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

Rules:

- **`magic` is globally unique.**  Collisions are caught at codegen
  with `duplicate schema magic`.  Suggested ranges: `0x0010..0x001F`
  for the Color sensor, `0x0020..0x002F` for the IMU, etc.
- **First field must be `ts_us: u32`** — enforced by
  `_Static_assert(offsetof(..., ts_us) == 0)`.
- Only the type tokens listed above are accepted.  `f128` etc. are
  not in the X-macro table.
- Field name and unit must be `[A-Za-z0-9_]` and ≤ 16 bytes
  (`CAPTURE_FIELD_NAME_MAX` / `CAPTURE_FIELD_UNIT_MAX`).
- `record_size` and `field_count` are optional in the initializer
  (codegen cross-checks them with `_Static_assert`).

### 3. Build wiring

Add `schemas_imu_raw_run.c` to `CSRCS` in `apps/capture/Makefile`.
No `Make.defs` / `Kconfig` change needed.

### 4. Register on the resolve table

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

Add a matching case to `convert_sample()` (uORB sample → packed
record translation).

### 5. Run the host codegen

```bash
python3 apps/capture/tools/gen_pc_schema.py \
    apps/capture/schemas_color_reflection_run.c \
    apps/capture/schemas_color_rgbi_run.c \
    apps/capture/schemas_imu_raw_run.c \
    --out-cs       host/CaptureViewer/src/CaptureViewer.Core/Generated \
    --out-registry host/CaptureViewer/src/CaptureViewer.Core/Generated/KnownSchemas.cs \
    --out-doc      docs/en/development/capture-schemas-generated.md  # optional
```

Outputs:

- `Schema_ImuRawRun.cs` — `Magic` / `Name` / `RecordSize` /
  `FieldCount` / `RateHzHint` constants + a typed `Record` struct +
  `static Parse(ReadOnlySpan<byte>)`
- `KnownSchemas.cs` — magic-lookup dictionary updated

Re-run `dotnet test host/CaptureViewer/CaptureViewer.slnx`.  Don't
forget to extend the `KnownSchemas` test case to assert the third
entry.

### 6. End-to-end smoke

```
nsh> sensor imu select 0
nsh> sensor imu capture 1000 &
nsh> btsensor mode capture
```

If `BTCS + meta(name="imu_raw_run", magic=0x0020) + .cap + BTCE` lands
on the host, the schema is wired correctly.

## Codegen design notes

`gen_pc_schema.py` parses the `.c` instance file with **line-oriented
regex matching**.  The motivation:

- v1 ships with two schemas in a strict format.
- The preprocessor-driven path (a `CAPTURE_TOOL_GEN` ifdef in
  `capture_field.h`) is the planned migration once the schema set
  grows enough that hand-rolled regexes start to slip.  The hook
  point is already in `capture_field.h` for that.

Validity checks live on the Python side: type-token whitelist, the
`ts_us` first-field requirement, and global magic-uniqueness.  Source
typos produce a `SystemExit` from the codegen step rather than a
silent miscompile.

## Related issues / commits

- Issue #122 (parent Issue for the capture pipeline)
- `apps/capture/include/capture_format.h` (canonical wire layout)
- `apps/capture/tools/gen_pc_schema.py` (codegen implementation)
