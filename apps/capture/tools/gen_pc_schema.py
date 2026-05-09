#!/usr/bin/env python3
"""Generate per-schema C# parsers from apps/capture schema instance .c files.

The host-side CaptureViewer needs typed access to every schema the
firmware can emit (`color_reflection_run`, `color_rgbi_run`, ...).
Hand-writing one C# class per schema would drift from the C source the
moment a field is added or renamed.  This tool reads the .c files that
contain the `g_capture_schema_<name>` const initializer, extracts the
schema metadata + the per-field descriptors, and emits a C# class with:

    * Wire-format constants (Magic, Name, RecordSize, FieldCount).
    * A typed record struct with one field per CAPTURE_FIELD_INIT entry
      (correct C# type derived from the u8/i8/.../f64 token).
    * A static `Parse(ReadOnlySpan<byte>)` method that uses
      `BinaryPrimitives` for endian-safe little-endian decoding.
    * A static field descriptor list (name / unit / scale_log10) that
      mirrors the firmware's `capture_field_desc_s` array.

Inputs (positional): one or more .c files each containing exactly one
schema initializer.

Outputs (CLI flags):
    --out-cs DIR       — emit `Schema_<PascalName>.cs` files here.
    --out-registry CS  — emit a registry `KnownSchemas.cs` indexing
                          all generated schemas by `Magic`.
    --out-doc PATH     — emit a Markdown table for
                          docs/{ja,en}/development/capture-schemas.md
                          (one section per schema).

The deliberate non-goal is full preprocessor fidelity.  v1 ships with
two schemas in a strict format; the parser does line-oriented matching
on the curly-brace layout the existing files already follow.  When a
new schema is added it must follow the same layout (template at the
top of `apps/capture/schemas_color_reflection_run.c`).
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import re
import sys
from typing import Iterable, Sequence


# ---------------------------------------------------------------------------
# Wire-format token tables.  Mirror apps/capture/include/capture_field.h
# (CAPTURE_TYPE_C_/TAG_/SZ_) and capture_format.h (CAPTURE_TYPE_*).
# ---------------------------------------------------------------------------

# token -> (C# type name, tag id, byte size, BinaryPrimitives reader expr).
# `expr` reads from `slice` (a ReadOnlySpan<byte>) at offset 0.
_TYPE_TABLE: dict[str, tuple[str, int, int, str]] = {
    "u8":  ("byte",   0, 1, "slice[0]"),
    "i8":  ("sbyte",  1, 1, "(sbyte)slice[0]"),
    "u16": ("ushort", 2, 2, "BinaryPrimitives.ReadUInt16LittleEndian(slice)"),
    "i16": ("short",  3, 2, "BinaryPrimitives.ReadInt16LittleEndian(slice)"),
    "u32": ("uint",   4, 4, "BinaryPrimitives.ReadUInt32LittleEndian(slice)"),
    "i32": ("int",    5, 4, "BinaryPrimitives.ReadInt32LittleEndian(slice)"),
    "u64": ("ulong",  6, 8, "BinaryPrimitives.ReadUInt64LittleEndian(slice)"),
    "i64": ("long",   7, 8, "BinaryPrimitives.ReadInt64LittleEndian(slice)"),
    "f32": ("float",  8, 4, "BinaryPrimitives.ReadSingleLittleEndian(slice)"),
    "f64": ("double", 9, 8, "BinaryPrimitives.ReadDoubleLittleEndian(slice)"),
}


# ---------------------------------------------------------------------------
# Parsed schema model
# ---------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class Field:
    name: str
    type_token: str
    unit: str
    scale_log10: int
    offset: int  # derived

    @property
    def cs_type(self) -> str:
        return _TYPE_TABLE[self.type_token][0]

    @property
    def tag(self) -> int:
        return _TYPE_TABLE[self.type_token][1]

    @property
    def size(self) -> int:
        return _TYPE_TABLE[self.type_token][2]

    @property
    def reader_expr(self) -> str:
        return _TYPE_TABLE[self.type_token][3]


@dataclasses.dataclass(frozen=True)
class Schema:
    name: str            # snake_case (matches schema_name on the wire)
    magic: int
    rate_hz_hint: int
    fields: tuple[Field, ...]

    @property
    def record_size(self) -> int:
        return sum(f.size for f in self.fields)

    @property
    def pascal_name(self) -> str:
        return "".join(part.capitalize() for part in self.name.split("_"))


# ---------------------------------------------------------------------------
# .c parser.  Hand-rolled because the input format is small and stable.
# Bails loudly on anything it does not recognize so silent drift is
# impossible.
# ---------------------------------------------------------------------------

_RE_SCHEMA_HEADER = re.compile(
    r"const\s+struct\s+capture_schema_s\s+"
    r"g_capture_schema_(?P<name>[a-z0-9_]+)\s*=\s*\{",
    re.MULTILINE,
)

_RE_INT_FIELD = re.compile(
    r"\.\s*(?P<key>magic|rate_hz_hint|record_size|field_count)\s*=\s*"
    r"(?P<val>0[xX][0-9a-fA-F]+|\d+)",
)

_RE_NAME_FIELD = re.compile(
    r"\.\s*name\s*=\s*\"(?P<val>[A-Za-z0-9_]+)\"",
)

_RE_FIELD_INIT = re.compile(
    r"CAPTURE_FIELD_INIT\(\s*"
    r"(?P<schema>[a-z0-9_]+)\s*,\s*"
    r"(?P<name>[a-z0-9_]+)\s*,\s*"
    r"(?P<type>[uif]\d{1,2})\s*,\s*"
    r"\"(?P<unit>[^\"]*)\"\s*,\s*"
    r"(?P<scale>-?\d+)\s*\)",
)


def _strip_c_comments(src: str) -> str:
    """Drop /* ... */ and // ... comments so the regexes don't trip on
    sample CAPTURE_FIELD_INIT calls in the file-level docstring.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.DOTALL)
    src = re.sub(r"//[^\n]*", "", src)
    return src


def parse_schema_file(path: str) -> Schema:
    with open(path, encoding="utf-8") as f:
        raw = f.read()
    text = _strip_c_comments(raw)

    m = _RE_SCHEMA_HEADER.search(text)
    if not m:
        raise SystemExit(
            f"{path}: no `const struct capture_schema_s g_capture_schema_<name>` "
            f"initializer found"
        )
    name = m.group("name")

    # Pull metadata fields.  Required: magic, rate_hz_hint.  Optional:
    # record_size + field_count are auto-derived from the field list,
    # but if present we cross-check.
    meta: dict[str, int] = {}
    for im in _RE_INT_FIELD.finditer(text):
        meta[im.group("key")] = int(im.group("val"), 0)

    if "magic" not in meta:
        raise SystemExit(f"{path}: schema is missing `.magic`")
    if "rate_hz_hint" not in meta:
        raise SystemExit(f"{path}: schema is missing `.rate_hz_hint`")

    name_match = _RE_NAME_FIELD.search(text)
    if not name_match:
        raise SystemExit(f"{path}: schema is missing `.name`")
    if name_match.group("val") != name:
        raise SystemExit(
            f"{path}: schema instance name mismatch "
            f"(symbol g_capture_schema_{name} vs .name = "
            f"\"{name_match.group('val')}\")"
        )

    # Fields, in declared order.
    fields: list[Field] = []
    offset = 0
    for fm in _RE_FIELD_INIT.finditer(text):
        if fm.group("schema") != name:
            raise SystemExit(
                f"{path}: CAPTURE_FIELD_INIT names schema "
                f"`{fm.group('schema')}` but file defines `{name}`"
            )
        ttok = fm.group("type")
        if ttok not in _TYPE_TABLE:
            raise SystemExit(f"{path}: unknown type token `{ttok}`")
        field = Field(
            name=fm.group("name"),
            type_token=ttok,
            unit=fm.group("unit"),
            scale_log10=int(fm.group("scale")),
            offset=offset,
        )
        offset += field.size
        fields.append(field)

    if not fields:
        raise SystemExit(f"{path}: schema declares no CAPTURE_FIELD_INIT entries")

    if fields[0].name != "ts_us" or fields[0].type_token != "u32":
        raise SystemExit(
            f"{path}: first field must be `ts_us: u32` "
            f"(got `{fields[0].name}: {fields[0].type_token}`)"
        )

    schema = Schema(
        name=name,
        magic=meta["magic"],
        rate_hz_hint=meta["rate_hz_hint"],
        fields=tuple(fields),
    )

    # Cross-check optional declared sizes.
    if "record_size" in meta and meta["record_size"] != schema.record_size:
        raise SystemExit(
            f"{path}: declared record_size={meta['record_size']} != "
            f"derived {schema.record_size}"
        )
    if "field_count" in meta and meta["field_count"] != len(schema.fields):
        raise SystemExit(
            f"{path}: declared field_count={meta['field_count']} != "
            f"derived {len(schema.fields)}"
        )

    return schema


# ---------------------------------------------------------------------------
# C# emitter
# ---------------------------------------------------------------------------

_CS_HEADER = """\
// <auto-generated>
//   This file is generated by apps/capture/tools/gen_pc_schema.py.
//   Do not edit by hand — re-run the tool after modifying the C source.
//
//   Source schema: {src}
// </auto-generated>
#nullable enable
using System;
using System.Buffers.Binary;
using System.Collections.Generic;

namespace CaptureViewer.Core.Generated;
"""


def _emit_schema_cs(schema: Schema, src_path: str) -> str:
    pn = schema.pascal_name

    # Each pre-formatted block is emitted with the indentation it needs
    # at the final substitution site; the surrounding braces use bare
    # f-string interpolation so we don't fight textwrap.dedent.
    field_lines = "\n".join(
        f"        public {f.cs_type} {f.name};" for f in schema.fields
    )

    parser_lines = "\n".join(
        f"        {{ var slice = data.Slice({f.offset}, {f.size}); "
        f"r.{f.name} = {f.reader_expr}; }}"
        for f in schema.fields
    )

    descriptor_lines = "\n".join(
        f"        new FieldDescriptor("
        f"\"{f.name}\", "
        f"FieldType.{f.type_token.upper()}, "
        f"{f.offset}, {f.size}, "
        f"{f.scale_log10}, "
        f"\"{f.unit}\"),"
        for f in schema.fields
    )

    body = (
f"""public static class Schema{pn}
{{
    public const ushort Magic        = 0x{schema.magic:04X};
    public const string Name         = "{schema.name}";
    public const int    RecordSize   = {schema.record_size};
    public const int    FieldCount   = {len(schema.fields)};
    public const int    RateHzHint   = {schema.rate_hz_hint};

    public static readonly IReadOnlyList<FieldDescriptor> Fields = new[]
    {{
{descriptor_lines}
    }};

    public static Record Parse(ReadOnlySpan<byte> data)
    {{
        if (data.Length < RecordSize)
            throw new ArgumentException(
                $"Schema{pn}.Parse needs {{RecordSize}} bytes, got {{data.Length}}",
                nameof(data));

        var r = new Record();
{parser_lines}
        return r;
    }}

    public struct Record
    {{
{field_lines}
    }}
}}
"""
    )

    return _CS_HEADER.format(src=os.path.relpath(src_path)) + "\n" + body


def _emit_registry_cs(schemas: Sequence[Schema]) -> str:
    entry_lines = "\n".join(
        f"        {{ Schema{s.pascal_name}.Magic, new SchemaInfo("
        f"Schema{s.pascal_name}.Magic, "
        f"Schema{s.pascal_name}.Name, "
        f"Schema{s.pascal_name}.RecordSize, "
        f"Schema{s.pascal_name}.FieldCount, "
        f"Schema{s.pascal_name}.Fields) }},"
        for s in schemas
    )
    body = (
f"""public static class KnownSchemas
{{
    public static readonly IReadOnlyDictionary<ushort, SchemaInfo> ByMagic =
        new Dictionary<ushort, SchemaInfo>
    {{
{entry_lines}
    }};

    public static SchemaInfo? TryGet(ushort magic) =>
        ByMagic.TryGetValue(magic, out var info) ? info : null;
}}
"""
    )
    return (
        "// <auto-generated>\n"
        "//   This file is generated by apps/capture/tools/gen_pc_schema.py.\n"
        "// </auto-generated>\n"
        "#nullable enable\n"
        "using System.Collections.Generic;\n"
        "\n"
        "namespace CaptureViewer.Core.Generated;\n"
        "\n"
        + body
    )


def _emit_doc(schemas: Iterable[Schema]) -> str:
    lines: list[str] = []
    lines.append("<!-- generated by apps/capture/tools/gen_pc_schema.py — do not edit -->")
    lines.append("")
    for s in schemas:
        lines.append(f"## `{s.name}` (magic 0x{s.magic:04X})")
        lines.append("")
        lines.append(f"- record size: {s.record_size} bytes")
        lines.append(f"- rate hint: {s.rate_hz_hint} Hz")
        lines.append("")
        lines.append("| offset | name | type | unit | scale 10^n |")
        lines.append("|-------:|------|------|------|-----------:|")
        for f in s.fields:
            lines.append(
                f"| {f.offset} | `{f.name}` | `{f.type_token}` | "
                f"`{f.unit}` | {f.scale_log10} |"
            )
        lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sources", nargs="+",
                    help="schema instance .c files")
    ap.add_argument("--out-cs",
                    help="emit per-schema Schema_<Name>.cs into this dir")
    ap.add_argument("--out-registry",
                    help="emit a KnownSchemas.cs registry to this path")
    ap.add_argument("--out-doc",
                    help="emit a Markdown schema table to this path")
    args = ap.parse_args(argv)

    schemas = [parse_schema_file(p) for p in args.sources]
    schemas.sort(key=lambda s: s.magic)

    # Sanity: magic values must be unique across the input set.
    seen: dict[int, str] = {}
    for s in schemas:
        if s.magic in seen:
            raise SystemExit(
                f"duplicate schema magic 0x{s.magic:04X}: "
                f"{seen[s.magic]} vs {s.name}"
            )
        seen[s.magic] = s.name

    if args.out_cs:
        os.makedirs(args.out_cs, exist_ok=True)
        for s, src in zip(schemas, args.sources):
            out = os.path.join(args.out_cs, f"Schema_{s.pascal_name}.cs")
            with open(out, "w", encoding="utf-8") as f:
                f.write(_emit_schema_cs(s, src))
            print(f"wrote {out}", file=sys.stderr)

    if args.out_registry:
        os.makedirs(os.path.dirname(args.out_registry) or ".", exist_ok=True)
        with open(args.out_registry, "w", encoding="utf-8") as f:
            f.write(_emit_registry_cs(schemas))
        print(f"wrote {args.out_registry}", file=sys.stderr)

    if args.out_doc:
        os.makedirs(os.path.dirname(args.out_doc) or ".", exist_ok=True)
        with open(args.out_doc, "w", encoding="utf-8") as f:
            f.write(_emit_doc(schemas))
        print(f"wrote {args.out_doc}", file=sys.stderr)

    if not (args.out_cs or args.out_registry or args.out_doc):
        # No outputs requested — just print a summary.
        for s in schemas:
            print(f"{s.name:30s} magic=0x{s.magic:04X} "
                  f"rec={s.record_size}B fields={len(s.fields)}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
