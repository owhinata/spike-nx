using System.Buffers.Binary;
using System.Text;

using CaptureViewer.Core;

namespace CaptureViewer.Core.Tests.Fixtures;

/// <summary>
/// Synthesizes a valid .cap byte stream + the wrapping BT framing so
/// the parser tests don't need a connected Hub.  Mirrors what the
/// firmware actually emits — when the firmware-side layout shifts,
/// these helpers must be updated alongside the parser to keep the
/// tests honest.
/// </summary>
internal static class CaptureBuilder
{
    /// <summary>
    /// Build a .cap byte stream (file header + field descriptors +
    /// records) for the color_reflection_run schema.  Records carry
    /// monotonically increasing ts_us and a caller-supplied list of
    /// reflection percentages.
    /// </summary>
    public static byte[] BuildReflectionCap(
        ulong startTsUs,
        ReadOnlySpan<byte> reflectionPercents,
        uint tsStrideUs = 10_000)
    {
        var fields = new (string Name, FieldType Type, byte Offset, byte Size, sbyte Scale, string Unit)[]
        {
            ("ts_us", FieldType.U32, 0, 4, 0, "us"),
            ("reflection_pct", FieldType.U8, 4, 1, 0, "%"),
        };
        const int recordSize = 5;
        const string schemaName = "color_reflection_run";
        const ushort schemaMagic = 0x0010;

        var headerLen = CaptureProtocol.FileHeaderSize;
        var fdLen = fields.Length * CaptureProtocol.FieldDescriptorSize;
        var recCount = reflectionPercents.Length;
        var total = headerLen + fdLen + recCount * recordSize;
        var buf = new byte[total];

        // File header
        BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(0, 4), CaptureProtocol.FileMagic);
        BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(4, 2), CaptureProtocol.FileVersion);
        BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(6, 2), schemaMagic);
        BinaryPrimitives.WriteUInt64LittleEndian(buf.AsSpan(8, 8), startTsUs);
        BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(16, 4), recordSize);
        BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(20, 4), (uint)recCount);
        WriteFixed(buf.AsSpan(24, CaptureProtocol.SchemaNameMax), schemaName);
        buf[56] = (byte)fields.Length;
        // bytes 57..63 stay zero (reserved)

        // Field descriptors
        for (var i = 0; i < fields.Length; i++)
        {
            var off = headerLen + i * CaptureProtocol.FieldDescriptorSize;
            var (name, type, fOff, fSize, fScale, unit) = fields[i];
            WriteFixed(buf.AsSpan(off, CaptureProtocol.FieldNameMax), name);
            buf[off + CaptureProtocol.FieldNameMax + 0] = (byte)type;
            buf[off + CaptureProtocol.FieldNameMax + 1] = fOff;
            buf[off + CaptureProtocol.FieldNameMax + 2] = fSize;
            buf[off + CaptureProtocol.FieldNameMax + 3] = (byte)fScale;
            WriteFixed(buf.AsSpan(off + CaptureProtocol.FieldNameMax + 4,
                                  CaptureProtocol.FieldUnitMax), unit);
            // 12 reserved bytes stay zero
        }

        // Records
        for (var i = 0; i < recCount; i++)
        {
            var off = headerLen + fdLen + i * recordSize;
            BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(off, 4),
                                                    (uint)(i * tsStrideUs));
            buf[off + 4] = reflectionPercents[i];
        }

        return buf;
    }

    /// <summary>
    /// Wrap a .cap payload with BTCS + meta + (BTCE | BTAB).  The meta
    /// block carries the schema_magic / total_bytes / name expected by
    /// the host scanner.
    /// </summary>
    public static byte[] WrapBt(
        byte[] capPayload,
        ushort schemaMagic,
        string schemaName,
        ReadOnlySpan<byte> terminator)
    {
        var total = CaptureProtocol.BTCS.Length
                  + CaptureProtocol.SessionMetaSize
                  + capPayload.Length
                  + 4;
        var buf = new byte[total];
        var off = 0;

        CaptureProtocol.BTCS.CopyTo(buf.AsSpan(off, CaptureProtocol.BTCS.Length));
        off += CaptureProtocol.BTCS.Length;

        BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(off, 2), schemaMagic);
        BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(off + 2, 2), 0);  // reserved
        BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(off + 4, 4),
                                                (uint)capPayload.Length);
        WriteFixed(buf.AsSpan(off + 8, CaptureProtocol.SchemaNameMax), schemaName);
        off += CaptureProtocol.SessionMetaSize;

        capPayload.CopyTo(buf.AsSpan(off));
        off += capPayload.Length;

        terminator.CopyTo(buf.AsSpan(off, 4));
        return buf;
    }

    private static void WriteFixed(Span<byte> dst, string value)
    {
        dst.Clear();
        var encoded = Encoding.UTF8.GetBytes(value);
        if (encoded.Length > dst.Length)
            throw new ArgumentException(
                $"`{value}` does not fit in {dst.Length} bytes");
        encoded.CopyTo(dst);
    }
}
