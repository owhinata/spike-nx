using System.Buffers.Binary;

namespace CaptureViewer.Core.Capture;

/// <summary>
/// Generic record-field accessor.  Given a record byte slice and a
/// <see cref="FieldDescriptor"/>, returns the field's value as a
/// <see cref="double"/> regardless of underlying wire type.
///
/// This trades the type fidelity of the codegen-emitted typed Parse()
/// for a uniform numeric API the plot / CSV layers can consume without
/// switching on schema magic.
/// </summary>
public static class FieldReader
{
    /// <summary>
    /// Decode <paramref name="field"/> from <paramref name="record"/>
    /// as a raw double, ignoring any <c>scale_log10</c>.
    /// </summary>
    public static double ReadRawDouble(
        ReadOnlySpan<byte> record,
        FieldDescriptor field)
    {
        var slice = record.Slice(field.Offset, field.Size);
        return field.Type switch
        {
            FieldType.U8  => slice[0],
            FieldType.I8  => (sbyte)slice[0],
            FieldType.U16 => BinaryPrimitives.ReadUInt16LittleEndian(slice),
            FieldType.I16 => BinaryPrimitives.ReadInt16LittleEndian(slice),
            FieldType.U32 => BinaryPrimitives.ReadUInt32LittleEndian(slice),
            FieldType.I32 => BinaryPrimitives.ReadInt32LittleEndian(slice),
            FieldType.U64 => BinaryPrimitives.ReadUInt64LittleEndian(slice),
            FieldType.I64 => BinaryPrimitives.ReadInt64LittleEndian(slice),
            FieldType.F32 => BinaryPrimitives.ReadSingleLittleEndian(slice),
            FieldType.F64 => BinaryPrimitives.ReadDoubleLittleEndian(slice),
            _ => throw new ArgumentException(
                     $"Unsupported field type {field.Type}", nameof(field)),
        };
    }

    /// <summary>
    /// Decode the field as a double scaled by 10^<c>scale_log10</c>
    /// (so values land in their declared unit).  Identity for
    /// <c>scale_log10 == 0</c>.
    /// </summary>
    public static double ReadScaledDouble(
        ReadOnlySpan<byte> record,
        FieldDescriptor field)
    {
        var raw = ReadRawDouble(record, field);
        return field.ScaleLog10 == 0 ? raw : raw * Math.Pow(10.0, field.ScaleLog10);
    }

    /// <summary>
    /// Pull all values of a named field across every record of a
    /// capture, scaled per <c>scale_log10</c>.  Returns a fresh array
    /// the caller can hand straight to a plotter.
    /// </summary>
    public static double[] ReadColumn(CaptureFile capture, string fieldName)
    {
        ArgumentNullException.ThrowIfNull(capture);
        ArgumentException.ThrowIfNullOrEmpty(fieldName);

        FieldDescriptor? match = null;
        foreach (var f in capture.Fields)
        {
            if (f.Name == fieldName) { match = f; break; }
        }
        if (match is null)
            throw new ArgumentException(
                $"Field `{fieldName}` not present in schema `{capture.SchemaName}`",
                nameof(fieldName));

        var col = new double[capture.RecordCount];
        for (var i = 0; i < capture.RecordCount; i++)
        {
            col[i] = ReadScaledDouble(capture.Records(i).Span, match);
        }
        return col;
    }
}
