namespace CaptureViewer.Core;

/// <summary>
/// Wire-level type tags.  Mirrors the CAPTURE_TYPE_* macros in
/// apps/capture/include/capture_format.h.  The numeric values are part
/// of the on-the-wire ABI — do not reorder.
/// </summary>
public enum FieldType : byte
{
    U8  = 0,
    I8  = 1,
    U16 = 2,
    I16 = 3,
    U32 = 4,
    I32 = 5,
    U64 = 6,
    I64 = 7,
    F32 = 8,
    F64 = 9,
}

/// <summary>
/// Describes one field inside a capture record.  Mirrors
/// `struct capture_field_desc_s` (48 bytes on the wire).  The C# side
/// only carries the semantic subset the viewer needs — the on-disk
/// layout is read once by <see cref="Capture.CaptureFile"/> and not
/// stored in this object.
/// </summary>
public sealed record FieldDescriptor(
    string Name,
    FieldType Type,
    int Offset,
    int Size,
    int ScaleLog10,
    string Unit);

/// <summary>
/// Aggregate view of one schema known to the host.  Created by the
/// generated <c>KnownSchemas</c> registry; the viewer uses this for
/// magic-number lookup before resolving the typed parser.
/// </summary>
public sealed record SchemaInfo(
    ushort Magic,
    string Name,
    int RecordSize,
    int FieldCount,
    IReadOnlyList<FieldDescriptor> Fields);
