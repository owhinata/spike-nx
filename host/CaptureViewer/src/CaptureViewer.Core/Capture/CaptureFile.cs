using System.Buffers.Binary;
using System.Text;

namespace CaptureViewer.Core.Capture;

/// <summary>
/// Disposition of a session's terminator on the BT stream.
/// </summary>
public enum CaptureTermination
{
    /// <summary>BTCE — writer FINALIZE'd cleanly, payload is complete.</summary>
    Clean,

    /// <summary>BTAB — writer ABORT or BT side detected truncation.</summary>
    Aborted,

    /// <summary>The .cap file does not encode a terminator (file-only path).</summary>
    Unknown,
}

/// <summary>
/// In-memory representation of one capture session.  Holds the parsed
/// file header, the field descriptors, and the raw record payload.
/// The <see cref="Records"/> property exposes records as a byte slice
/// each so callers can run them through a generated typed parser.
/// </summary>
public sealed class CaptureFile
{
    public uint Magic { get; }
    public ushort Version { get; }
    public ushort SchemaMagic { get; }
    public ulong StartTimestampUs { get; }
    public int RecordSize { get; }
    public int RecordCount { get; }
    public string SchemaName { get; }
    public IReadOnlyList<FieldDescriptor> Fields { get; }
    public ReadOnlyMemory<byte> Payload { get; }
    public CaptureTermination Termination { get; }

    private CaptureFile(
        uint magic,
        ushort version,
        ushort schemaMagic,
        ulong startTsUs,
        int recordSize,
        int recordCount,
        string schemaName,
        IReadOnlyList<FieldDescriptor> fields,
        ReadOnlyMemory<byte> payload,
        CaptureTermination termination)
    {
        Magic = magic;
        Version = version;
        SchemaMagic = schemaMagic;
        StartTimestampUs = startTsUs;
        RecordSize = recordSize;
        RecordCount = recordCount;
        SchemaName = schemaName;
        Fields = fields;
        Payload = payload;
        Termination = termination;
    }

    /// <summary>
    /// Returns the byte slice for the n-th record.  No allocation —
    /// the slice points into <see cref="Payload"/>.
    /// </summary>
    public ReadOnlyMemory<byte> Records(int index)
    {
        if ((uint)index >= (uint)RecordCount)
            throw new ArgumentOutOfRangeException(nameof(index));
        var off = RecordsOffset + index * RecordSize;
        return Payload.Slice(off, RecordSize);
    }

    /// <summary>
    /// Byte offset inside <see cref="Payload"/> where the first record
    /// starts.  Equals 64-byte file header + N field descriptors.
    /// </summary>
    public int RecordsOffset =>
        CaptureProtocol.FileHeaderSize + Fields.Count * CaptureProtocol.FieldDescriptorSize;

    /// <summary>
    /// Parse a .cap byte buffer.  The buffer is the *full* on-disk
    /// payload starting at the "CAPB" magic, i.e. file header + field
    /// descriptors + records.  No BTCS/BTCE framing here — for live BT
    /// receive use <see cref="SessionScanner"/> first.
    /// </summary>
    /// <param name="bytes">The raw .cap content.</param>
    /// <param name="termination">Optional BT-level termination hint;
    /// defaults to <see cref="CaptureTermination.Unknown"/> for plain
    /// file reads.</param>
    public static CaptureFile Parse(
        ReadOnlySpan<byte> bytes,
        CaptureTermination termination = CaptureTermination.Unknown)
    {
        if (bytes.Length < CaptureProtocol.FileHeaderSize)
            throw new InvalidCaptureFileException(
                $"Buffer too small: {bytes.Length} < {CaptureProtocol.FileHeaderSize} bytes");

        var magic = BinaryPrimitives.ReadUInt32LittleEndian(bytes[..4]);
        if (magic != CaptureProtocol.FileMagic)
            throw new InvalidCaptureFileException(
                $"Bad magic: 0x{magic:X8} (want 0x{CaptureProtocol.FileMagic:X8} \"CAPB\")");

        var version = BinaryPrimitives.ReadUInt16LittleEndian(bytes.Slice(4, 2));
        if (version != CaptureProtocol.FileVersion)
            throw new InvalidCaptureFileException(
                $"Unsupported version: {version} (want {CaptureProtocol.FileVersion})");

        var schemaMagic = BinaryPrimitives.ReadUInt16LittleEndian(bytes.Slice(6, 2));
        var startTsUs = BinaryPrimitives.ReadUInt64LittleEndian(bytes.Slice(8, 8));
        var recordSize = (int)BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(16, 4));
        var recordCount = (int)BinaryPrimitives.ReadUInt32LittleEndian(bytes.Slice(20, 4));

        if (recordSize <= 0 || recordSize > 255)
            throw new InvalidCaptureFileException($"Implausible record_size {recordSize}");
        if (recordCount < 0)
            throw new InvalidCaptureFileException($"Negative record_count {recordCount}");

        var schemaName = ReadFixedString(bytes.Slice(24, CaptureProtocol.SchemaNameMax));
        var fieldCount = bytes[56];
        if (fieldCount == 0 || fieldCount > 32)
            throw new InvalidCaptureFileException($"Implausible field_count {fieldCount}");

        var expectedSize =
            CaptureProtocol.FileHeaderSize
            + fieldCount * CaptureProtocol.FieldDescriptorSize
            + (long)recordSize * recordCount;
        if (bytes.Length < expectedSize)
            throw new InvalidCaptureFileException(
                $"Truncated capture: have {bytes.Length} bytes, header expects {expectedSize}");

        var fields = new List<FieldDescriptor>(fieldCount);
        var fdStart = CaptureProtocol.FileHeaderSize;
        for (var i = 0; i < fieldCount; i++)
        {
            var off = fdStart + i * CaptureProtocol.FieldDescriptorSize;
            fields.Add(ReadFieldDescriptor(bytes.Slice(off, CaptureProtocol.FieldDescriptorSize)));
        }

        // Sanity: the cumulative offset+size of the last field must
        // fit inside record_size.  Rejects mangled descriptor blocks.
        foreach (var f in fields)
        {
            if (f.Offset + f.Size > recordSize)
                throw new InvalidCaptureFileException(
                    $"Field `{f.Name}` extends past record_size " +
                    $"(offset={f.Offset} size={f.Size} record_size={recordSize})");
        }

        // Slice the field descriptors + records into Payload.  We hold
        // a copy to keep the lifetime detached from the caller's span.
        var payload = bytes[..(int)expectedSize].ToArray();

        return new CaptureFile(
            magic, version, schemaMagic, startTsUs,
            recordSize, recordCount, schemaName,
            fields, payload, termination);
    }

    /// <summary>Read a .cap file from disk.</summary>
    public static CaptureFile Open(string path)
    {
        var bytes = File.ReadAllBytes(path);
        return Parse(bytes);
    }

    private static FieldDescriptor ReadFieldDescriptor(ReadOnlySpan<byte> b)
    {
        var name = ReadFixedString(b[..CaptureProtocol.FieldNameMax]);
        var type = (FieldType)b[CaptureProtocol.FieldNameMax];
        var offset = b[CaptureProtocol.FieldNameMax + 1];
        var size = b[CaptureProtocol.FieldNameMax + 2];
        var scale = (sbyte)b[CaptureProtocol.FieldNameMax + 3];
        var unit = ReadFixedString(b.Slice(CaptureProtocol.FieldNameMax + 4,
                                           CaptureProtocol.FieldUnitMax));
        return new FieldDescriptor(name, type, offset, size, scale, unit);
    }

    /// <summary>
    /// Read a null-padded fixed-width ASCII/UTF-8 string.  Stops at
    /// the first NUL byte; the rest is treated as padding.
    /// </summary>
    internal static string ReadFixedString(ReadOnlySpan<byte> raw)
    {
        var nulIdx = raw.IndexOf((byte)0);
        var slice = nulIdx >= 0 ? raw[..nulIdx] : raw;
        return Encoding.UTF8.GetString(slice);
    }
}

public sealed class InvalidCaptureFileException : Exception
{
    public InvalidCaptureFileException(string message) : base(message) { }
}
