using System.Buffers.Binary;

namespace ImuViewer.Core.Wire;

/// <summary>
/// Parses a single 27 B IMU_CAP frame (envelope + 22 B payload, Phase
/// 2.5 / Issue #145).  Mirrors <see cref="BundleFrameParser"/> for the
/// BUNDLE frame type — callers (typically
/// <see cref="Btsensor.BtsensorSession"/>) hand a span starting at
/// offset 0 of the frame and get back the decoded payload or null on
/// any size / header inconsistency.
/// </summary>
public static class ImuCapFrameParser
{
    /// <summary>
    /// Decode an IMU_CAP frame whose bytes start at offset 0 of the
    /// supplied span.  Returns null when the span is too short, the
    /// magic does not match, the frame_type is not
    /// <see cref="WireConstants.ImuCapFrameType"/>, or the declared
    /// frame_len is not exactly <see cref="WireConstants.ImuCapFrameSize"/>.
    /// </summary>
    public static ImuCapFrame? TryDecode(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < WireConstants.ImuCapFrameSize)
        {
            return null;
        }

        ushort magic = BinaryPrimitives.ReadUInt16LittleEndian(bytes[0..2]);
        if (magic != WireConstants.Magic)
        {
            return null;
        }

        if (bytes[2] != WireConstants.ImuCapFrameType)
        {
            return null;
        }

        ushort frameLen = BinaryPrimitives.ReadUInt16LittleEndian(bytes[3..5]);
        if (frameLen != WireConstants.ImuCapFrameSize)
        {
            return null;
        }

        // Payload starts at byte 5 (right after the envelope).
        ReadOnlySpan<byte> p = bytes.Slice(WireConstants.BundleEnvelopeSize,
                                           WireConstants.ImuCapPayloadSize);

        return new ImuCapFrame(
            TimestampUs:    BinaryPrimitives.ReadUInt32LittleEndian(p[0..4]),
            Ax:             BinaryPrimitives.ReadInt16LittleEndian(p[4..6]),
            Ay:             BinaryPrimitives.ReadInt16LittleEndian(p[6..8]),
            Az:             BinaryPrimitives.ReadInt16LittleEndian(p[8..10]),
            Gx:             BinaryPrimitives.ReadInt16LittleEndian(p[10..12]),
            Gy:             BinaryPrimitives.ReadInt16LittleEndian(p[12..14]),
            Gz:             BinaryPrimitives.ReadInt16LittleEndian(p[14..16]),
            TemperatureRaw: BinaryPrimitives.ReadInt16LittleEndian(p[16..18]),
            FsrXlIdx:       p[18],
            FsrGyIdx:       p[19],
            Seq:            BinaryPrimitives.ReadUInt16LittleEndian(p[20..22]));
    }
}
