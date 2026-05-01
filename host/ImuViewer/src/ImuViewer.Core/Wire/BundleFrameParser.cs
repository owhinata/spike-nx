using System.Buffers.Binary;
using System.Collections.Immutable;

namespace ImuViewer.Core.Wire;

/// <summary>
/// Parses a single complete BUNDLE frame from a byte span.
/// </summary>
/// <remarks>
/// Used by <see cref="Btsensor.BtsensorSession"/> after that class has
/// scanned the byte stream for the magic + envelope and confirmed the
/// declared <c>frame_len</c> is fully buffered.  This parser does the
/// inner field-by-field decode and validation.
/// </remarks>
public static class BundleFrameParser
{
    /// <summary>
    /// Decode a full BUNDLE frame whose bytes start at offset 0 of the
    /// supplied span.  Returns null on any header / size inconsistency
    /// (caller should advance the byte stream by 1 to resync).
    /// </summary>
    public static BundleFrame? TryDecode(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length < WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize)
        {
            return null;
        }

        ushort magic = BinaryPrimitives.ReadUInt16LittleEndian(bytes[0..2]);
        if (magic != WireConstants.Magic)
        {
            return null;
        }

        if (bytes[2] != WireConstants.BundleFrameType)
        {
            return null;
        }

        ushort frameLen = BinaryPrimitives.ReadUInt16LittleEndian(bytes[3..5]);
        if (frameLen > bytes.Length || frameLen < WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize)
        {
            return null;
        }

        ReadOnlySpan<byte> hdr = bytes.Slice(WireConstants.BundleEnvelopeSize, WireConstants.BundleHeaderSize);
        BundleFrameHeader header = new(
            Seq: BinaryPrimitives.ReadUInt16LittleEndian(hdr[0..2]),
            TickTsUs: BinaryPrimitives.ReadUInt32LittleEndian(hdr[2..6]),
            ImuSectionLen: BinaryPrimitives.ReadUInt16LittleEndian(hdr[6..8]),
            ImuSampleCount: hdr[8],
            TlvCount: hdr[9],
            ImuSampleRateHz: BinaryPrimitives.ReadUInt16LittleEndian(hdr[10..12]),
            ImuAccelFsrG: hdr[12],
            ImuGyroFsrDps: BinaryPrimitives.ReadUInt16LittleEndian(hdr[13..15]),
            Flags: hdr[15]);

        if (header.ImuSampleCount > WireConstants.MaxImuSamplesPerBundle)
        {
            return null;
        }

        if (header.TlvCount != WireConstants.TlvCount)
        {
            return null;
        }

        int imuSectionLen = header.ImuSampleCount * WireConstants.ImuSampleSize;
        if (header.ImuSectionLen != imuSectionLen)
        {
            return null;
        }

        int imuStart = WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize;
        int tlvStart = imuStart + imuSectionLen;
        if (tlvStart > frameLen)
        {
            return null;
        }

        // IMU subsection.
        ImuSample[] samples = new ImuSample[header.ImuSampleCount];
        for (int i = 0; i < header.ImuSampleCount; i++)
        {
            int off = imuStart + i * WireConstants.ImuSampleSize;
            samples[i] = ReadImuSample(bytes.Slice(off, WireConstants.ImuSampleSize));
        }

        // TLV subsection: TlvCount entries, each variable-size.
        LegoTlv[] tlvs = new LegoTlv[header.TlvCount];
        int tlvOff = tlvStart;
        for (int i = 0; i < header.TlvCount; i++)
        {
            if (tlvOff + WireConstants.TlvHeaderSize > frameLen)
            {
                return null;
            }

            ReadOnlySpan<byte> tlvHdr = bytes.Slice(tlvOff, WireConstants.TlvHeaderSize);
            byte payloadLen = tlvHdr[5];
            if (payloadLen > WireConstants.MaxTlvPayload)
            {
                return null;
            }

            int payloadOff = tlvOff + WireConstants.TlvHeaderSize;
            int payloadEnd = payloadOff + payloadLen;
            if (payloadEnd > frameLen)
            {
                return null;
            }

            ImmutableArray<byte> payload = payloadLen == 0
                ? ImmutableArray<byte>.Empty
                : ImmutableArray.Create(bytes.Slice(payloadOff, payloadLen).ToArray());

            tlvs[i] = new LegoTlv(
                ClassId: (LegoClassId)tlvHdr[0],
                PortId: tlvHdr[1],
                ModeId: tlvHdr[2],
                DataType: (LegoDataType)tlvHdr[3],
                NumValues: tlvHdr[4],
                Flags: (LegoTlvFlags)tlvHdr[6],
                Age10ms: tlvHdr[7],
                Seq: BinaryPrimitives.ReadUInt16LittleEndian(tlvHdr[8..10]),
                Payload: payload);

            tlvOff = payloadEnd;
        }

        if (tlvOff != frameLen)
        {
            return null;
        }

        return new BundleFrame(header, [.. samples], [.. tlvs]);
    }

    private static ImuSample ReadImuSample(ReadOnlySpan<byte> bytes) => new(
        BinaryPrimitives.ReadInt16LittleEndian(bytes[0..2]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[2..4]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[4..6]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[6..8]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[8..10]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[10..12]),
        BinaryPrimitives.ReadUInt32LittleEndian(bytes[12..16]));
}
