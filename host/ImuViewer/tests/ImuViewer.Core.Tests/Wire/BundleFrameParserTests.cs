using System.Buffers.Binary;
using System.Collections.Immutable;
using FluentAssertions;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Wire;

/// <summary>
/// BUNDLE wire format decode coverage (Issue #88).
/// </summary>
public class BundleFrameParserTests
{
    /// <summary>
    /// Build a synthetic BUNDLE: caller specifies the IMU sample count and
    /// per-class TLV payloads.  TLVs absent from <paramref name="tlvPayloads"/>
    /// are emitted as unbound (flags=0, payload_len=0).
    /// </summary>
    private static byte[] BuildBundle(
        byte imuSampleCount,
        Dictionary<LegoClassId, byte[]>? tlvPayloads = null,
        ushort seq = 1,
        uint tickTsUs = 0,
        ushort sampleRate = 833,
        byte accelFsr = 8,
        ushort gyroFsr = 2000,
        byte flags = WireConstants.FlagImuOn)
    {
        tlvPayloads ??= new();
        int imuSectionLen = imuSampleCount * WireConstants.ImuSampleSize;
        int tlvSectionLen = WireConstants.TlvCount * WireConstants.TlvHeaderSize;
        foreach ((_, byte[] p) in tlvPayloads)
        {
            tlvSectionLen += p.Length;
        }
        int frameLen = WireConstants.BundleEnvelopeSize +
                       WireConstants.BundleHeaderSize +
                       imuSectionLen + tlvSectionLen;
        byte[] buf = new byte[frameLen];
        Span<byte> s = buf;

        BinaryPrimitives.WriteUInt16LittleEndian(s[0..2], WireConstants.Magic);
        s[2] = WireConstants.BundleFrameType;
        BinaryPrimitives.WriteUInt16LittleEndian(s[3..5], (ushort)frameLen);

        Span<byte> hdr = s.Slice(WireConstants.BundleEnvelopeSize, WireConstants.BundleHeaderSize);
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[0..2], seq);
        BinaryPrimitives.WriteUInt32LittleEndian(hdr[2..6], tickTsUs);
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[6..8], (ushort)imuSectionLen);
        hdr[8] = imuSampleCount;
        hdr[9] = WireConstants.TlvCount;
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[10..12], sampleRate);
        hdr[12] = accelFsr;
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[13..15], gyroFsr);
        hdr[15] = flags;

        // IMU samples: ax = i, others 0, ts_delta = i*1000.
        int imuStart = WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize;
        for (int i = 0; i < imuSampleCount; i++)
        {
            Span<byte> sample = s.Slice(imuStart + i * WireConstants.ImuSampleSize, WireConstants.ImuSampleSize);
            BinaryPrimitives.WriteInt16LittleEndian(sample[0..2], (short)(i + 1));
            BinaryPrimitives.WriteInt16LittleEndian(sample[2..4], 0);
            BinaryPrimitives.WriteInt16LittleEndian(sample[4..6], 0);
            BinaryPrimitives.WriteInt16LittleEndian(sample[6..8], 0);
            BinaryPrimitives.WriteInt16LittleEndian(sample[8..10], 0);
            BinaryPrimitives.WriteInt16LittleEndian(sample[10..12], 0);
            BinaryPrimitives.WriteUInt32LittleEndian(sample[12..16], (uint)(i * 1000));
        }

        // TLVs in fixed class order.
        int tlvOff = imuStart + imuSectionLen;
        for (int i = 0; i < WireConstants.TlvCount; i++)
        {
            LegoClassId classId = (LegoClassId)i;
            byte[] payload = tlvPayloads.TryGetValue(classId, out byte[]? p) ? p : Array.Empty<byte>();
            Span<byte> tlv = s.Slice(tlvOff, WireConstants.TlvHeaderSize + payload.Length);
            tlv[0] = (byte)i;
            tlv[1] = payload.Length > 0 ? (byte)0 : (byte)0xFF;
            tlv[2] = 0;
            tlv[3] = 0;
            tlv[4] = 0;
            tlv[5] = (byte)payload.Length;
            tlv[6] = payload.Length > 0
                ? (byte)(LegoTlvFlags.Bound | LegoTlvFlags.Fresh)
                : (byte)0;
            tlv[7] = payload.Length > 0 ? (byte)0 : (byte)0xFF;
            BinaryPrimitives.WriteUInt16LittleEndian(tlv[8..10], (ushort)i);
            payload.CopyTo(tlv[WireConstants.TlvHeaderSize..]);
            tlvOff += WireConstants.TlvHeaderSize + payload.Length;
        }

        return buf;
    }

    [Fact]
    public void Empty_imu_with_all_unbound_tlvs_decodes()
    {
        byte[] frame = BuildBundle(imuSampleCount: 0);
        BundleFrame? result = BundleFrameParser.TryDecode(frame);
        result.Should().NotBeNull();
        result!.Header.ImuSampleCount.Should().Be(0);
        result.Header.TlvCount.Should().Be(WireConstants.TlvCount);
        result.ImuSamples.Length.Should().Be(0);
        result.Tlvs.Length.Should().Be(WireConstants.TlvCount);
        foreach (LegoTlv tlv in result.Tlvs)
        {
            tlv.IsBound.Should().BeFalse();
            tlv.IsFresh.Should().BeFalse();
            tlv.Payload.Length.Should().Be(0);
        }
    }

    [Fact]
    public void Imu_only_bundle_decodes_samples_in_order()
    {
        byte[] frame = BuildBundle(imuSampleCount: 8);
        BundleFrame? result = BundleFrameParser.TryDecode(frame);
        result.Should().NotBeNull();
        result!.ImuSamples.Length.Should().Be(8);
        result.ImuSamples[0].RawAx.Should().Be(1);
        result.ImuSamples[7].RawAx.Should().Be(8);
        result.ImuSamples[7].TimestampDeltaUs.Should().Be(7000u);
    }

    [Fact]
    public void All_tlvs_fresh_with_payload_decodes()
    {
        Dictionary<LegoClassId, byte[]> payloads = new()
        {
            [LegoClassId.Color] = new byte[] { 1, 2, 3 },
            [LegoClassId.Ultrasonic] = new byte[] { 4, 5 },
            [LegoClassId.Force] = new byte[] { 6 },
            [LegoClassId.MotorM] = new byte[] { 7, 8 },
            [LegoClassId.MotorR] = new byte[] { 9, 10 },
            [LegoClassId.MotorL] = new byte[] { 11, 12 },
        };
        byte[] frame = BuildBundle(imuSampleCount: 4, tlvPayloads: payloads);
        BundleFrame? result = BundleFrameParser.TryDecode(frame);
        result.Should().NotBeNull();
        result!.Tlvs[(int)LegoClassId.Color].Payload.ToArray().Should().Equal((byte)1, (byte)2, (byte)3);
        result.Tlvs[(int)LegoClassId.Force].Payload.ToArray().Should().Equal((byte)6);
        foreach (LegoTlv tlv in result.Tlvs)
        {
            tlv.IsBound.Should().BeTrue();
            tlv.IsFresh.Should().BeTrue();
        }
    }

    [Fact]
    public void Mixed_fresh_and_unbound_tlvs_decode()
    {
        Dictionary<LegoClassId, byte[]> payloads = new()
        {
            [LegoClassId.Color] = new byte[] { 0xAA, 0xBB },
            [LegoClassId.MotorR] = new byte[] { 0xCC },
        };
        byte[] frame = BuildBundle(imuSampleCount: 2, tlvPayloads: payloads);
        BundleFrame? result = BundleFrameParser.TryDecode(frame);
        result.Should().NotBeNull();
        result!.Tlvs[(int)LegoClassId.Color].IsFresh.Should().BeTrue();
        result.Tlvs[(int)LegoClassId.Ultrasonic].IsFresh.Should().BeFalse();
        result.Tlvs[(int)LegoClassId.MotorR].IsFresh.Should().BeTrue();
        result.Tlvs[(int)LegoClassId.MotorR].Payload.ToArray().Should().Equal((byte)0xCC);
    }

    [Fact]
    public void Wrong_magic_returns_null()
    {
        byte[] frame = BuildBundle(imuSampleCount: 1);
        frame[0] = 0x00;
        BundleFrameParser.TryDecode(frame).Should().BeNull();
    }

    [Fact]
    public void Wrong_type_returns_null()
    {
        byte[] frame = BuildBundle(imuSampleCount: 1);
        frame[2] = 0x01;
        BundleFrameParser.TryDecode(frame).Should().BeNull();
    }

    [Fact]
    public void TlvCount_other_than_six_returns_null()
    {
        byte[] frame = BuildBundle(imuSampleCount: 1);
        // Header tlv_count is at frame offset 5+9 = 14.
        frame[14] = 5;
        BundleFrameParser.TryDecode(frame).Should().BeNull();
    }

    [Fact]
    public void Imu_section_length_mismatch_returns_null()
    {
        byte[] frame = BuildBundle(imuSampleCount: 2);
        // Mutate imu_section_len to a wrong value.
        frame[5 + 6] = 0x00;
        frame[5 + 7] = 0x00;
        BundleFrameParser.TryDecode(frame).Should().BeNull();
    }

    [Fact]
    public void Truncated_input_returns_null()
    {
        byte[] frame = BuildBundle(imuSampleCount: 1);
        ReadOnlySpan<byte> truncated = frame.AsSpan(0, frame.Length - 5);
        BundleFrameParser.TryDecode(truncated).Should().BeNull();
    }
}
