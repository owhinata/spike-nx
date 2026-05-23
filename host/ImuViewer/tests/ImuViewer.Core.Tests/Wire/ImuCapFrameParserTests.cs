using System.Buffers.Binary;
using FluentAssertions;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Wire;

/// <summary>
/// Phase 2.5 / Issue #145 — IMU_CAP wire format decode coverage.
/// Mirrors BundleFrameParserTests for the 27 B fixed-size frame.
/// </summary>
public class ImuCapFrameParserTests
{
    private static byte[] BuildFrame(
        uint timestampUs = 1_000_000,
        short ax = 100, short ay = -200, short az = 16000,
        short gx = 5, short gy = -10, short gz = 15,
        short tempRaw = 1024,
        byte fsrXlIdx = 0,  // 2g
        byte fsrGyIdx = 4,  // 1000 dps
        ushort seq = 42,
        ushort? overrideFrameLen = null,
        ushort? overrideMagic = null,
        byte? overrideFrameType = null)
    {
        byte[] buf = new byte[WireConstants.ImuCapFrameSize];
        Span<byte> s = buf;

        BinaryPrimitives.WriteUInt16LittleEndian(s[0..2],
            overrideMagic ?? WireConstants.Magic);
        s[2] = overrideFrameType ?? WireConstants.ImuCapFrameType;
        BinaryPrimitives.WriteUInt16LittleEndian(s[3..5],
            overrideFrameLen ?? (ushort)WireConstants.ImuCapFrameSize);

        Span<byte> p = s.Slice(WireConstants.BundleEnvelopeSize,
                               WireConstants.ImuCapPayloadSize);
        BinaryPrimitives.WriteUInt32LittleEndian(p[0..4], timestampUs);
        BinaryPrimitives.WriteInt16LittleEndian(p[4..6], ax);
        BinaryPrimitives.WriteInt16LittleEndian(p[6..8], ay);
        BinaryPrimitives.WriteInt16LittleEndian(p[8..10], az);
        BinaryPrimitives.WriteInt16LittleEndian(p[10..12], gx);
        BinaryPrimitives.WriteInt16LittleEndian(p[12..14], gy);
        BinaryPrimitives.WriteInt16LittleEndian(p[14..16], gz);
        BinaryPrimitives.WriteInt16LittleEndian(p[16..18], tempRaw);
        p[18] = fsrXlIdx;
        p[19] = fsrGyIdx;
        BinaryPrimitives.WriteUInt16LittleEndian(p[20..22], seq);

        return buf;
    }

    [Fact]
    public void Decodes_well_formed_frame()
    {
        byte[] bytes = BuildFrame(
            timestampUs: 12_345_678u,
            ax: 1, ay: 2, az: 3,
            gx: -1, gy: -2, gz: -3,
            tempRaw: 256,
            fsrXlIdx: 2, fsrGyIdx: 6,
            seq: 0xDEAD);

        ImuCapFrame? frame = ImuCapFrameParser.TryDecode(bytes);

        frame.Should().NotBeNull();
        frame!.Value.TimestampUs.Should().Be(12_345_678u);
        frame.Value.Ax.Should().Be(1);
        frame.Value.Ay.Should().Be(2);
        frame.Value.Az.Should().Be(3);
        frame.Value.Gx.Should().Be(-1);
        frame.Value.Gy.Should().Be(-2);
        frame.Value.Gz.Should().Be(-3);
        frame.Value.TemperatureRaw.Should().Be(256);
        frame.Value.FsrXlIdx.Should().Be(2);
        frame.Value.FsrGyIdx.Should().Be(6);
        frame.Value.Seq.Should().Be(0xDEAD);
    }

    [Fact]
    public void Returns_null_when_buffer_is_too_short()
    {
        byte[] bytes = BuildFrame().AsSpan(0, WireConstants.ImuCapFrameSize - 1).ToArray();

        ImuCapFrameParser.TryDecode(bytes).Should().BeNull();
    }

    [Fact]
    public void Returns_null_on_magic_mismatch()
    {
        byte[] bytes = BuildFrame(overrideMagic: 0x1234);

        ImuCapFrameParser.TryDecode(bytes).Should().BeNull();
    }

    [Fact]
    public void Returns_null_on_frame_type_mismatch()
    {
        // BUNDLE type (0x02) in an envelope sized for IMU_CAP — must
        // reject so BtsensorSession can resync.
        byte[] bytes = BuildFrame(overrideFrameType: WireConstants.BundleFrameType);

        ImuCapFrameParser.TryDecode(bytes).Should().BeNull();
    }

    [Fact]
    public void Returns_null_on_frame_len_mismatch()
    {
        byte[] bytes = BuildFrame(overrideFrameLen: 28);

        ImuCapFrameParser.TryDecode(bytes).Should().BeNull();
    }

    [Fact]
    public void Sample_at_int16_extremes_round_trips()
    {
        byte[] bytes = BuildFrame(
            ax: short.MinValue, ay: short.MaxValue, az: 0,
            gx: short.MaxValue, gy: short.MinValue, gz: 0,
            tempRaw: short.MinValue,
            seq: ushort.MaxValue);

        ImuCapFrame? frame = ImuCapFrameParser.TryDecode(bytes);

        frame.Should().NotBeNull();
        frame!.Value.Ax.Should().Be(short.MinValue);
        frame.Value.Ay.Should().Be(short.MaxValue);
        frame.Value.Gx.Should().Be(short.MaxValue);
        frame.Value.Gy.Should().Be(short.MinValue);
        frame.Value.TemperatureRaw.Should().Be(short.MinValue);
        frame.Value.Seq.Should().Be(ushort.MaxValue);
    }
}
