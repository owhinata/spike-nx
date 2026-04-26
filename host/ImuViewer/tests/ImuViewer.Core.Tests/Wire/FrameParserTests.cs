using System.Buffers.Binary;
using FluentAssertions;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Wire;

public class FrameParserTests
{
    private static byte[] BuildFrame(
        ushort seq = 0,
        uint firstTsUs = 0,
        ushort odr = 833,
        ushort accelFsr = 8,
        ushort gyroFsr = 2000,
        params (short ax, short ay, short az, short gx, short gy, short gz, uint dt)[] samples)
    {
        int count = samples.Length;
        int frameLen = WireConstants.HeaderSize + count * WireConstants.SampleSize;
        byte[] buf = new byte[frameLen];
        Span<byte> s = buf;
        BinaryPrimitives.WriteUInt16LittleEndian(s[0..2], WireConstants.Magic);
        s[2] = WireConstants.ImuFrameType;
        s[3] = (byte)count;
        BinaryPrimitives.WriteUInt16LittleEndian(s[4..6], odr);
        BinaryPrimitives.WriteUInt16LittleEndian(s[6..8], accelFsr);
        BinaryPrimitives.WriteUInt16LittleEndian(s[8..10], gyroFsr);
        BinaryPrimitives.WriteUInt16LittleEndian(s[10..12], seq);
        BinaryPrimitives.WriteUInt32LittleEndian(s[12..16], firstTsUs);
        BinaryPrimitives.WriteUInt16LittleEndian(s[16..18], (ushort)frameLen);
        for (int i = 0; i < count; i++)
        {
            int off = WireConstants.HeaderSize + i * WireConstants.SampleSize;
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 0, 2), samples[i].ax);
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 2, 2), samples[i].ay);
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 4, 2), samples[i].az);
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 6, 2), samples[i].gx);
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 8, 2), samples[i].gy);
            BinaryPrimitives.WriteInt16LittleEndian(s.Slice(off + 10, 2), samples[i].gz);
            BinaryPrimitives.WriteUInt32LittleEndian(s.Slice(off + 12, 4), samples[i].dt);
        }
        return buf;
    }

    [Fact]
    public void Parses_single_well_formed_frame()
    {
        byte[] bytes = BuildFrame(
            seq: 7,
            firstTsUs: 1_234_567,
            odr: 833,
            accelFsr: 8,
            gyroFsr: 2000,
            samples: [
                (10, 20, 30, 40, 50, 60, 0),
                (-10, -20, -30, -40, -50, -60, 1200),
            ]);

        FrameParser parser = new();
        parser.Append(bytes);
        parser.TryReadFrame(out ImuFrame? frame).Should().BeTrue();
        frame!.Header.Seq.Should().Be(7);
        frame.Header.SampleCount.Should().Be(2);
        frame.Header.AccelFsrG.Should().Be(8);
        frame.Header.GyroFsrDps.Should().Be(2000);
        frame.Samples.Length.Should().Be(2);
        frame.Samples[0].RawAx.Should().Be(10);
        frame.Samples[1].RawGz.Should().Be(-60);
        frame.Samples[1].TimestampDeltaUs.Should().Be(1200u);

        parser.TryReadFrame(out _).Should().BeFalse();
    }

    [Fact]
    public void Resyncs_after_garbage_prefix()
    {
        byte[] frame = BuildFrame(
            samples: [(1, 2, 3, 4, 5, 6, 0)]);
        byte[] payload = new byte[5 + frame.Length];
        // Garbage prefix that does NOT contain the magic byte pair.
        payload[0] = 0x00;
        payload[1] = 0xFF;
        payload[2] = 0x10;
        payload[3] = 0x20;
        payload[4] = 0x30;
        Array.Copy(frame, 0, payload, 5, frame.Length);

        FrameParser parser = new();
        parser.Append(payload);
        parser.TryReadFrame(out ImuFrame? f).Should().BeTrue();
        f!.Samples[0].RawAx.Should().Be(1);
    }

    [Fact]
    public void Parses_two_back_to_back_frames()
    {
        byte[] first = BuildFrame(seq: 1, samples: [(10, 0, 0, 0, 0, 0, 0)]);
        byte[] second = BuildFrame(seq: 2, samples: [(20, 0, 0, 0, 0, 0, 0)]);
        byte[] both = new byte[first.Length + second.Length];
        Array.Copy(first, 0, both, 0, first.Length);
        Array.Copy(second, 0, both, first.Length, second.Length);

        FrameParser parser = new();
        parser.Append(both);
        parser.TryReadFrame(out ImuFrame? f1).Should().BeTrue();
        parser.TryReadFrame(out ImuFrame? f2).Should().BeTrue();
        f1!.Header.Seq.Should().Be(1);
        f2!.Header.Seq.Should().Be(2);
        parser.TryReadFrame(out _).Should().BeFalse();
    }

    [Fact]
    public void Handles_partial_chunks()
    {
        byte[] frame = BuildFrame(seq: 99, samples: [(1, 2, 3, 4, 5, 6, 0)]);
        FrameParser parser = new();

        for (int i = 0; i < frame.Length - 1; i++)
        {
            parser.Append(frame.AsSpan(i, 1));
            parser.TryReadFrame(out _).Should().BeFalse();
        }

        parser.Append(frame.AsSpan(frame.Length - 1, 1));
        parser.TryReadFrame(out ImuFrame? f).Should().BeTrue();
        f!.Header.Seq.Should().Be(99);
    }

    [Fact]
    public void Rejects_frame_with_wrong_type_and_advances_one_byte()
    {
        byte[] frame = BuildFrame(samples: [(1, 0, 0, 0, 0, 0, 0)]);
        // Corrupt the type byte.
        frame[2] = 0x02;

        FrameParser parser = new();
        parser.Append(frame);
        parser.TryReadFrame(out _).Should().BeFalse();
    }

    [Fact]
    public void Rejects_frame_with_invalid_frame_len_and_resyncs_to_next_frame()
    {
        byte[] bad = BuildFrame(seq: 1, samples: [(1, 0, 0, 0, 0, 0, 0)]);
        // Corrupt frame_len so it disagrees with sample_count.
        BinaryPrimitives.WriteUInt16LittleEndian(bad.AsSpan(16, 2), 999);
        byte[] good = BuildFrame(seq: 2, samples: [(2, 0, 0, 0, 0, 0, 0)]);
        byte[] combined = new byte[bad.Length + good.Length];
        Array.Copy(bad, combined, bad.Length);
        Array.Copy(good, 0, combined, bad.Length, good.Length);

        FrameParser parser = new();
        parser.Append(combined);
        parser.TryReadFrame(out ImuFrame? f).Should().BeTrue();
        f!.Header.Seq.Should().Be(2);
        f.Samples[0].RawAx.Should().Be(2);
    }

    [Fact]
    public void Rejects_invalid_sample_count()
    {
        byte[] frame = BuildFrame(samples: [(1, 0, 0, 0, 0, 0, 0)]);
        // sample_count = 200, exceeds MaxSampleCount (80)
        frame[3] = 200;

        FrameParser parser = new();
        parser.Append(frame);
        parser.TryReadFrame(out _).Should().BeFalse();
    }
}
