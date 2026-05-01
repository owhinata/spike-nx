using System.Buffers.Binary;
using System.Text;
using FluentAssertions;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Btsensor;

public class BtsensorSessionTests
{
    /// <summary>
    /// Build a minimal valid BUNDLE frame: one IMU sample, six "unbound"
    /// TLVs (one per LegoClassId, all flags=0 / payload_len=0).
    /// </summary>
    internal static byte[] BuildOneSampleBundle(ushort seq, short ax = 100)
    {
        const int imuSampleCount = 1;
        const int imuSectionLen = imuSampleCount * WireConstants.ImuSampleSize;
        int frameLen =
            WireConstants.BundleEnvelopeSize +
            WireConstants.BundleHeaderSize +
            imuSectionLen +
            WireConstants.TlvHeaderSize * WireConstants.TlvCount;

        byte[] buf = new byte[frameLen];
        Span<byte> s = buf;

        // Envelope.
        BinaryPrimitives.WriteUInt16LittleEndian(s[0..2], WireConstants.Magic);
        s[2] = WireConstants.BundleFrameType;
        BinaryPrimitives.WriteUInt16LittleEndian(s[3..5], (ushort)frameLen);

        // Bundle header.
        Span<byte> hdr = s.Slice(WireConstants.BundleEnvelopeSize, WireConstants.BundleHeaderSize);
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[0..2], seq);
        BinaryPrimitives.WriteUInt32LittleEndian(hdr[2..6], 0u); // tick_ts_us
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[6..8], imuSectionLen);
        hdr[8] = imuSampleCount;
        hdr[9] = WireConstants.TlvCount;
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[10..12], 833);
        hdr[12] = 8;          // accel FSR g
        BinaryPrimitives.WriteUInt16LittleEndian(hdr[13..15], 2000);
        hdr[15] = WireConstants.FlagImuOn;

        // IMU sample.
        Span<byte> sample = s.Slice(
            WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize,
            WireConstants.ImuSampleSize);
        BinaryPrimitives.WriteInt16LittleEndian(sample[0..2], ax);
        BinaryPrimitives.WriteInt16LittleEndian(sample[2..4], 0);
        BinaryPrimitives.WriteInt16LittleEndian(sample[4..6], 0);
        BinaryPrimitives.WriteInt16LittleEndian(sample[6..8], 0);
        BinaryPrimitives.WriteInt16LittleEndian(sample[8..10], 0);
        BinaryPrimitives.WriteInt16LittleEndian(sample[10..12], 0);
        BinaryPrimitives.WriteUInt32LittleEndian(sample[12..16], 0u);

        // 6 unbound TLVs (header only, no payload).
        int tlvOff = WireConstants.BundleEnvelopeSize +
                     WireConstants.BundleHeaderSize +
                     imuSectionLen;
        for (int i = 0; i < WireConstants.TlvCount; i++)
        {
            Span<byte> tlv = s.Slice(tlvOff, WireConstants.TlvHeaderSize);
            tlv[0] = (byte)i;          // class_id
            tlv[1] = 0xFF;             // port_id (unbound)
            tlv[2] = 0;                // mode_id
            tlv[3] = 0;                // data_type
            tlv[4] = 0;                // num_values
            tlv[5] = 0;                // payload_len
            tlv[6] = 0;                // flags
            tlv[7] = 0xFF;             // age_10ms saturated
            BinaryPrimitives.WriteUInt16LittleEndian(tlv[8..10], 0); // seq
            tlvOff += WireConstants.TlvHeaderSize;
        }

        return buf;
    }

    [Fact]
    public async Task Reads_a_single_bundle_followed_by_an_OK_reply()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        List<BundleFrame> frames = [];
        List<string> lines = [];
        session.BundleReceived += f => frames.Add(f);
        session.ReplyLineReceived += s => lines.Add(s);
        session.Start();

        await pipe.InjectAsync(BuildOneSampleBundle(seq: 1, ax: 4096));
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("OK\n"));
        await WaitUntil(() => frames.Count >= 1 && lines.Count >= 1);

        frames.Should().HaveCount(1);
        frames[0].Header.Seq.Should().Be(1);
        frames[0].ImuSamples[0].RawAx.Should().Be(4096);
        frames[0].Header.ImuSampleCount.Should().Be(1);
        frames[0].Tlvs.Length.Should().Be(WireConstants.TlvCount);
        lines.Should().Equal("OK");

        await session.DisposeAsync();
    }

    [Fact]
    public async Task Demuxes_reply_line_interleaved_between_two_bundles()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        List<ushort> seqs = [];
        List<string> lines = [];
        session.BundleReceived += f => { lock (seqs) seqs.Add(f.Header.Seq); };
        session.ReplyLineReceived += s => { lock (lines) lines.Add(s); };
        session.Start();

        await pipe.InjectAsync(BuildOneSampleBundle(seq: 1));
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("ERR busy\n"));
        await pipe.InjectAsync(BuildOneSampleBundle(seq: 2));

        await WaitUntil(() => seqs.Count >= 2 && lines.Count >= 1);
        seqs.Should().Equal((ushort)1, (ushort)2);
        lines.Should().Equal("ERR busy");

        await session.DisposeAsync();
    }

    [Fact]
    public async Task Reassembles_bundle_when_bytes_arrive_one_at_a_time()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        TaskCompletionSource<BundleFrame> got = new(TaskCreationOptions.RunContinuationsAsynchronously);
        session.BundleReceived += f => got.TrySetResult(f);
        session.Start();

        byte[] frame = BuildOneSampleBundle(seq: 99);
        for (int i = 0; i < frame.Length; i++)
        {
            await pipe.InjectAsync([frame[i]]);
        }

        BundleFrame f = await got.Task.WaitAsync(TimeSpan.FromSeconds(2));
        f.Header.Seq.Should().Be(99);

        await session.DisposeAsync();
    }

    [Fact]
    public async Task WriteLineAsync_appends_newline_to_outgoing_bytes()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);
        session.Start();

        await session.WriteLineAsync("IMU ON", CancellationToken.None);
        pipe.GetWritten().Should().Equal(Encoding.ASCII.GetBytes("IMU ON\n"));

        await session.DisposeAsync();
    }

    private static async Task WaitUntil(Func<bool> predicate, int timeoutMs = 2000)
    {
        int waited = 0;
        while (!predicate() && waited < timeoutMs)
        {
            await Task.Delay(20);
            waited += 20;
        }
        if (!predicate())
        {
            throw new TimeoutException("predicate did not become true within timeout");
        }
    }
}
