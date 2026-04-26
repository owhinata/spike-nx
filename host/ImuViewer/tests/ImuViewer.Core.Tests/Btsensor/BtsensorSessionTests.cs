using System.Buffers.Binary;
using System.Text;
using FluentAssertions;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Btsensor;

public class BtsensorSessionTests
{
    private static byte[] BuildOneSampleFrame(ushort seq, short ax = 100)
    {
        int frameLen = WireConstants.HeaderSize + WireConstants.SampleSize;
        byte[] buf = new byte[frameLen];
        Span<byte> s = buf;
        BinaryPrimitives.WriteUInt16LittleEndian(s[0..2], WireConstants.Magic);
        s[2] = WireConstants.ImuFrameType;
        s[3] = 1;
        BinaryPrimitives.WriteUInt16LittleEndian(s[4..6], 833);
        BinaryPrimitives.WriteUInt16LittleEndian(s[6..8], 8);
        BinaryPrimitives.WriteUInt16LittleEndian(s[8..10], 2000);
        BinaryPrimitives.WriteUInt16LittleEndian(s[10..12], seq);
        BinaryPrimitives.WriteUInt32LittleEndian(s[12..16], 0);
        BinaryPrimitives.WriteUInt16LittleEndian(s[16..18], (ushort)frameLen);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(18, 2), ax);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(20, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(22, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(24, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(26, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(s.Slice(28, 2), 0);
        BinaryPrimitives.WriteUInt32LittleEndian(s.Slice(30, 4), 0);
        return buf;
    }

    [Fact]
    public async Task Reads_a_single_frame_followed_by_an_OK_reply()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        List<ImuFrame> frames = [];
        List<string> lines = [];
        session.FrameReceived += f => frames.Add(f);
        session.ReplyLineReceived += s => lines.Add(s);
        session.Start();

        await pipe.InjectAsync(BuildOneSampleFrame(seq: 1, ax: 4096));
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("OK\n"));
        await WaitUntil(() => frames.Count >= 1 && lines.Count >= 1);

        frames.Should().HaveCount(1);
        frames[0].Header.Seq.Should().Be(1);
        frames[0].Samples[0].RawAx.Should().Be(4096);
        lines.Should().Equal("OK");

        await session.DisposeAsync();
    }

    [Fact]
    public async Task Demuxes_reply_line_interleaved_between_two_frames()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        List<ushort> seqs = [];
        List<string> lines = [];
        session.FrameReceived += f => { lock (seqs) seqs.Add(f.Header.Seq); };
        session.ReplyLineReceived += s => { lock (lines) lines.Add(s); };
        session.Start();

        await pipe.InjectAsync(BuildOneSampleFrame(seq: 1));
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("ERR busy\n"));
        await pipe.InjectAsync(BuildOneSampleFrame(seq: 2));

        await WaitUntil(() => seqs.Count >= 2 && lines.Count >= 1);
        seqs.Should().Equal((ushort)1, (ushort)2);
        lines.Should().Equal("ERR busy");

        await session.DisposeAsync();
    }

    [Fact]
    public async Task Reassembles_frame_when_bytes_arrive_one_at_a_time()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);

        TaskCompletionSource<ImuFrame> got = new(TaskCreationOptions.RunContinuationsAsynchronously);
        session.FrameReceived += f => got.TrySetResult(f);
        session.Start();

        byte[] frame = BuildOneSampleFrame(seq: 99);
        for (int i = 0; i < frame.Length; i++)
        {
            await pipe.InjectAsync([frame[i]]);
        }

        ImuFrame f = await got.Task.WaitAsync(TimeSpan.FromSeconds(2));
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
