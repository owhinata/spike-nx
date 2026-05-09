using CaptureViewer.Core;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;
using CaptureViewer.Core.Live;
using CaptureViewer.Core.Tests.Fixtures;

using FluentAssertions;

using Xunit;

namespace CaptureViewer.Core.Tests;

public class LiveCaptureReceiverTests
{
    private static readonly Func<ushort, bool> ResolveKnown =
        magic => KnownSchemas.ByMagic.ContainsKey(magic);

    private static byte[] OneSession(ushort magic = 0x0010, byte recordValue = 50)
    {
        var cap = CaptureBuilder.BuildReflectionCap(
            startTsUs: 1_000, reflectionPercents: new[] { recordValue });
        return CaptureBuilder.WrapBt(
            cap, magic, "color_reflection_run", CaptureProtocol.BTCE);
    }

    [Fact]
    public async Task ReceivesSingleSessionFromMemoryStream()
    {
        var stream = new MemoryStream(OneSession());
        var captures = new List<SessionScan>();

        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.CaptureReceived += s => captures.Add(s);
        rx.Start();

        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        captures.Should().HaveCount(1);
        captures[0].Capture.SchemaName.Should().Be("color_reflection_run");
        captures[0].Capture.RecordCount.Should().Be(1);
        rx.SessionsReceived.Should().Be(1);
        rx.BytesRead.Should().BeGreaterThan(0);
    }

    [Fact]
    public async Task ReceivesBackToBackSessions()
    {
        var s1 = OneSession(recordValue: 25);
        var s2 = OneSession(recordValue: 75);
        var stream = new MemoryStream(s1.Concat(s2).ToArray());

        var captures = new List<SessionScan>();
        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.CaptureReceived += s => captures.Add(s);
        rx.Start();
        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        captures.Should().HaveCount(2);
        rx.SessionsReceived.Should().Be(2);
    }

    [Fact]
    public async Task SkipsLeadingNoiseBeforeBtcs()
    {
        // Simulate the live wire where the receiver attaches mid-
        // telemetry: random bytes (no BTCS) precede the real session.
        var noise = new byte[200];
        new Random(0xC0DE).NextBytes(noise);
        // Make extra sure the noise contains no BTCS substring.
        for (var i = 0; i < noise.Length - 3; i++)
        {
            if (noise[i] == (byte)'B' && noise[i + 1] == (byte)'T' &&
                noise[i + 2] == (byte)'C' && noise[i + 3] == (byte)'S')
                noise[i] = 0x00;
        }
        var stream = new MemoryStream(noise.Concat(OneSession()).ToArray());

        var captures = new List<SessionScan>();
        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.CaptureReceived += s => captures.Add(s);
        rx.Start();
        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        captures.Should().HaveCount(1);
    }

    [Fact]
    public async Task TruncatedSession_DoesNotFire()
    {
        // Drop the last 8 bytes — no terminator + half-payload missing.
        var full = OneSession();
        var truncated = full.AsMemory(0, full.Length - 8).ToArray();
        var stream = new MemoryStream(truncated);

        var captures = new List<SessionScan>();
        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.CaptureReceived += s => captures.Add(s);
        rx.Start();
        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        captures.Should().BeEmpty();
        rx.SessionsReceived.Should().Be(0);
    }

    [Fact]
    public async Task UnknownSchemaMagic_IsRejectedWithResolver()
    {
        // Wrap with magic 0xCAFE that the resolver does not know about.
        var cap = CaptureBuilder.BuildReflectionCap(0, new byte[] { 1 });
        var bytes = CaptureBuilder.WrapBt(cap, 0xCAFE, "unknown_schema",
                                          CaptureProtocol.BTCE);
        // The schema name "unknown_schema" passes the charset check, so
        // sanity falls through to the resolver — which says no.
        var stream = new MemoryStream(bytes);

        var captures = new List<SessionScan>();
        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.CaptureReceived += s => captures.Add(s);
        rx.Start();
        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        captures.Should().BeEmpty();
    }

    [Fact]
    public async Task StreamClosed_FiresWhenStreamReachesEof()
    {
        var stream = new MemoryStream(OneSession());
        var closed = false;

        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.StreamClosed += () => closed = true;
        rx.Start();
        await rx.WaitForExitAsync(new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

        closed.Should().BeTrue();
    }

    [Fact]
    public async Task SendCommand_AppendsNewlineWhenMissing()
    {
        var ms = new BidirectionalMemoryStream();
        await using var rx = new LiveCaptureReceiver(ms, ResolveKnown, ownsStream: false);
        rx.Start();

        await rx.SendCommandAsync("MODE CAPTURE");

        var sent = ms.WrittenBytes;
        System.Text.Encoding.ASCII.GetString(sent).Should().Be("MODE CAPTURE\n");
    }

    [Fact]
    public async Task DoubleStart_Throws()
    {
        var stream = new MemoryStream(OneSession());
        await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
        rx.Start();
        var act = () => rx.Start();
        act.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public async Task SaveToDisk_RoundTripsThroughCaptureFileWriter()
    {
        var dir = Path.Combine(Path.GetTempPath(),
            "captureviewer-tests-" + Guid.NewGuid().ToString("N"));
        try
        {
            var stream = new MemoryStream(OneSession());
            var writer = new CaptureFileWriter(dir);
            string? savedPath = null;

            await using var rx = new LiveCaptureReceiver(stream, ResolveKnown);
            rx.CaptureReceived += s => savedPath = writer.Save(s.Capture);
            rx.Start();
            await rx.WaitForExitAsync(
                new CancellationTokenSource(TimeSpan.FromSeconds(2)).Token);

            savedPath.Should().NotBeNull();
            File.Exists(savedPath!).Should().BeTrue();

            // The file we just saved should round-trip through the
            // file parser back to the same SchemaName / RecordCount.
            var roundTrip = CaptureFile.Open(savedPath!);
            roundTrip.SchemaName.Should().Be("color_reflection_run");
            roundTrip.RecordCount.Should().Be(1);
        }
        finally
        {
            if (Directory.Exists(dir)) Directory.Delete(dir, recursive: true);
        }
    }

    /// <summary>
    /// Memory stream that lets the test inspect bytes written to it by
    /// the receiver while still allowing reads (returning EOF
    /// immediately).  Real RFCOMM is bidirectional; MemoryStream is
    /// not, so we wire two memory streams together.
    /// </summary>
    private sealed class BidirectionalMemoryStream : Stream
    {
        private readonly MemoryStream _read = new();        // empty -> EOF
        private readonly MemoryStream _write = new();
        public byte[] WrittenBytes => _write.ToArray();
        public override bool CanRead => true;
        public override bool CanWrite => true;
        public override bool CanSeek => false;
        public override long Length => throw new NotSupportedException();
        public override long Position
        {
            get => throw new NotSupportedException();
            set => throw new NotSupportedException();
        }
        public override void Flush() => _write.Flush();
        public override int Read(byte[] buffer, int offset, int count) =>
            _read.Read(buffer, offset, count);
        public override long Seek(long offset, SeekOrigin origin) =>
            throw new NotSupportedException();
        public override void SetLength(long value) =>
            throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) =>
            _write.Write(buffer, offset, count);
    }
}
