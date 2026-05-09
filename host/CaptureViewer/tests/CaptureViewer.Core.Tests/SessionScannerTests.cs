using System.Buffers.Binary;

using CaptureViewer.Core;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;
using CaptureViewer.Core.Tests.Fixtures;

using FluentAssertions;

using Xunit;

namespace CaptureViewer.Core.Tests;

public class SessionScannerTests
{
    private static readonly Func<ushort, bool> ResolveKnown =
        magic => KnownSchemas.ByMagic.ContainsKey(magic);

    [Fact]
    public void TryScan_FindsBtceTerminatedSession()
    {
        var cap = CaptureBuilder.BuildReflectionCap(
            startTsUs: 1_000,
            reflectionPercents: new byte[] { 25, 50, 75 });
        var stream = CaptureBuilder.WrapBt(
            cap, schemaMagic: 0x0010,
            schemaName: "color_reflection_run",
            terminator: CaptureProtocol.BTCE);

        var ok = SessionScanner.TryScan(stream, ResolveKnown, out var scan);

        ok.Should().BeTrue();
        scan!.StartIndex.Should().Be(0);
        scan.EndIndex.Should().Be(stream.Length);
        scan.Capture.Termination.Should().Be(CaptureTermination.Clean);
        scan.Capture.RecordCount.Should().Be(3);
        scan.Capture.SchemaName.Should().Be("color_reflection_run");
    }

    [Fact]
    public void TryScan_RecognisesBtabAsAborted()
    {
        var cap = CaptureBuilder.BuildReflectionCap(
            startTsUs: 0, reflectionPercents: new byte[] { 1 });
        var stream = CaptureBuilder.WrapBt(
            cap, 0x0010, "color_reflection_run",
            CaptureProtocol.BTAB);

        SessionScanner.TryScan(stream, ResolveKnown, out var scan).Should().BeTrue();
        scan!.Capture.Termination.Should().Be(CaptureTermination.Aborted);
    }

    [Fact]
    public void TryScan_SkipsLeadingNoiseUntilBtcs()
    {
        // Some preamble that contains no BTCS substring.
        var noise = new byte[] { 0x00, 0x10, 0xAA, 0xFF, (byte)'B', (byte)'T', (byte)'X' };
        var cap = CaptureBuilder.BuildReflectionCap(0, new byte[] { 50 });
        var session = CaptureBuilder.WrapBt(cap, 0x0010, "color_reflection_run",
                                             CaptureProtocol.BTCE);
        var combined = new byte[noise.Length + session.Length];
        noise.CopyTo(combined.AsSpan());
        session.CopyTo(combined.AsSpan(noise.Length));

        SessionScanner.TryScan(combined, ResolveKnown, out var scan).Should().BeTrue();
        scan!.StartIndex.Should().Be(noise.Length);
        scan.EndIndex.Should().Be(combined.Length);
    }

    [Fact]
    public void TryScan_RejectsFalsePositiveBtcsBeforeRealOne()
    {
        // First "BTCS" lands but is followed by garbage — meta sanity
        // (impossible total_bytes and unknown schema_magic) rejects
        // it.  Real session sits right after.
        var fake = new byte[CaptureProtocol.BTCS.Length + CaptureProtocol.SessionMetaSize + 8];
        CaptureProtocol.BTCS.CopyTo(fake.AsSpan(0, CaptureProtocol.BTCS.Length));
        // schema_magic = 0xCAFE (unknown), total_bytes = 1 GiB,
        // schema_name = full of 0x55 (rejected by name-charset check)
        BinaryPrimitives.WriteUInt16LittleEndian(fake.AsSpan(4, 2), 0xCAFE);
        BinaryPrimitives.WriteUInt32LittleEndian(fake.AsSpan(8, 4), 1024u * 1024 * 1024);
        for (var i = 0; i < CaptureProtocol.SchemaNameMax; i++)
            fake[12 + i] = 0x55;

        var cap = CaptureBuilder.BuildReflectionCap(0, new byte[] { 70 });
        var real = CaptureBuilder.WrapBt(cap, 0x0010, "color_reflection_run",
                                         CaptureProtocol.BTCE);
        var combined = new byte[fake.Length + real.Length];
        fake.CopyTo(combined.AsSpan());
        real.CopyTo(combined.AsSpan(fake.Length));

        SessionScanner.TryScan(combined, ResolveKnown, out var scan).Should().BeTrue();
        scan!.StartIndex.Should().Be(fake.Length);
        scan.Capture.SchemaName.Should().Be("color_reflection_run");
    }

    [Fact]
    public void TryScan_NotEnoughBytesForMeta_ReturnsFalse()
    {
        var partial = new byte[CaptureProtocol.BTCS.Length + 5];
        CaptureProtocol.BTCS.CopyTo(partial.AsSpan(0, CaptureProtocol.BTCS.Length));

        SessionScanner.TryScan(partial, ResolveKnown, out var scan).Should().BeFalse();
        scan.Should().BeNull();
    }

    [Fact]
    public void TryScan_NotEnoughBytesForPayload_ReturnsFalse()
    {
        var cap = CaptureBuilder.BuildReflectionCap(0, new byte[] { 90 });
        var full = CaptureBuilder.WrapBt(cap, 0x0010, "color_reflection_run",
                                         CaptureProtocol.BTCE);
        // Drop the trailing terminator + half the payload.
        var partial = full.AsSpan(0, full.Length - cap.Length / 2 - 4).ToArray();

        SessionScanner.TryScan(partial, ResolveKnown, out var scan).Should().BeFalse();
        scan.Should().BeNull();
    }

    [Fact]
    public void TryScan_TerminatorMismatch_RescansAndFindsRealSession()
    {
        // First wrap uses fake terminator "XXXX" — sanity check on the
        // BTCE/BTAB byte string fails so the session is rejected.
        var cap1 = CaptureBuilder.BuildReflectionCap(0, new byte[] { 5 });
        var fake = CaptureBuilder.WrapBt(cap1, 0x0010, "color_reflection_run",
                                         "XXXX"u8);

        var cap2 = CaptureBuilder.BuildReflectionCap(1_000, new byte[] { 6, 7 });
        var real = CaptureBuilder.WrapBt(cap2, 0x0010, "color_reflection_run",
                                         CaptureProtocol.BTCE);

        var combined = new byte[fake.Length + real.Length];
        fake.CopyTo(combined.AsSpan());
        real.CopyTo(combined.AsSpan(fake.Length));

        SessionScanner.TryScan(combined, ResolveKnown, out var scan).Should().BeTrue();
        scan!.Capture.RecordCount.Should().Be(2);
        scan.Capture.StartTimestampUs.Should().Be(1_000ul);
    }
}
