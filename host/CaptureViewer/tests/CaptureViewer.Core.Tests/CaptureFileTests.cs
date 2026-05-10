using System.Buffers.Binary;

using CaptureViewer.Core;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;
using CaptureViewer.Core.Tests.Fixtures;

using FluentAssertions;

using Xunit;

namespace CaptureViewer.Core.Tests;

public class CaptureFileTests
{
    [Fact]
    public void Parse_RoundTripsFileHeaderAndFieldDescriptors()
    {
        var bytes = CaptureBuilder.BuildReflectionCap(
            startTsUs: 12_345_678,
            reflectionPercents: new byte[] { 30, 40, 50, 60 });

        var cap = CaptureFile.Parse(bytes);

        cap.Magic.Should().Be(CaptureProtocol.FileMagic);
        cap.Version.Should().Be(CaptureProtocol.FileVersion);
        cap.SchemaMagic.Should().Be((ushort)0x0010);
        cap.SchemaName.Should().Be("color_reflection_run");
        cap.RecordSize.Should().Be(5);
        cap.RecordCount.Should().Be(4);
        cap.StartTimestampUs.Should().Be(12_345_678ul);
        cap.Termination.Should().Be(CaptureTermination.Unknown);

        cap.Fields.Should().HaveCount(2);
        cap.Fields[0].Name.Should().Be("ts_us");
        cap.Fields[0].Type.Should().Be(FieldType.U32);
        cap.Fields[0].Offset.Should().Be(0);
        cap.Fields[0].Size.Should().Be(4);
        cap.Fields[0].Unit.Should().Be("us");
        cap.Fields[1].Name.Should().Be("reflection_pct");
        cap.Fields[1].Type.Should().Be(FieldType.U8);
        cap.Fields[1].Offset.Should().Be(4);
        cap.Fields[1].Size.Should().Be(1);
    }

    [Fact]
    public void Records_ReturnPerIndexSliceMatchingPayload()
    {
        var pcts = new byte[] { 10, 20, 30 };
        var bytes = CaptureBuilder.BuildReflectionCap(0, pcts, tsStrideUs: 5_000);

        var cap = CaptureFile.Parse(bytes);

        for (var i = 0; i < pcts.Length; i++)
        {
            var rec = cap.Records(i).Span;
            rec.Length.Should().Be(5);
            BinaryPrimitives.ReadUInt32LittleEndian(rec[..4]).Should().Be((uint)(i * 5_000));
            rec[4].Should().Be(pcts[i]);
        }
    }

    [Fact]
    public void Records_OutOfRangeIndexThrows()
    {
        var bytes = CaptureBuilder.BuildReflectionCap(0, new byte[] { 1 });
        var cap = CaptureFile.Parse(bytes);

        var act = () => cap.Records(1);
        act.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Fact]
    public void Parse_WrongMagic_Throws()
    {
        var bytes = CaptureBuilder.BuildReflectionCap(0, new byte[] { 1 });
        bytes[0] = 0xFF;  // corrupt the magic byte

        var act = () => CaptureFile.Parse(bytes);
        act.Should().Throw<InvalidCaptureFileException>()
           .WithMessage("*Bad magic*");
    }

    [Fact]
    public void Parse_BufferShorterThanHeader_Throws()
    {
        var act = () => CaptureFile.Parse(new byte[10]);
        act.Should().Throw<InvalidCaptureFileException>()
           .WithMessage("*too small*");
    }

    [Fact]
    public void Parse_TruncatedPayload_Throws()
    {
        var bytes = CaptureBuilder.BuildReflectionCap(0, new byte[] { 1, 2, 3 });
        // Lop off the trailing record so header still claims 3 records
        // but only ~2 fit.
        var truncated = bytes.AsSpan(0, bytes.Length - 5).ToArray();

        var act = () => CaptureFile.Parse(truncated);
        act.Should().Throw<InvalidCaptureFileException>()
           .WithMessage("*Truncated*");
    }

    [Fact]
    public void GeneratedSchema_Parse_DecodesRecord()
    {
        // Pull the second record out of a synthesized .cap and run it
        // through the codegen-emitted parser.  This is the contract
        // between the codegen tool and the host parser.
        var pcts = new byte[] { 12, 73, 200 };
        var cap = CaptureFile.Parse(CaptureBuilder.BuildReflectionCap(
            0, pcts, tsStrideUs: 100_000));

        var rec = SchemaColorReflectionRun.Parse(cap.Records(1).Span);
        rec.ts_us.Should().Be(100_000u);
        rec.reflection_pct.Should().Be(73);
    }

    [Fact]
    public void KnownSchemas_ResolvesByMagic()
    {
        KnownSchemas.ByMagic.Should().ContainKey((ushort)0x0010);
        KnownSchemas.TryGet(0x0010)!.Name.Should().Be("color_reflection_run");
        KnownSchemas.TryGet(0x0011)!.Name.Should().Be("color_rgbi_run");
        KnownSchemas.TryGet(0xFFFF).Should().BeNull();
    }
}
