using CaptureViewer.Core;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Tests.Fixtures;

using FluentAssertions;

using Xunit;

namespace CaptureViewer.Core.Tests;

public class FieldReaderTests
{
    [Fact]
    public void ReadColumn_PullsReflectionPercentsAcrossRecords()
    {
        var pcts = new byte[] { 12, 34, 56, 78, 99 };
        var cap = CaptureFile.Parse(CaptureBuilder.BuildReflectionCap(0, pcts));

        var col = FieldReader.ReadColumn(cap, "reflection_pct");
        col.Should().Equal(12.0, 34.0, 56.0, 78.0, 99.0);
    }

    [Fact]
    public void ReadColumn_DecodesTimeStampU32()
    {
        var pcts = new byte[] { 1, 1, 1 };
        var cap = CaptureFile.Parse(CaptureBuilder.BuildReflectionCap(
            startTsUs: 0, pcts, tsStrideUs: 50_000));

        var col = FieldReader.ReadColumn(cap, "ts_us");
        col.Should().Equal(0.0, 50_000.0, 100_000.0);
    }

    [Fact]
    public void ReadColumn_UnknownField_Throws()
    {
        var cap = CaptureFile.Parse(CaptureBuilder.BuildReflectionCap(
            0, new byte[] { 1 }));

        var act = () => FieldReader.ReadColumn(cap, "no_such_field");
        act.Should().Throw<ArgumentException>()
           .WithMessage("*not present in schema*");
    }

    [Fact]
    public void ReadScaledDouble_AppliesScaleLog10()
    {
        var f = new FieldDescriptor("v", FieldType.I32, 0, 4, -3, "mg");
        var rec = new byte[4];
        System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(rec, 1500);

        FieldReader.ReadScaledDouble(rec, f).Should().BeApproximately(1.5, 1e-9);
    }
}
