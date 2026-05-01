using FluentAssertions;
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.LegoSensor;

public class ScaleTablesTests
{
    [Fact]
    public void Color_mode6_HSV_decodes_3xI16()
    {
        // H=180, S=50, V=75 as INT16 little-endian.
        byte[] payload =
        {
            0xB4, 0x00, // 180
            0x32, 0x00, // 50
            0x4B, 0x00, // 75
        };
        (string label, IReadOnlyList<float> values, IReadOnlyList<string> units) =
            ScaleTables.Decode(LegoClassId.Color, modeId: 6,
                LegoDataType.Int16, numValues: 3, payload);

        label.Should().Be("HSV");
        values.Should().HaveCount(3);
        values[0].Should().Be(180);
        values[1].Should().Be(50);
        values[2].Should().Be(75);
        units[0].Should().Be("°");
    }

    [Fact]
    public void Ultrasonic_mode0_distance_decodes_int16_mm()
    {
        // 312 mm
        byte[] payload = { 0x38, 0x01 };
        (string label, IReadOnlyList<float> values, IReadOnlyList<string> units) =
            ScaleTables.Decode(LegoClassId.Ultrasonic, modeId: 0,
                LegoDataType.Int16, numValues: 1, payload);

        label.Should().Be("DISTL");
        values.Should().ContainSingle().Which.Should().Be(312f);
        units[0].Should().Be("mm");
    }

    [Fact]
    public void Force_mode0_decodes_signed_byte()
    {
        // -42 as INT8 → encoded as 0xD6
        byte[] payload = { 0xD6 };
        (string label, IReadOnlyList<float> values, _) =
            ScaleTables.Decode(LegoClassId.Force, modeId: 0,
                LegoDataType.Int8, numValues: 1, payload);

        label.Should().Be("FORCE");
        values.Should().ContainSingle().Which.Should().Be(-42f);
    }

    [Fact]
    public void MotorR_mode2_position_decodes_int32_degrees()
    {
        // -180000 as INT32 LE: 0xFFFD40E0
        byte[] payload =
        {
            0xE0, 0x40, 0xFD, 0xFF,
        };
        (string label, IReadOnlyList<float> values, IReadOnlyList<string> units) =
            ScaleTables.Decode(LegoClassId.MotorR, modeId: 2,
                LegoDataType.Int32, numValues: 1, payload);

        label.Should().Be("POS");
        values.Should().ContainSingle().Which.Should().Be(-180000f);
        units[0].Should().Be("°");
    }

    [Fact]
    public void Unknown_mode_falls_back_to_raw()
    {
        byte[] payload = { 0x10, 0x20, 0x30 };
        (string label, IReadOnlyList<float> values, IReadOnlyList<string> units) =
            ScaleTables.Decode(LegoClassId.Color, modeId: 99,
                LegoDataType.Int8, numValues: 3, payload);

        label.Should().Contain("99");
        values.Should().HaveCount(3);
        values[0].Should().Be(0x10);
        values[1].Should().Be(0x20);
        values[2].Should().Be(0x30);
        units.Should().AllSatisfy(u => u.Should().BeEmpty());
    }

    [Fact]
    public void Channel_labels_returns_named_channels_for_known_mode()
    {
        IReadOnlyList<string> names = ScaleTables.ChannelLabels(
            LegoClassId.Color, modeId: 5, fallbackCount: 4);
        names.Should().Equal("R", "G", "B", "IR");
    }

    [Fact]
    public void Channel_labels_falls_back_when_unknown()
    {
        IReadOnlyList<string> names = ScaleTables.ChannelLabels(
            LegoClassId.Color, modeId: 99, fallbackCount: 3);
        names.Should().Equal("ch0", "ch1", "ch2");
    }
}
