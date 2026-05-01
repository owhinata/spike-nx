using System.Collections.Immutable;
using FluentAssertions;
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.LegoSensor;

public class LegoSampleAggregatorTests
{
    private static LegoTlv UnboundTlv(LegoClassId classId) => new(
        ClassId: classId,
        PortId: 0xFF,
        ModeId: 0,
        DataType: LegoDataType.Int8,
        NumValues: 0,
        Flags: LegoTlvFlags.None,
        Age10ms: 0xFF,
        Seq: 0,
        Payload: ImmutableArray<byte>.Empty);

    private static LegoTlv BoundTlv(LegoClassId classId, byte port, byte mode,
        LegoDataType dataType, byte numValues, byte[] payload, bool fresh = true,
        ushort seq = 1, byte age10ms = 0)
    {
        LegoTlvFlags flags = LegoTlvFlags.Bound | (fresh ? LegoTlvFlags.Fresh : LegoTlvFlags.None);
        return new(
            ClassId: classId,
            PortId: port,
            ModeId: mode,
            DataType: dataType,
            NumValues: numValues,
            Flags: flags,
            Age10ms: age10ms,
            Seq: seq,
            Payload: payload.Length == 0
                ? ImmutableArray<byte>.Empty
                : ImmutableArray.Create(payload));
    }

    private static ImmutableArray<LegoTlv> AllUnboundExcept(
        LegoClassId classId, LegoTlv tlv) =>
        ImmutableArray.CreateRange(Enum.GetValues<LegoClassId>()
            .Select(id => id == classId ? tlv : UnboundTlv(id)));

    [Fact]
    public void Six_buckets_initialized_unbound()
    {
        LegoSampleAggregator agg = new();
        agg.State.Should().HaveCount(6);
        foreach (LegoClassId id in Enum.GetValues<LegoClassId>())
        {
            agg.State[id].IsBound.Should().BeFalse();
            agg.State[id].PortId.Should().BeNull();
            agg.State[id].LastSample.Should().BeNull();
        }
    }

    [Fact]
    public void Fresh_TLV_fires_SampleReceived_and_StatusChanged()
    {
        LegoSampleAggregator agg = new();
        List<LegoSamplePoint> samples = [];
        List<LegoClassId> statusFires = [];
        agg.SampleReceived += (id, s) => { if (id == LegoClassId.Color) samples.Add(s); };
        agg.StatusChanged += (id, _) => statusFires.Add(id);

        // RGB I mode 5: 4 × INT16
        byte[] rgb = { 0x10, 0x00, 0x20, 0x00, 0x30, 0x00, 0x40, 0x00 };
        ImmutableArray<LegoTlv> tlvs = AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, port: 4, mode: 5, LegoDataType.Int16,
                     numValues: 4, rgb));

        agg.OnTlvBatch(tickTsUs: 1000, tlvs);

        statusFires.Should().Contain(LegoClassId.Color);
        samples.Should().ContainSingle();
        samples[0].Values.Should().Equal(16f, 32f, 48f, 64f);
        samples[0].Label.Should().Be("RGB I");
        agg.State[LegoClassId.Color].IsBound.Should().BeTrue();
        agg.State[LegoClassId.Color].PortId.Should().Be(4);
        agg.State[LegoClassId.Color].ModeId.Should().Be(5);
    }

    [Fact]
    public void NotFresh_TLV_holds_last_known_values()
    {
        LegoSampleAggregator agg = new();
        List<LegoSamplePoint> samples = [];
        agg.SampleReceived += (id, s) => { if (id == LegoClassId.Color) samples.Add(s); };

        // First tick: FRESH with payload.
        byte[] payload = { 0x10, 0x00 };
        agg.OnTlvBatch(100, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, 0, 0, LegoDataType.Int16, 1, payload)));
        samples.Should().HaveCount(1);

        // Second tick: BOUND but not FRESH (no payload) → no new sample.
        agg.OnTlvBatch(110, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, 0, 0, LegoDataType.Int16, 1,
                     Array.Empty<byte>(), fresh: false)));
        samples.Should().HaveCount(1);   // no new sample
        agg.State[LegoClassId.Color].LastSample.Should().NotBeNull();
        agg.State[LegoClassId.Color].LastSample!.Values[0].Should().Be(16f);
    }

    [Fact]
    public void Port_or_mode_change_fires_PortChanged()
    {
        LegoSampleAggregator agg = new();
        List<LegoClassId> portChangedFires = [];
        agg.PortChanged += (id, _) => portChangedFires.Add(id);

        byte[] p = { 0x01 };
        agg.OnTlvBatch(0, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, port: 0, mode: 0, LegoDataType.Int8, 1, p)));
        portChangedFires.Should().BeEmpty(); // first bind; not "changed"

        // Same port/mode again — no PortChanged.
        agg.OnTlvBatch(10, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, port: 0, mode: 0, LegoDataType.Int8, 1, p)));
        portChangedFires.Should().BeEmpty();

        // Same port, different mode — fires.
        agg.OnTlvBatch(20, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, port: 0, mode: 1, LegoDataType.Int8, 1, p)));
        portChangedFires.Should().Contain(LegoClassId.Color);

        // Different port, same mode — fires again.
        portChangedFires.Clear();
        agg.OnTlvBatch(30, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, port: 2, mode: 1, LegoDataType.Int8, 1, p)));
        portChangedFires.Should().Contain(LegoClassId.Color);
    }

    [Fact]
    public void Many_unbound_bundles_flips_IsBound_to_false()
    {
        LegoSampleAggregator agg = new() { UnboundAfterMissedBundles = 3 };
        List<bool> boundHistory = [];
        agg.StatusChanged += (id, st) =>
        {
            if (id == LegoClassId.Color) boundHistory.Add(st.IsBound);
        };

        // Bound once.
        agg.OnTlvBatch(0, AllUnboundExcept(LegoClassId.Color,
            BoundTlv(LegoClassId.Color, 0, 0, LegoDataType.Int8, 1, new byte[] { 0 })));
        boundHistory.Should().Equal(true);

        // 3 consecutive unbound bundles — should flip false.
        boundHistory.Clear();
        for (int i = 0; i < 3; i++)
        {
            agg.OnTlvBatch((uint)(10 * (i + 1)), ImmutableArray.CreateRange(
                Enum.GetValues<LegoClassId>().Select(UnboundTlv)));
        }
        boundHistory.Should().Contain(false);
        agg.State[LegoClassId.Color].IsBound.Should().BeFalse();
    }
}
