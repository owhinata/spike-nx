using System.Collections.Immutable;
using FluentAssertions;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Aggregation;

public class SensorAggregatorTests
{
    private static BundleFrame BuildBundle(
        byte accelFsr,
        ushort gyroFsr,
        uint tickTsUs,
        params (short ax, short ay, short az, short gx, short gy, short gz, uint dt)[] samples)
    {
        BundleFrameHeader header = new(
            Seq: 1,
            TickTsUs: tickTsUs,
            ImuSectionLen: (ushort)(samples.Length * WireConstants.ImuSampleSize),
            ImuSampleCount: (byte)samples.Length,
            TlvCount: WireConstants.TlvCount,
            ImuSampleRateHz: 833,
            ImuAccelFsrG: accelFsr,
            ImuGyroFsrDps: gyroFsr,
            Flags: WireConstants.FlagImuOn);

        ImuSample[] xs = new ImuSample[samples.Length];
        for (int i = 0; i < samples.Length; i++)
        {
            (short ax, short ay, short az, short gx, short gy, short gz, uint dt) = samples[i];
            xs[i] = new ImuSample(ax, ay, az, gx, gy, gz, dt);
        }

        // Six unbound TLVs to satisfy TlvCount.
        LegoTlv[] tlvs = new LegoTlv[WireConstants.TlvCount];
        for (int i = 0; i < tlvs.Length; i++)
        {
            tlvs[i] = new LegoTlv(
                ClassId: (LegoClassId)i,
                PortId: 0xFF,
                ModeId: 0,
                DataType: LegoDataType.Int8,
                NumValues: 0,
                Flags: LegoTlvFlags.None,
                Age10ms: 0xFF,
                Seq: 0,
                Payload: ImmutableArray<byte>.Empty);
        }

        return new BundleFrame(header, [.. xs], [.. tlvs]);
    }

    [Fact]
    public void Each_raw_sample_is_emitted_individually_with_absolute_timestamp()
    {
        // FSR=8g, raw=4096 -> 1.0 g; raw=8192 -> 2.0 g.
        BundleFrame frame = BuildBundle(
            accelFsr: 8,
            gyroFsr: 2000,
            tickTsUs: 1_000_000u,
            samples: [
                (4096, 0, 0, 0, 0, 0, 0u),
                (8192, 0, 0, 0, 0, 0, 1200u),
                (-4096, 0, 0, 0, 0, 0, 2400u),
            ]);

        SensorAggregator agg = new();
        agg.OnBundle(frame);

        agg.TryRead(out AggregatedSample s1).Should().BeTrue();
        s1.AccelG.X.Should().BeApproximately(1.0f, 1e-4f);
        s1.TimestampUs.Should().Be(1_000_000u);

        agg.TryRead(out AggregatedSample s2).Should().BeTrue();
        s2.AccelG.X.Should().BeApproximately(2.0f, 1e-4f);
        s2.TimestampUs.Should().Be(1_001_200u);

        agg.TryRead(out AggregatedSample s3).Should().BeTrue();
        s3.AccelG.X.Should().BeApproximately(-1.0f, 1e-4f);
        s3.TimestampUs.Should().Be(1_002_400u);

        agg.TryRead(out _).Should().BeFalse();
    }

    [Fact]
    public void Gyro_is_converted_dps_to_rad_per_second_per_sample()
    {
        // FSR=2000 dps, raw=1000 -> 70 dps. rad/s = 70 * π/180.
        BundleFrame frame = BuildBundle(
            accelFsr: 8,
            gyroFsr: 2000,
            tickTsUs: 0,
            samples: [(0, 0, 0, 0, 0, 1000, 0u)]);

        SensorAggregator agg = new();
        agg.OnBundle(frame);

        agg.TryRead(out AggregatedSample s).Should().BeTrue();
        s.GyroDps.Z.Should().BeApproximately(70f, 1e-3f);
        s.GyroRadS.Z.Should().BeApproximately(70f * MathF.PI / 180f, 1e-3f);
    }

    [Fact]
    public void Latest_tracks_most_recently_emitted_sample()
    {
        SensorAggregator agg = new();
        agg.Latest.Should().BeNull();

        agg.OnBundle(BuildBundle(8, 2000, 0u, [(4096, 0, 0, 0, 0, 0, 0u)]));
        agg.OnBundle(BuildBundle(8, 2000, 1_000_000u, [(8192, 0, 0, 0, 0, 0, 0u)]));

        AggregatedSample? latest = agg.Latest;
        latest.Should().NotBeNull();
        latest!.AccelG.X.Should().BeApproximately(2.0f, 1e-4f);
        latest.TimestampUs.Should().Be(1_000_000u);
    }

    [Fact]
    public void TryRead_drains_each_sample_exactly_once()
    {
        SensorAggregator agg = new();
        agg.TryRead(out _).Should().BeFalse();

        agg.OnBundle(BuildBundle(8, 2000, 0u, [
            (1, 0, 0, 0, 0, 0, 0u),
            (2, 0, 0, 0, 0, 0, 1200u),
        ]));

        agg.TryRead(out AggregatedSample a).Should().BeTrue();
        a.TimestampUs.Should().Be(0u);
        agg.TryRead(out AggregatedSample b).Should().BeTrue();
        b.TimestampUs.Should().Be(1200u);
        agg.TryRead(out _).Should().BeFalse();
    }

    [Fact]
    public void Empty_imu_sections_are_ignored()
    {
        SensorAggregator agg = new();

        // Bundle with 0 IMU samples, all 6 TLVs unbound — the IMU pipeline
        // should be a no-op.
        BundleFrame frame = BuildBundle(8, 2000, 0u);
        agg.OnBundle(frame);

        agg.TryRead(out _).Should().BeFalse();
        agg.Latest.Should().BeNull();
    }
}
