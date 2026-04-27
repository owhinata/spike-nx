using System.Collections.Immutable;
using FluentAssertions;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Aggregation;

public class SensorAggregatorTests
{
    private static ImuFrame BuildFrame(
        ushort accelFsr,
        ushort gyroFsr,
        uint firstTsUs,
        params (short ax, short ay, short az, short gx, short gy, short gz, uint dt)[] samples)
    {
        ImuFrameHeader header = new()
        {
            Magic = WireConstants.Magic,
            Type = WireConstants.ImuFrameType,
            SampleCount = (byte)samples.Length,
            SampleRateHz = 833,
            AccelFsrG = accelFsr,
            GyroFsrDps = gyroFsr,
            Seq = 1,
            FirstSampleTimestampUs = firstTsUs,
            FrameLen = (ushort)(WireConstants.HeaderSize + samples.Length * WireConstants.SampleSize),
        };
        ImuSample[] xs = new ImuSample[samples.Length];
        for (int i = 0; i < samples.Length; i++)
        {
            (short ax, short ay, short az, short gx, short gy, short gz, uint dt) = samples[i];
            xs[i] = new ImuSample(ax, ay, az, gx, gy, gz, dt);
        }
        return new ImuFrame(header, [.. xs]);
    }

    [Fact]
    public void Each_raw_sample_is_emitted_individually_with_absolute_timestamp()
    {
        // FSR=8g, raw=4096 -> 1.0 g; raw=8192 -> 2.0 g.
        ImuFrame frame = BuildFrame(
            accelFsr: 8,
            gyroFsr: 2000,
            firstTsUs: 1_000_000u,
            samples: [
                (4096, 0, 0, 0, 0, 0, 0u),
                (8192, 0, 0, 0, 0, 0, 1200u),
                (-4096, 0, 0, 0, 0, 0, 2400u),
            ]);

        SensorAggregator agg = new();
        agg.OnFrame(frame);

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
        ImuFrame frame = BuildFrame(
            accelFsr: 8,
            gyroFsr: 2000,
            firstTsUs: 0,
            samples: [(0, 0, 0, 0, 0, 1000, 0u)]);

        SensorAggregator agg = new();
        agg.OnFrame(frame);

        agg.TryRead(out AggregatedSample s).Should().BeTrue();
        s.GyroDps.Z.Should().BeApproximately(70f, 1e-3f);
        s.GyroRadS.Z.Should().BeApproximately(70f * MathF.PI / 180f, 1e-3f);
    }

    [Fact]
    public void Latest_tracks_most_recently_emitted_sample()
    {
        SensorAggregator agg = new();
        agg.Latest.Should().BeNull();

        agg.OnFrame(BuildFrame(8, 2000, 0u, [(4096, 0, 0, 0, 0, 0, 0u)]));
        agg.OnFrame(BuildFrame(8, 2000, 1_000_000u, [(8192, 0, 0, 0, 0, 0, 0u)]));

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

        agg.OnFrame(BuildFrame(8, 2000, 0u, [
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
    public void Empty_frames_are_ignored()
    {
        SensorAggregator agg = new();
        ImuFrameHeader header = new()
        {
            Magic = WireConstants.Magic,
            Type = WireConstants.ImuFrameType,
            SampleCount = 0,
            SampleRateHz = 833,
            AccelFsrG = 8,
            GyroFsrDps = 2000,
            Seq = 1,
            FirstSampleTimestampUs = 0,
            FrameLen = (ushort)WireConstants.HeaderSize,
        };
        agg.OnFrame(new ImuFrame(header, ImmutableArray<ImuSample>.Empty));
        agg.TryRead(out _).Should().BeFalse();
        agg.Latest.Should().BeNull();
    }
}
