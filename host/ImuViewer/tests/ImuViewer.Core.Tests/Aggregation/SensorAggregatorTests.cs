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
        params (short ax, short ay, short az, short gx, short gy, short gz)[] samples)
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
            FirstSampleTimestampUs = 0u,
            FrameLen = (ushort)(WireConstants.HeaderSize + samples.Length * WireConstants.SampleSize),
        };
        ImuSample[] xs = new ImuSample[samples.Length];
        for (int i = 0; i < samples.Length; i++)
        {
            (short ax, short ay, short az, short gx, short gy, short gz) = samples[i];
            xs[i] = new ImuSample(ax, ay, az, gx, gy, gz, (uint)i);
        }
        return new ImuFrame(header, [.. xs]);
    }

    [Fact]
    public void Mean_accel_matches_arithmetic_mean_of_samples()
    {
        // FSR=8g, raw=4096 -> 1.0 g; raw=8192 -> 2.0 g. Mean of (1g, 2g) = 1.5g.
        ImuFrame frame = BuildFrame(
            accelFsr: 8,
            gyroFsr: 2000,
            samples: [
                (4096, 0, 0, 0, 0, 0),
                (8192, 0, 0, 0, 0, 0),
            ]);

        SensorAggregator agg = new();
        agg.OnFrame(frame);

        agg.Latest.Should().NotBeNull();
        agg.Latest!.AccelG.X.Should().BeApproximately(1.5f, 1e-4f);
        agg.Latest.AccelG.Y.Should().Be(0f);
        agg.Latest.AccelG.Z.Should().Be(0f);
    }

    [Fact]
    public void Gyro_is_converted_dps_to_rad_per_second()
    {
        // FSR=2000 dps, raw=1000 -> 70 dps. rad/s = 70 * π/180.
        ImuFrame frame = BuildFrame(
            accelFsr: 8,
            gyroFsr: 2000,
            samples: [(0, 0, 0, 0, 0, 1000)]);

        SensorAggregator agg = new();
        agg.OnFrame(frame);

        agg.Latest!.GyroDps.Z.Should().BeApproximately(70f, 1e-3f);
        agg.Latest.GyroRadS.Z.Should().BeApproximately(70f * MathF.PI / 180f, 1e-3f);
    }

    [Fact]
    public void TryConsumeNext_returns_each_aggregated_sample_exactly_once()
    {
        SensorAggregator agg = new();
        agg.TryConsumeNext(out _).Should().BeFalse();

        agg.OnFrame(BuildFrame(8, 2000, [(0, 0, 0, 0, 0, 0)]));
        agg.TryConsumeNext(out AggregatedSample first).Should().BeTrue();
        first.Seq.Should().Be(1);

        agg.TryConsumeNext(out _).Should().BeFalse();

        agg.OnFrame(BuildFrame(8, 2000, [(0, 0, 0, 0, 0, 0)]));
        agg.TryConsumeNext(out AggregatedSample second).Should().BeTrue();
        second.Seq.Should().Be(2);
    }

    [Fact]
    public void Latest_is_observable_without_consuming()
    {
        SensorAggregator agg = new();
        agg.Latest.Should().BeNull();

        agg.OnFrame(BuildFrame(8, 2000, [(4096, 0, 0, 0, 0, 0)]));

        AggregatedSample? a = agg.Latest;
        AggregatedSample? b = agg.Latest;
        a.Should().NotBeNull();
        b.Should().BeSameAs(a);
    }
}
