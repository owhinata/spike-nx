using System.Numerics;
using FluentAssertions;
using ImuViewer.Core.Aggregation;
using Xunit;

namespace ImuViewer.Core.Tests.Aggregation;

public class StationaryDetectorTests
{
    [Fact]
    public void Latches_after_required_consecutive_stationary_samples()
    {
        StationaryDetector det = new()
        {
            AccelEpsilonG = 0.05f,
            GyroEpsilonDps = 1f,
            RequiredConsecutiveSamples = 5,
        };

        for (int i = 0; i < 4; i++)
        {
            det.Submit(new Vector3(0, 0, 1), Vector3.Zero).Should().BeFalse();
        }
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero).Should().BeTrue();
        det.IsStationary.Should().BeTrue();
    }

    [Fact]
    public void Resets_on_movement_sample()
    {
        StationaryDetector det = new() { RequiredConsecutiveSamples = 3 };
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero);
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero);
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero);
        det.IsStationary.Should().BeTrue();

        // One non-stationary sample drops the run.
        det.Submit(new Vector3(0, 0, 1), new Vector3(50, 0, 0)).Should().BeFalse();
        det.IsStationary.Should().BeFalse();
        det.ConsecutiveStationarySamples.Should().Be(0);
    }

    [Fact]
    public void Accel_norm_far_from_1g_is_treated_as_movement()
    {
        StationaryDetector det = new()
        {
            AccelEpsilonG = 0.05f,
            GyroEpsilonDps = 1f,
            RequiredConsecutiveSamples = 1,
        };
        det.Submit(new Vector3(0, 0, 1.5f), Vector3.Zero).Should().BeFalse();
        det.Submit(new Vector3(0, 0, 0.5f), Vector3.Zero).Should().BeFalse();
        det.Submit(new Vector3(0, 0, 1.0f), Vector3.Zero).Should().BeTrue();
    }

    [Fact]
    public void Reset_clears_run()
    {
        StationaryDetector det = new() { RequiredConsecutiveSamples = 2 };
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero);
        det.Submit(new Vector3(0, 0, 1), Vector3.Zero);
        det.IsStationary.Should().BeTrue();

        det.Reset();
        det.IsStationary.Should().BeFalse();
        det.ConsecutiveStationarySamples.Should().Be(0);
    }
}
