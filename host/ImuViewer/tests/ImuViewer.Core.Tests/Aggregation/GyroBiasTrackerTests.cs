using System.Numerics;
using FluentAssertions;
using ImuViewer.Core.Aggregation;
using Xunit;

namespace ImuViewer.Core.Tests.Aggregation;

public class GyroBiasTrackerTests
{
    private static GyroBiasTracker NewTracker(int requiredStationarySamples = 1, float tau = 1f)
    {
        StationaryDetector det = new()
        {
            AccelEpsilonG = 0.05f,
            GyroEpsilonDps = 50f,
            RequiredConsecutiveSamples = requiredStationarySamples,
        };
        return new GyroBiasTracker(det) { TauSeconds = tau };
    }

    [Fact]
    public void Subtracts_seeded_bias_immediately()
    {
        GyroBiasTracker t = NewTracker();
        t.SetBias(new Vector3(0.1f, 0.2f, 0.3f));

        // Raw gyro of 1 rad/s on Z = ~57 dps; bias-corrected = ~57 - 17 = ~40 dps,
        // exceeding the 50 dps detector threshold? Let's pick a clearly-moving
        // input: raw 2 rad/s = ~114 dps; corrected ~ 97 dps, well above 50.
        Vector3 corrected = t.Update(
            new Vector3(0, 0, 1f),
            new Vector3(0, 0, 2f),
            dt: 0.01f);
        corrected.Should().Be(new Vector3(0f - 0.1f, 0f - 0.2f, 2f - 0.3f));
        t.BiasRadS.Should().Be(new Vector3(0.1f, 0.2f, 0.3f));
    }

    [Fact]
    public void Lpf_drives_bias_toward_live_gyro_when_stationary()
    {
        // τ = 1 s, dt = 1/100 s → α = 0.01 per sample. 200 samples ≈ 87% to target.
        GyroBiasTracker t = NewTracker(requiredStationarySamples: 1, tau: 1f);

        // Target raw gyro = 0.1 rad/s (~5.7 dps) — small enough that even with
        // bias = 0 initially, the corrected reading stays under 50 dps so the
        // detector latches stationary.
        Vector3 target = new(0.1f, -0.05f, 0.02f);
        for (int i = 0; i < 200; i++)
        {
            t.Update(new Vector3(0, 0, 1f), target, dt: 1f / 100f);
        }

        t.IsStationary.Should().BeTrue();
        t.BiasRadS.X.Should().BeApproximately(target.X, 0.02f);
        t.BiasRadS.Y.Should().BeApproximately(target.Y, 0.02f);
        t.BiasRadS.Z.Should().BeApproximately(target.Z, 0.02f);
    }

    [Fact]
    public void Bias_held_during_motion()
    {
        GyroBiasTracker t = NewTracker(requiredStationarySamples: 1, tau: 0.1f);
        t.SetBias(new Vector3(0.1f, 0, 0));

        Vector3 startBias = t.BiasRadS;
        for (int i = 0; i < 50; i++)
        {
            // Raw 2 rad/s → corrected ~ (2 - 0.1) = 1.9 rad/s ≈ 109 dps, well above
            // the 50 dps detector threshold.
            t.Update(new Vector3(0, 0, 1f), new Vector3(2f, 0, 0), dt: 0.01f);
        }
        t.IsStationary.Should().BeFalse();
        t.BiasRadS.Should().Be(startBias);
    }

    [Fact]
    public void Detection_uses_corrected_gyro_so_a_seeded_bias_lets_lpf_engage()
    {
        // Raw gyro = 0.5 rad/s (~28 dps) is *above* the 25 dps threshold without
        // bias, so detection would normally reject it. With bias seeded to the
        // matching value the corrected reading is zero and detection latches
        // immediately, so the LPF can refine the bias further.
        StationaryDetector det = new()
        {
            AccelEpsilonG = 0.05f,
            GyroEpsilonDps = 25f,
            RequiredConsecutiveSamples = 1,
        };
        GyroBiasTracker t = new(det) { TauSeconds = 1f };
        t.SetBias(new Vector3(0, 0, 0.5f));

        t.Update(new Vector3(0, 0, 1f), new Vector3(0, 0, 0.5f), dt: 0.01f);
        t.IsStationary.Should().BeTrue();
    }

    [Fact]
    public void Reset_clears_bias_and_detector_state()
    {
        GyroBiasTracker t = NewTracker(requiredStationarySamples: 1);
        t.SetBias(new Vector3(0.5f, 0.5f, 0.5f));
        t.Update(new Vector3(0, 0, 1f), new Vector3(0.5f, 0.5f, 0.5f), 0.01f);
        t.IsStationary.Should().BeTrue();

        t.Reset();
        t.BiasRadS.Should().Be(Vector3.Zero);
        t.IsStationary.Should().BeFalse();
    }
}
