using System.Numerics;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Tracks whether the IMU has been stationary for a configurable consecutive
/// run of samples. "Stationary" requires both the accelerometer norm to be
/// near 1 g and every gyro axis below a small threshold; transient noise on
/// either input resets the run.
/// </summary>
public sealed class StationaryDetector
{
    /// <summary>Allowed deviation of |accel| from 1 g.</summary>
    public float AccelEpsilonG { get; init; } = 0.05f;

    /// <summary>
    /// Per-axis gyro magnitude limit, in dps. Generous enough that the
    /// LSM6DSL's uncalibrated ZRL (typically a few dps per axis) still
    /// passes — otherwise the auto-bias LPF cannot bootstrap on its own.
    /// Real intentional motion is typically &gt;10 dps, so this still
    /// rejects rotation reliably. Settable so the GUI can tune it for the
    /// individual chip and operating environment.
    /// </summary>
    public float GyroEpsilonDps { get; set; } = 5.0f;

    /// <summary>Number of consecutive stationary samples needed before
    /// <see cref="IsStationary"/> latches true. Default is ~0.5 s at 833 Hz.</summary>
    public int RequiredConsecutiveSamples { get; init; } = 416;

    private int _consecutive;

    public int ConsecutiveStationarySamples => _consecutive;

    public bool IsStationary => _consecutive >= RequiredConsecutiveSamples;

    public bool Submit(Vector3 accelG, Vector3 gyroDps)
    {
        float aDelta = MathF.Abs(accelG.Length() - 1f);
        bool sampleStationary =
            aDelta < AccelEpsilonG &&
            MathF.Abs(gyroDps.X) < GyroEpsilonDps &&
            MathF.Abs(gyroDps.Y) < GyroEpsilonDps &&
            MathF.Abs(gyroDps.Z) < GyroEpsilonDps;

        if (sampleStationary)
        {
            // Cap to avoid overflow in long stationary periods.
            if (_consecutive < int.MaxValue)
            {
                _consecutive++;
            }
        }
        else
        {
            _consecutive = 0;
        }
        return IsStationary;
    }

    public void Reset() => _consecutive = 0;
}
