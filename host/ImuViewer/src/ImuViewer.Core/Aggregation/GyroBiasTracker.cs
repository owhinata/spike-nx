using System.Numerics;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Maintains a running per-axis gyroscope bias estimate. While the IMU is
/// stationary (per <see cref="StationaryDetector"/>) the bias is pushed
/// toward the live gyro reading via a first-order LPF with time constant
/// <see cref="TauSeconds"/>; while the IMU is moving the bias is held.
/// Manual calibration can seed the bias directly via <see cref="SetBias"/>.
/// </summary>
public sealed class GyroBiasTracker
{
    private readonly StationaryDetector _detector;

    private Vector3 _bias;

    public GyroBiasTracker(StationaryDetector? detector = null)
    {
        _detector = detector ?? new StationaryDetector();
    }

    /// <summary>Time constant of the LPF used when stationary.</summary>
    public float TauSeconds { get; set; } = 30f;

    public Vector3 BiasRadS => _bias;

    public bool IsStationary => _detector.IsStationary;

    public int ConsecutiveStationarySamples => _detector.ConsecutiveStationarySamples;

    public StationaryDetector Detector => _detector;

    /// <summary>
    /// Feeds one sample. Returns the bias-corrected gyro reading in rad/s.
    /// </summary>
    /// <remarks>
    /// Detection runs against the bias-corrected gyro (in dps), not the raw
    /// reading. With the LSM6DSL's modest ZRL the corrected magnitude is well
    /// under the threshold once bias is roughly known, so detection latches
    /// reliably; using raw gyro instead would let an uncompensated DC offset
    /// (e.g. the value Calibrate gyro is meant to absorb) keep the detector
    /// permanently in the "moving" state.
    /// </remarks>
    public Vector3 Update(Vector3 accelG, Vector3 gyroRadS, float dt)
    {
        const float radToDps = 180f / MathF.PI;
        Vector3 correctedRadS = gyroRadS - _bias;
        Vector3 correctedDps = correctedRadS * radToDps;
        _detector.Submit(accelG, correctedDps);

        if (_detector.IsStationary && dt > 0f && TauSeconds > 0f)
        {
            float alpha = MathF.Min(dt / TauSeconds, 1f);
            _bias += alpha * (gyroRadS - _bias);
            return gyroRadS - _bias;
        }
        return correctedRadS;
    }

    /// <summary>Replaces the current bias estimate (used by manual calibration).</summary>
    public void SetBias(Vector3 biasRadS) => _bias = biasRadS;

    public void Reset()
    {
        _bias = Vector3.Zero;
        _detector.Reset();
    }
}
