using System.Numerics;
using ImuViewer.Core.Coordinates;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Averages all samples in a btsensor frame into a single g/rad-per-second sample
/// for the orientation filter, applying the chip-to-world mount transform.
/// Thread-safe for one writer (Bluetooth reader) and many readers (UI tick).
/// </summary>
public sealed class SensorAggregator
{
    private const float DegToRad = MathF.PI / 180f;

    private readonly ChipToWorldTransform _transform;
    private long _seq;
    private long _lastConsumedSeq;
    private AggregatedSample? _latest;

    public SensorAggregator(ChipToWorldTransform? transform = null)
    {
        _transform = transform ?? ChipToWorldTransform.Identity;
    }

    public AggregatedSample? Latest => Volatile.Read(ref _latest);

    public void OnFrame(ImuFrame frame)
    {
        int n = frame.Samples.Length;
        if (n <= 0)
        {
            return;
        }

        int accelFsr = frame.Header.AccelFsrG;
        int gyroFsr = frame.Header.GyroFsrDps;

        Vector3 sumAccelG = Vector3.Zero;
        Vector3 sumGyroDps = Vector3.Zero;

        foreach (ImuSample s in frame.Samples)
        {
            sumAccelG += new Vector3(
                ScaleFactors.AccelG(s.RawAx, accelFsr),
                ScaleFactors.AccelG(s.RawAy, accelFsr),
                ScaleFactors.AccelG(s.RawAz, accelFsr));
            sumGyroDps += new Vector3(
                ScaleFactors.GyroDps(s.RawGx, gyroFsr),
                ScaleFactors.GyroDps(s.RawGy, gyroFsr),
                ScaleFactors.GyroDps(s.RawGz, gyroFsr));
        }

        Vector3 meanAccelChip = sumAccelG / n;
        Vector3 meanGyroDpsChip = sumGyroDps / n;

        Vector3 accelWorld = _transform.Apply(meanAccelChip);
        Vector3 gyroDpsWorld = _transform.Apply(meanGyroDpsChip);
        Vector3 gyroRadSWorld = gyroDpsWorld * DegToRad;

        long seq = Interlocked.Increment(ref _seq);
        AggregatedSample sample = new(
            accelWorld,
            gyroRadSWorld,
            gyroDpsWorld,
            frame.Header.FirstSampleTimestampUs,
            n,
            seq);
        Volatile.Write(ref _latest, sample);
    }

    /// <summary>
    /// Returns the latest aggregated sample if it has not been consumed yet.
    /// Used by the UI/filter tick to avoid integrating the same sample twice.
    /// </summary>
    public bool TryConsumeNext(out AggregatedSample sample)
    {
        AggregatedSample? latest = Volatile.Read(ref _latest);
        long lastConsumed = Interlocked.Read(ref _lastConsumedSeq);
        if (latest is null || latest.Seq <= lastConsumed)
        {
            sample = null!;
            return false;
        }
        Interlocked.Exchange(ref _lastConsumedSeq, latest.Seq);
        sample = latest;
        return true;
    }
}
