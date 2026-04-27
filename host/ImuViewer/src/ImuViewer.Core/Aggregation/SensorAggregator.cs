using System.Numerics;
using System.Threading.Channels;
using ImuViewer.Core.Coordinates;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Converts btsensor frames into world-frame <see cref="AggregatedSample"/>
/// instances and hands them off to the consumer one-by-one. Each LSM6DSL raw
/// sample becomes one <see cref="AggregatedSample"/> with the absolute
/// timestamp recovered from <c>first_sample_ts_us + ts_delta_us</c>; the
/// orientation filter then integrates at chip ODR with the actual sample
/// spacing as <c>dt</c>, instead of averaging at the UI tick rate.
/// </summary>
/// <remarks>
/// Single writer (Bluetooth reader thread) → single reader (UI tick thread).
/// Backed by an unbounded <see cref="Channel{T}"/>; under realistic conditions
/// the queue holds at most a few frames worth of samples (~13 × 64 = 832/s)
/// and the UI tick drains it every ~16 ms.
/// </remarks>
public sealed class SensorAggregator
{
    private const float DegToRad = MathF.PI / 180f;

    private readonly ChipToWorldTransform _transform;
    private readonly Channel<AggregatedSample> _channel;
    private AggregatedSample? _latest;

    public SensorAggregator(ChipToWorldTransform? transform = null)
    {
        _transform = transform ?? ChipToWorldTransform.Identity;
        _channel = Channel.CreateUnbounded<AggregatedSample>(
            new UnboundedChannelOptions { SingleReader = true, SingleWriter = true });
    }

    /// <summary>Most recently emitted sample; useful for non-consuming display.</summary>
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
        uint baseTs = frame.Header.FirstSampleTimestampUs;

        for (int i = 0; i < n; i++)
        {
            ImuSample s = frame.Samples[i];
            uint absTs = unchecked(baseTs + s.TimestampDeltaUs);

            Vector3 accelChip = new(
                ScaleFactors.AccelG(s.RawAx, accelFsr),
                ScaleFactors.AccelG(s.RawAy, accelFsr),
                ScaleFactors.AccelG(s.RawAz, accelFsr));
            Vector3 gyroDpsChip = new(
                ScaleFactors.GyroDps(s.RawGx, gyroFsr),
                ScaleFactors.GyroDps(s.RawGy, gyroFsr),
                ScaleFactors.GyroDps(s.RawGz, gyroFsr));

            Vector3 accelWorld = _transform.Apply(accelChip);
            Vector3 gyroDpsWorld = _transform.Apply(gyroDpsChip);
            Vector3 gyroRadSWorld = gyroDpsWorld * DegToRad;

            AggregatedSample sample = new(accelWorld, gyroRadSWorld, gyroDpsWorld, absTs);

            // Channel is unbounded → TryWrite always succeeds.
            _channel.Writer.TryWrite(sample);
            Volatile.Write(ref _latest, sample);
        }
    }

    /// <summary>
    /// Pulls the next pending sample. Callers should drain in a while-loop
    /// from the UI tick to keep up with the chip ODR (~833 Hz).
    /// </summary>
    public bool TryRead(out AggregatedSample sample)
    {
        return _channel.Reader.TryRead(out sample!);
    }
}
