using System.Numerics;
using System.Threading.Channels;
using ImuViewer.Core.Calibration;
using ImuViewer.Core.Coordinates;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Converts BUNDLE frames into world-frame <see cref="AggregatedSample"/>
/// instances and hands them off to the consumer one-by-one.  Each LSM6DSL raw
/// sample becomes one <see cref="AggregatedSample"/> with the absolute
/// timestamp recovered from <c>tick_ts_us + ts_delta_us</c>; the orientation
/// filter then integrates at chip ODR with the actual sample spacing as
/// <c>dt</c>, instead of averaging at the UI tick rate.
/// </summary>
/// <remarks>
/// Single writer (Bluetooth reader thread) → single reader (UI tick thread).
/// Backed by an unbounded <see cref="Channel{T}"/>; under realistic conditions
/// the queue holds at most a few frames worth of samples (~833/s in 100 Hz
/// BUNDLEs of 8 samples) and the UI tick drains it every ~16 ms.
/// </remarks>
public sealed class SensorAggregator
{
    private const float DegToRad = MathF.PI / 180f;

    private readonly ChipToWorldTransform _transform;
    private readonly Channel<AggregatedSample> _channel;
    // Reads on the BT-reader thread, writes on the UI thread when the user
    // toggles the cal checkbox / picks a file.  Reference assignment is
    // atomic in CLR; volatile prevents the JIT from caching the field
    // value across OnBundle calls.
    private volatile ImuCalibration? _calibration;
    private AggregatedSample? _latest;
    private bool _calFsrMismatchLogged;

    public SensorAggregator(
        ChipToWorldTransform? transform = null,
        ImuCalibration? calibration = null)
    {
        _transform = transform ?? ChipToWorldTransform.Identity;
        _calibration = calibration;
        _channel = Channel.CreateUnbounded<AggregatedSample>(
            new UnboundedChannelOptions { SingleReader = true, SingleWriter = true });
    }

    /// <summary>
    /// Active offline calibration, or <c>null</c> for raw (FSR-scaled only).
    /// May be swapped at runtime from the UI thread.
    /// </summary>
    public ImuCalibration? Calibration
    {
        get => _calibration;
        set
        {
            _calibration = value;
            // Re-arm FSR mismatch logging so a freshly swapped cal can
            // warn about its own FSR header mismatch.
            _calFsrMismatchLogged = false;
        }
    }

    /// <summary>True iff an offline calibration is currently active.</summary>
    public bool HasCalibration => _calibration is not null;

    /// <summary>Most recently emitted sample; useful for non-consuming display.</summary>
    public AggregatedSample? Latest => Volatile.Read(ref _latest);

    public void OnBundle(BundleFrame frame)
    {
        int n = frame.ImuSamples.Length;
        if (n <= 0)
        {
            return;
        }

        int accelFsr = frame.Header.ImuAccelFsrG;
        int gyroFsr = frame.Header.ImuGyroFsrDps;
        uint baseTs = frame.Header.TickTsUs;

        ImuCalibration? cal = _calibration;
        if (cal is not null &&
            (cal.FsrXlG != accelFsr || cal.FsrGyDps != gyroFsr))
        {
            if (!_calFsrMismatchLogged)
            {
                Console.Error.WriteLine(
                    $"warning: imu_cal FSR mismatch (cal=±{cal.FsrXlG}g/±{cal.FsrGyDps}dps, " +
                    $"bundle=±{accelFsr}g/±{gyroFsr}dps); skipping cal until match.");
                _calFsrMismatchLogged = true;
            }
            cal = null;
        }

        for (int i = 0; i < n; i++)
        {
            ImuSample s = frame.ImuSamples[i];
            uint absTs = unchecked(baseTs + s.TimestampDeltaUs);

            Vector3 accelChip, gyroDpsChip;
            if (cal is not null)
            {
                Vector3 accelLsb = cal.ApplyAccel(s.RawAx, s.RawAy, s.RawAz);
                Vector3 gyroLsb = cal.ApplyGyro(s.RawGx, s.RawGy, s.RawGz);
                accelChip = new(
                    ScaleFactors.AccelG(accelLsb.X, accelFsr),
                    ScaleFactors.AccelG(accelLsb.Y, accelFsr),
                    ScaleFactors.AccelG(accelLsb.Z, accelFsr));
                gyroDpsChip = new(
                    ScaleFactors.GyroDps(gyroLsb.X, gyroFsr),
                    ScaleFactors.GyroDps(gyroLsb.Y, gyroFsr),
                    ScaleFactors.GyroDps(gyroLsb.Z, gyroFsr));
            }
            else
            {
                accelChip = new(
                    ScaleFactors.AccelG(s.RawAx, accelFsr),
                    ScaleFactors.AccelG(s.RawAy, accelFsr),
                    ScaleFactors.AccelG(s.RawAz, accelFsr));
                gyroDpsChip = new(
                    ScaleFactors.GyroDps(s.RawGx, gyroFsr),
                    ScaleFactors.GyroDps(s.RawGy, gyroFsr),
                    ScaleFactors.GyroDps(s.RawGz, gyroFsr));
            }

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
