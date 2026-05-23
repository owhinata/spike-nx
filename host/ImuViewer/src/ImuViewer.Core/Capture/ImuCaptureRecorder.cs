using System.Buffers.Binary;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.Capture;

/// <summary>
/// Phase 2.5 / Issue #145: writes the raw 27 B IMU_CAP frames to a
/// `.bin` file as they arrive from <see cref="Btsensor.BtsensorSession"/>
/// and tracks the running stats (frame count, seq drops, elapsed
/// duration, average raw temperature) the UI displays during a capture
/// session.
/// </summary>
/// <remarks>
/// The Tedaldi calibration pipeline (tools/imu_cal/bin_to_ascii.py)
/// reads this file back as a sequence of 27 B frames.  Sessions with
/// <see cref="DropCount"/> > 0 should be rejected by the host UI —
/// the offline pipeline cannot recover lost samples and the imu_tk fit
/// quality degrades sharply with gaps.
///
/// The recorder is not thread-safe: it expects all
/// <see cref="OnImuCapFrame(in ImuCapFrame)"/> calls to come from a
/// single subscriber (typically the BtsensorSession reader thread).
/// The stats are read by the UI tick thread and use simple value reads
/// — momentary inconsistencies between Count and AverageTemperatureRaw
/// are tolerated because they are display-only.
/// </remarks>
public sealed class ImuCaptureRecorder : IDisposable
{
    private readonly Stream _output;
    private readonly bool _ownsStream;
    private readonly byte[] _frameBuffer = new byte[WireConstants.ImuCapFrameSize];
    private readonly DateTime _startedUtc;

    private uint _frameCount;
    private uint _dropCount;
    private long _temperatureSum;
    private ushort _lastSeq;
    private bool _hasPriorSeq;
    private bool _disposed;

    /// <summary>
    /// Create a recorder that writes raw 27 B frames into
    /// <paramref name="output"/>.  When
    /// <paramref name="ownsStream"/> is true the stream is disposed on
    /// <see cref="Dispose"/>; pass false when the caller controls the
    /// stream lifetime.
    /// </summary>
    public ImuCaptureRecorder(Stream output, bool ownsStream = true)
    {
        _output = output ?? throw new ArgumentNullException(nameof(output));
        _ownsStream = ownsStream;
        _startedUtc = DateTime.UtcNow;
    }

    public uint FrameCount => _frameCount;

    /// <summary>
    /// Total samples lost to BT drops, derived from gaps in the
    /// monotonic <see cref="ImuCapFrame.Seq"/> counter.  Non-zero
    /// rejects the session for Tedaldi calibration use.
    /// </summary>
    public uint DropCount => _dropCount;

    public TimeSpan Elapsed => DateTime.UtcNow - _startedUtc;

    /// <summary>
    /// Running mean of the raw OUT_TEMP value.  Convert to °C with
    /// <c>25 + raw / 256</c> (LSM6DSL datasheet sensitivity).  Returns
    /// 0 when no frames have been received.
    /// </summary>
    public double AverageTemperatureRaw =>
        _frameCount == 0 ? 0.0 : (double)_temperatureSum / _frameCount;

    /// <summary>
    /// Append a frame to the output stream and update stats.  Counts
    /// any wrap-aware gap in <see cref="ImuCapFrame.Seq"/> against
    /// <see cref="DropCount"/>.  Returns the number of dropped samples
    /// implied by this frame's seq (0 in steady state).
    /// </summary>
    public uint OnImuCapFrame(in ImuCapFrame frame)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        // Serialise the payload back to bytes.  The on-wire bytes have
        // already been validated by ImuCapFrameParser; re-encoding from
        // the typed record keeps the recorder independent of the byte
        // span the parser saw (it does not retain a reference), and
        // the cost is dominated by the file write anyway.
        Span<byte> b = _frameBuffer;
        BinaryPrimitives.WriteUInt16LittleEndian(b[0..2], WireConstants.Magic);
        b[2] = WireConstants.ImuCapFrameType;
        BinaryPrimitives.WriteUInt16LittleEndian(b[3..5],
            (ushort)WireConstants.ImuCapFrameSize);
        Span<byte> p = b.Slice(WireConstants.BundleEnvelopeSize,
                               WireConstants.ImuCapPayloadSize);
        BinaryPrimitives.WriteUInt32LittleEndian(p[0..4], frame.TimestampUs);
        BinaryPrimitives.WriteInt16LittleEndian(p[4..6], frame.Ax);
        BinaryPrimitives.WriteInt16LittleEndian(p[6..8], frame.Ay);
        BinaryPrimitives.WriteInt16LittleEndian(p[8..10], frame.Az);
        BinaryPrimitives.WriteInt16LittleEndian(p[10..12], frame.Gx);
        BinaryPrimitives.WriteInt16LittleEndian(p[12..14], frame.Gy);
        BinaryPrimitives.WriteInt16LittleEndian(p[14..16], frame.Gz);
        BinaryPrimitives.WriteInt16LittleEndian(p[16..18], frame.TemperatureRaw);
        p[18] = frame.FsrXlIdx;
        p[19] = frame.FsrGyIdx;
        BinaryPrimitives.WriteUInt16LittleEndian(p[20..22], frame.Seq);

        _output.Write(_frameBuffer);

        uint gap = 0;
        if (_hasPriorSeq)
        {
            // Wrap-aware: (cur - prev - 1) mod 2^16 = number of missed
            // samples between prev and cur.  Steady state = 0.
            gap = (uint)((ushort)(frame.Seq - _lastSeq - 1));
            _dropCount += gap;
        }
        _lastSeq = frame.Seq;
        _hasPriorSeq = true;

        _temperatureSum += frame.TemperatureRaw;
        _frameCount++;
        return gap;
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        try
        {
            _output.Flush();
        }
        catch
        {
            // Best-effort flush — surface on Dispose only if the stream
            // itself throws below.
        }

        if (_ownsStream)
        {
            _output.Dispose();
        }
    }
}
