using System.Text;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.Btsensor;

/// <summary>
/// Reads the multiplexed RFCOMM byte stream emitted by btsensor and dispatches
/// binary IMU frames and ASCII reply lines to subscribers.
/// </summary>
/// <remarks>
/// btsensor emits two interleaved channels on RFCOMM channel 1:
///   - Binary IMU frames starting with magic 0x6B 0xB6 (LE) (see WireConstants)
///   - ASCII command replies "OK\n" / "ERR &lt;reason&gt;\n"
///
/// Disambiguation: 0xB6 is not printable ASCII, so a 0x6B 0xB6 prefix unambiguously
/// signals a frame boundary. A bare 0x6B is just ASCII 'k' and is treated as the
/// start of a reply line. Bytes that do not match either prefix shape are dropped
/// one at a time to resync (mirrors the Python reference parser).
/// </remarks>
public sealed class BtsensorSession : IAsyncDisposable
{
    private const int InitialBufferCapacity = 4096;

    private readonly Stream _stream;
    private readonly bool _ownsStream;
    private readonly CancellationTokenSource _cts = new();
    private readonly SemaphoreSlim _writeLock = new(1, 1);
    private byte[] _buffer = new byte[InitialBufferCapacity];
    private int _bufferLength;
    private Task? _readerTask;

    public BtsensorSession(Stream stream, bool ownsStream = true)
    {
        _stream = stream ?? throw new ArgumentNullException(nameof(stream));
        _ownsStream = ownsStream;
    }

    public event Action<BundleFrame>? BundleReceived;
    public event Action<string>? ReplyLineReceived;

    public void Start()
    {
        if (_readerTask is not null)
        {
            throw new InvalidOperationException("session already started");
        }
        _readerTask = Task.Run(() => RunReaderAsync(_cts.Token));
    }

    public async Task WriteLineAsync(string line, CancellationToken ct)
    {
        ArgumentNullException.ThrowIfNull(line);
        byte[] payload = Encoding.ASCII.GetBytes(line + "\n");
        await _writeLock.WaitAsync(ct).ConfigureAwait(false);
        try
        {
            await _stream.WriteAsync(payload, ct).ConfigureAwait(false);
            await _stream.FlushAsync(ct).ConfigureAwait(false);
        }
        finally
        {
            _writeLock.Release();
        }
    }

    public async ValueTask DisposeAsync()
    {
        if (!_cts.IsCancellationRequested)
        {
            await _cts.CancelAsync().ConfigureAwait(false);
        }
        // Dispose the stream BEFORE awaiting the reader task. The reader is
        // most likely parked inside a blocking recv() syscall (see
        // LinuxRfcommStream.ReadAsync — Task.Run wraps a libc recv that the
        // CancellationToken cannot interrupt), and only fd close unblocks
        // it. Awaiting the reader first would deadlock indefinitely. The
        // reader's catch handlers below cover the resulting IOException /
        // ObjectDisposedException.
        if (_ownsStream)
        {
            await _stream.DisposeAsync().ConfigureAwait(false);
        }
        if (_readerTask is not null)
        {
            try { await _readerTask.ConfigureAwait(false); }
            catch (OperationCanceledException) { }
            catch (IOException) { }
            catch (ObjectDisposedException) { }
        }
        _writeLock.Dispose();
        _cts.Dispose();
    }

    private async Task RunReaderAsync(CancellationToken ct)
    {
        byte[] readBuffer = new byte[1024];
        while (!ct.IsCancellationRequested)
        {
            int read;
            try
            {
                read = await _stream.ReadAsync(readBuffer.AsMemory(), ct).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (IOException)
            {
                break;
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            if (read <= 0)
            {
                break;
            }
            Append(readBuffer.AsSpan(0, read));
            DrainBuffer();
        }
    }

    private void Append(ReadOnlySpan<byte> bytes)
    {
        EnsureCapacity(_bufferLength + bytes.Length);
        bytes.CopyTo(_buffer.AsSpan(_bufferLength));
        _bufferLength += bytes.Length;
    }

    private void DrainBuffer()
    {
        int idx = 0;
        while (idx < _bufferLength)
        {
            byte b = _buffer[idx];
            if (b == 0x6B && idx + 1 < _bufferLength && _buffer[idx + 1] == 0xB6)
            {
                int frameSize = TryConsumeFrame(idx);
                if (frameSize == 0)
                {
                    // Need more bytes to complete the header or body.
                    break;
                }
                if (frameSize < 0)
                {
                    // Header sanity failed; advance one byte and try resync.
                    idx += 1;
                    continue;
                }
                idx += frameSize;
                continue;
            }
            if (IsLineByte(b))
            {
                int nl = IndexOfNewline(idx);
                if (nl < 0)
                {
                    break;
                }
                EmitLine(idx, nl);
                idx = nl + 1;
                continue;
            }
            // Not a frame magic prefix and not ASCII; drop one byte.
            idx += 1;
        }
        if (idx > 0)
        {
            ShiftLeft(idx);
        }
    }

    /// <returns>
    /// Number of bytes consumed (frame length) on success, 0 if more bytes are
    /// needed, -1 if the header is invalid and the caller should advance one byte.
    /// </returns>
    private int TryConsumeFrame(int start)
    {
        // Need at least envelope (5 B) to read type + frame_len.
        if (_bufferLength - start < WireConstants.BundleEnvelopeSize)
        {
            return 0;
        }

        ReadOnlySpan<byte> envelope = _buffer.AsSpan(start, WireConstants.BundleEnvelopeSize);
        byte type = envelope[2];
        ushort frameLen = (ushort)(envelope[3] | (envelope[4] << 8));

        if (type != WireConstants.BundleFrameType)
        {
            return -1;
        }

        // Lower bound on a sane frame_len: envelope + bundle header.
        if (frameLen < WireConstants.BundleEnvelopeSize + WireConstants.BundleHeaderSize ||
            frameLen > WireConstants.MaxBundleFrameSize)
        {
            return -1;
        }

        if (_bufferLength - start < frameLen)
        {
            return 0;
        }

        BundleFrame? frame = BundleFrameParser.TryDecode(_buffer.AsSpan(start, frameLen));
        if (frame is null)
        {
            return -1;
        }

        try
        {
            BundleReceived?.Invoke(frame);
        }
        catch
        {
            // Subscriber exceptions must not break the reader loop.
        }

        return frameLen;
    }

    private void EmitLine(int start, int newlineIndex)
    {
        int length = newlineIndex - start;
        if (length > 0 && _buffer[newlineIndex - 1] == (byte)'\r')
        {
            length -= 1;
        }
        string line = length > 0
            ? Encoding.ASCII.GetString(_buffer, start, length)
            : string.Empty;
        try
        {
            ReplyLineReceived?.Invoke(line);
        }
        catch
        {
            // Subscriber exceptions must not break the reader loop.
        }
    }

    private int IndexOfNewline(int start)
    {
        for (int i = start; i < _bufferLength; i++)
        {
            if (_buffer[i] == (byte)'\n')
            {
                return i;
            }
        }
        return -1;
    }

    private static bool IsLineByte(byte b) =>
        b == (byte)'\r' || b == (byte)'\n' || (b >= 0x20 && b <= 0x7E);

    private void ShiftLeft(int count)
    {
        if (count >= _bufferLength)
        {
            _bufferLength = 0;
            return;
        }
        Buffer.BlockCopy(_buffer, count, _buffer, 0, _bufferLength - count);
        _bufferLength -= count;
    }

    private void EnsureCapacity(int required)
    {
        if (_buffer.Length >= required)
        {
            return;
        }
        int newSize = _buffer.Length;
        while (newSize < required)
        {
            newSize *= 2;
        }
        Array.Resize(ref _buffer, newSize);
    }
}
