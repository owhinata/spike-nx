using CaptureViewer.Core.Capture;

namespace CaptureViewer.Core.Live;

/// <summary>
/// Reads a continuous BT byte stream produced by btsensor (telemetry +
/// MODE CAPTURE traffic interleaved) and surfaces complete capture
/// sessions as <see cref="SessionScan"/> events.
/// </summary>
/// <remarks>
/// Designed to be pointed at any byte-oriented <see cref="Stream"/>:
/// in production it is the <c>LinuxRfcommStream</c> from ImuViewer.Core
/// (project-ref'd by the App layer); in tests it can be a
/// <see cref="MemoryStream"/> seeded with synthetic bytes.
///
/// The receiver does NOT parse BUNDLE telemetry frames — those are
/// scanned for BTCS markers and otherwise discarded.  The
/// <see cref="SessionScanner"/> sanity check rejects false-positive
/// BTCS hits inside random binary noise (BUNDLE bytes included).
/// </remarks>
public sealed class LiveCaptureReceiver : IAsyncDisposable
{
    private const int InitialBufferCapacity = 4096;
    private const int ReadChunkBytes = 1024;

    private readonly Stream _stream;
    private readonly Func<ushort, bool>? _resolveSchema;
    private readonly bool _ownsStream;
    private readonly CancellationTokenSource _cts = new();
    private readonly SemaphoreSlim _writeLock = new(1, 1);

    private byte[] _buf = new byte[InitialBufferCapacity];
    private int _bufLen;
    private Task? _readerTask;
    private bool _disposed;

    /// <param name="stream">A connected byte stream (RFCOMM, file, or
    /// in-memory pipe).</param>
    /// <param name="resolveSchema">Optional callback used by
    /// <see cref="SessionScanner"/> to reject sessions whose
    /// schema_magic is unknown.  Pass <c>null</c> to accept all
    /// magics — useful in tests.</param>
    /// <param name="ownsStream">If <c>true</c>, the receiver disposes
    /// <paramref name="stream"/> when it is itself disposed.</param>
    public LiveCaptureReceiver(
        Stream stream,
        Func<ushort, bool>? resolveSchema = null,
        bool ownsStream = true)
    {
        _stream = stream ?? throw new ArgumentNullException(nameof(stream));
        _resolveSchema = resolveSchema;
        _ownsStream = ownsStream;
    }

    /// <summary>
    /// Fired (on the reader task thread) for each complete session
    /// the scanner decodes.  Raised after the session bytes have been
    /// consumed from the rolling buffer, so handlers may safely take
    /// their time.
    /// </summary>
    public event Action<SessionScan>? CaptureReceived;

    /// <summary>
    /// Fired once when the underlying stream returns 0 (EOF) or has
    /// been closed by the peer.  The reader task exits after this.
    /// </summary>
    public event Action? StreamClosed;

    /// <summary>
    /// Fired when the read loop encounters an unexpected exception
    /// (other than cancellation / EOF).  The reader task exits.
    /// </summary>
    public event Action<Exception>? ReceiveError;

    /// <summary>
    /// Fired (on the reader task thread) for every successful read
    /// from the underlying stream — payload of <c>chunkBytes</c>
    /// bytes, <see cref="BytesRead"/> already updated.  Useful for
    /// status heartbeats so the operator can see traffic even when no
    /// session has fully landed yet.
    /// </summary>
    public event Action<int>? BytesReceived;

    /// <summary>
    /// Total bytes the receiver has read from <see cref="_stream"/>
    /// so far.  Useful for status displays / regression assertions.
    /// </summary>
    public long BytesRead { get; private set; }

    /// <summary>
    /// Total complete sessions surfaced through <see cref="CaptureReceived"/>.
    /// </summary>
    public int SessionsReceived { get; private set; }

    /// <summary>
    /// Start the background reader task.  Throws if the receiver was
    /// already started or is disposed.
    /// </summary>
    public void Start()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (_readerTask is not null)
            throw new InvalidOperationException("Receiver already started");
        _readerTask = Task.Run(() => RunAsync(_cts.Token));
    }

    /// <summary>
    /// Wait for the reader task to exit.  Returns immediately if it
    /// has already exited.
    /// </summary>
    public async Task WaitForExitAsync(CancellationToken ct = default)
    {
        if (_readerTask is null)
            return;
        var task = _readerTask;
        if (ct.CanBeCanceled)
        {
            using var reg = ct.Register(() => _cts.Cancel());
            try { await task.ConfigureAwait(false); }
            catch (OperationCanceledException) { }
        }
        else
        {
            try { await task.ConfigureAwait(false); }
            catch (OperationCanceledException) { }
        }
    }

    /// <summary>
    /// Send a raw byte sequence to the peer (for example
    /// <c>"MODE CAPTURE\n"</c> to trigger a drain on the Hub).  Writes
    /// are mutex-serialized so callers can pump from any thread.
    /// </summary>
    public async Task SendAsync(ReadOnlyMemory<byte> data, CancellationToken ct = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        await _writeLock.WaitAsync(ct).ConfigureAwait(false);
        try
        {
            await _stream.WriteAsync(data, ct).ConfigureAwait(false);
            await _stream.FlushAsync(ct).ConfigureAwait(false);
        }
        finally
        {
            _writeLock.Release();
        }
    }

    /// <summary>
    /// Convenience wrapper for ASCII command lines.  Appends a single
    /// '\n' byte if the input does not already end with one.
    /// </summary>
    public Task SendCommandAsync(string line, CancellationToken ct = default)
    {
        if (!line.EndsWith('\n'))
            line += "\n";
        return SendAsync(System.Text.Encoding.ASCII.GetBytes(line), ct);
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed) return;
        _disposed = true;

        if (!_cts.IsCancellationRequested)
        {
            try { _cts.Cancel(); }
            catch (ObjectDisposedException) { /* benign */ }
        }

        // Dispose the stream BEFORE awaiting the reader: a blocking
        // recv() on Linux RFCOMM can only be unblocked by closing the
        // fd, not by the cancellation token.  Mirrors the same comment
        // in ImuViewer.Core.Btsensor.BtsensorSession.
        if (_ownsStream)
        {
            try { await _stream.DisposeAsync().ConfigureAwait(false); }
            catch { /* the catch in RunAsync will surface this */ }
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

    private async Task RunAsync(CancellationToken ct)
    {
        var chunk = new byte[ReadChunkBytes];
        try
        {
            while (!ct.IsCancellationRequested)
            {
                int n;
                try
                {
                    n = await _stream.ReadAsync(chunk, ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex) when (ex is IOException || ex is ObjectDisposedException)
                {
                    if (!ct.IsCancellationRequested)
                        ReceiveError?.Invoke(ex);
                    break;
                }

                if (n <= 0)
                {
                    StreamClosed?.Invoke();
                    return;
                }

                Append(chunk, n);
                BytesRead += n;
                BytesReceived?.Invoke(n);
                DrainSessions();
            }
        }
        catch (Exception ex)
        {
            ReceiveError?.Invoke(ex);
        }
    }

    private void Append(byte[] src, int count)
    {
        EnsureCapacity(_bufLen + count);
        Buffer.BlockCopy(src, 0, _buf, _bufLen, count);
        _bufLen += count;
    }

    private void EnsureCapacity(int needed)
    {
        if (_buf.Length >= needed) return;
        var newCap = _buf.Length;
        while (newCap < needed) newCap *= 2;
        var newBuf = new byte[newCap];
        Buffer.BlockCopy(_buf, 0, newBuf, 0, _bufLen);
        _buf = newBuf;
    }

    private void DrainSessions()
    {
        // Drain as many complete sessions as the rolling buffer holds.
        // Sessions that arrive back-to-back (rare but possible if the
        // user fires two `sensor capture` runs in a row) are surfaced
        // in arrival order.
        while (true)
        {
            var found = SessionScanner.TryScan(
                _buf.AsSpan(0, _bufLen),
                _resolveSchema,
                out var scan);
            if (!found) break;

            CaptureReceived?.Invoke(scan!);
            SessionsReceived++;

            // Compact: drop everything up to scan.EndIndex.  Bytes
            // before StartIndex were noise (telemetry / partial
            // frames); the scanner already skipped them.
            var consumed = scan!.EndIndex;
            var remaining = _bufLen - consumed;
            if (remaining > 0)
            {
                Buffer.BlockCopy(_buf, consumed, _buf, 0, remaining);
            }
            _bufLen = remaining;
        }
    }
}
