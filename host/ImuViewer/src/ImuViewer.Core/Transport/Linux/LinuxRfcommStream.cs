namespace ImuViewer.Core.Transport.Linux;

/// <summary>
/// Adapts <see cref="LinuxRfcommSocket"/> to a <see cref="Stream"/>. Reads and
/// writes are blocking syscalls; async overloads dispatch them to the thread
/// pool, which is sufficient for the BTPROTO_RFCOMM byte rates we expect (max
/// ~1300 byte frames at ~64 Hz).
/// </summary>
internal sealed class LinuxRfcommStream : Stream
{
    private readonly LinuxRfcommSocket _socket;
    private bool _disposed;

    public LinuxRfcommStream(LinuxRfcommSocket socket)
    {
        _socket = socket;
    }

    public override bool CanRead => !_disposed;
    public override bool CanWrite => !_disposed;
    public override bool CanSeek => false;
    public override long Length => throw new NotSupportedException();
    public override long Position
    {
        get => throw new NotSupportedException();
        set => throw new NotSupportedException();
    }

    public override int Read(byte[] buffer, int offset, int count)
    {
        ThrowIfDisposed();
        return _socket.Read(buffer.AsSpan(offset, count));
    }

    public override int Read(Span<byte> buffer)
    {
        ThrowIfDisposed();
        return _socket.Read(buffer);
    }

    public override async ValueTask<int> ReadAsync(Memory<byte> buffer, CancellationToken ct = default)
    {
        ThrowIfDisposed();
        ct.ThrowIfCancellationRequested();
        // recv blocks; run on thread pool. ct cannot interrupt the syscall — caller
        // is expected to Dispose() the stream to abort.
        return await Task.Run(() => _socket.Read(buffer.Span), ct).ConfigureAwait(false);
    }

    public override void Write(byte[] buffer, int offset, int count)
    {
        ThrowIfDisposed();
        _socket.Write(buffer.AsSpan(offset, count));
    }

    public override void Write(ReadOnlySpan<byte> buffer)
    {
        ThrowIfDisposed();
        _socket.Write(buffer);
    }

    public override async ValueTask WriteAsync(ReadOnlyMemory<byte> buffer, CancellationToken ct = default)
    {
        ThrowIfDisposed();
        ct.ThrowIfCancellationRequested();
        byte[] copy = buffer.ToArray();
        await Task.Run(() => _socket.Write(copy), ct).ConfigureAwait(false);
    }

    public override void Flush() { /* stream-of-bytes; no buffering to flush. */ }

    public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
    public override void SetLength(long value) => throw new NotSupportedException();

    protected override void Dispose(bool disposing)
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        if (disposing)
        {
            _socket.Dispose();
        }
        base.Dispose(disposing);
    }

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
    }
}
