using System.Buffers;
using System.IO.Pipelines;

namespace ImuViewer.Core.Tests.Btsensor;

/// <summary>
/// Two-direction Stream backed by a Pipe (incoming, fed by tests) and an
/// in-memory list (outgoing, captured for assertions).
/// </summary>
internal sealed class FakeDuplexStream : Stream
{
    private readonly Pipe _incoming = new();
    private readonly object _outgoingLock = new();
    private readonly MemoryStream _outgoing = new();
    private bool _disposed;

    public override bool CanRead => !_disposed;
    public override bool CanWrite => !_disposed;
    public override bool CanSeek => false;
    public override long Length => throw new NotSupportedException();
    public override long Position { get => throw new NotSupportedException(); set => throw new NotSupportedException(); }

    public async Task InjectAsync(byte[] bytes)
    {
        await _incoming.Writer.WriteAsync(bytes).ConfigureAwait(false);
    }

    public byte[] GetWritten()
    {
        lock (_outgoingLock)
        {
            return _outgoing.ToArray();
        }
    }

    public override int Read(byte[] buffer, int offset, int count) =>
        ReadAsync(buffer.AsMemory(offset, count), CancellationToken.None).AsTask().GetAwaiter().GetResult();

    public override async ValueTask<int> ReadAsync(Memory<byte> buffer, CancellationToken ct = default)
    {
        ReadResult result = await _incoming.Reader.ReadAsync(ct).ConfigureAwait(false);
        ReadOnlySequence<byte> seq = result.Buffer;
        if (seq.Length == 0 && result.IsCompleted)
        {
            return 0;
        }
        int n = (int)Math.Min(seq.Length, buffer.Length);
        seq.Slice(0, n).CopyTo(buffer.Span);
        _incoming.Reader.AdvanceTo(seq.GetPosition(n));
        return n;
    }

    public override void Write(byte[] buffer, int offset, int count)
    {
        lock (_outgoingLock)
        {
            _outgoing.Write(buffer, offset, count);
        }
    }

    public override ValueTask WriteAsync(ReadOnlyMemory<byte> buffer, CancellationToken ct = default)
    {
        lock (_outgoingLock)
        {
            _outgoing.Write(buffer.Span);
        }
        return ValueTask.CompletedTask;
    }

    public override void Flush() { }
    public override Task FlushAsync(CancellationToken ct) => Task.CompletedTask;
    public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
    public override void SetLength(long value) => throw new NotSupportedException();

    protected override void Dispose(bool disposing)
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        _incoming.Writer.Complete();
        _outgoing.Dispose();
        base.Dispose(disposing);
    }
}
