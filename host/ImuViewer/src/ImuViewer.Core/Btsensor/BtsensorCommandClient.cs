namespace ImuViewer.Core.Btsensor;

/// <summary>
/// Sends btsensor ASCII commands ("IMU ON", "SET BATCH 13", ...) and resolves
/// the next reply line as a <see cref="BtsensorReply"/>. Reply lines arrive on
/// the session's reader thread; this class FIFO-matches them to the pending
/// SendAsync calls.
/// </summary>
/// <remarks>
/// btsensor processes commands serially, so the reply order matches the send
/// order. Tests pin this assumption via H-8 (test_bt_rfcomm_command_suite).
/// </remarks>
public sealed class BtsensorCommandClient : IDisposable
{
    private readonly BtsensorSession _session;
    private readonly Lock _gate = new();
    private readonly Queue<TaskCompletionSource<BtsensorReply>> _pending = new();
    private readonly SemaphoreSlim _sendLock = new(1, 1);
    private bool _disposed;

    public BtsensorCommandClient(BtsensorSession session)
    {
        _session = session ?? throw new ArgumentNullException(nameof(session));
        _session.ReplyLineReceived += OnReplyLine;
    }

    public async Task<BtsensorReply> SendAsync(string command, CancellationToken ct)
    {
        ArgumentException.ThrowIfNullOrEmpty(command);
        ObjectDisposedException.ThrowIf(_disposed, this);

        await _sendLock.WaitAsync(ct).ConfigureAwait(false);
        TaskCompletionSource<BtsensorReply> tcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        try
        {
            lock (_gate)
            {
                _pending.Enqueue(tcs);
            }
            await _session.WriteLineAsync(command, ct).ConfigureAwait(false);
        }
        catch
        {
            lock (_gate)
            {
                // Best-effort removal so a future reply does not satisfy a
                // failed send.
                if (_pending.Count > 0 && _pending.Peek() == tcs)
                {
                    _pending.Dequeue();
                }
            }
            throw;
        }
        finally
        {
            _sendLock.Release();
        }

        using CancellationTokenRegistration reg = ct.Register(static state =>
        {
            ((TaskCompletionSource<BtsensorReply>)state!).TrySetCanceled();
        }, tcs);
        return await tcs.Task.ConfigureAwait(false);
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        _session.ReplyLineReceived -= OnReplyLine;
        _sendLock.Dispose();
        lock (_gate)
        {
            while (_pending.Count > 0)
            {
                _pending.Dequeue().TrySetCanceled();
            }
        }
    }

    private void OnReplyLine(string line)
    {
        TaskCompletionSource<BtsensorReply>? tcs = null;
        lock (_gate)
        {
            if (_pending.Count > 0)
            {
                tcs = _pending.Dequeue();
            }
        }
        // If no caller is waiting, drop the reply (out-of-band noise).
        tcs?.TrySetResult(BtsensorReply.Parse(line));
    }
}
