using CaptureViewer.Core;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;
using CaptureViewer.Core.Live;

using ImuViewer.Core.Transport;
using ImuViewer.Core.Transport.Linux;

namespace CaptureViewer.App.Services;

/// <summary>
/// Lifecycle wrapper around <see cref="IBluetoothTransport"/> +
/// <see cref="LiveCaptureReceiver"/>.  One Hub at a time.  Forwards
/// incoming sessions to the configured handler and disposes the
/// pipeline cleanly on disconnect.
/// </summary>
public sealed class BtConnection : IAsyncDisposable
{
    private readonly Action<SessionScan> _onSession;
    private readonly Action<string>? _onLog;

    private LiveCaptureReceiver? _receiver;
    private IAsyncDisposable? _transport;
    private CancellationTokenSource? _connectCts;

    public BtConnection(Action<SessionScan> onSession, Action<string>? onLog = null)
    {
        _onSession = onSession;
        _onLog = onLog;
    }

    public bool IsConnected => _receiver is not null;

    /// <summary>
    /// Open an RFCOMM connection to <paramref name="bdAddr"/> on
    /// channel 1 and start the receiver.  Linux only; macOS / Windows
    /// throw <see cref="PlatformNotSupportedException"/>.
    /// </summary>
    public async Task ConnectAsync(string bdAddr, int channel = 1)
    {
        if (IsConnected)
            throw new InvalidOperationException("Already connected");

        _connectCts = new CancellationTokenSource();
        IBluetoothTransport transport = new LinuxRfcommTransport();
        _transport = transport;

        Log($"Connecting to {bdAddr} ch={channel}...");
        var stream = await transport.ConnectAsync(bdAddr, channel, _connectCts.Token)
                                    .ConfigureAwait(false);
        Log($"Connected.");

        _receiver = new LiveCaptureReceiver(
            stream,
            magic => KnownSchemas.ByMagic.ContainsKey(magic));
        _receiver.CaptureReceived += OnCaptureReceived;
        _receiver.StreamClosed += () => Log("RFCOMM stream closed by peer");
        _receiver.ReceiveError += ex => Log($"Receiver error: {ex.Message}");
        _receiver.Start();
    }

    /// <summary>
    /// Send <c>"MODE CAPTURE\n"</c> to ask btsensor to drain any
    /// pending writer.  Convenience pass-through to
    /// <see cref="LiveCaptureReceiver.SendCommandAsync"/>.
    /// </summary>
    public Task TriggerCaptureModeAsync(CancellationToken ct = default)
    {
        if (_receiver is null)
            throw new InvalidOperationException("Not connected");
        Log("Sending `MODE CAPTURE`");
        return _receiver.SendCommandAsync("MODE CAPTURE", ct);
    }

    public async ValueTask DisposeAsync()
    {
        if (_connectCts is not null)
        {
            try { _connectCts.Cancel(); } catch { }
        }

        if (_receiver is not null)
        {
            try { await _receiver.DisposeAsync().ConfigureAwait(false); }
            catch { /* receiver already logs the error */ }
            _receiver = null;
        }

        if (_transport is not null)
        {
            try { await _transport.DisposeAsync().ConfigureAwait(false); }
            catch { }
            _transport = null;
        }

        _connectCts?.Dispose();
        _connectCts = null;
    }

    private void OnCaptureReceived(SessionScan scan)
    {
        Log($"Capture received: {scan.Capture.SchemaName} " +
            $"({scan.Capture.RecordCount} records, " +
            $"{scan.EndIndex - scan.StartIndex} bytes)");
        _onSession(scan);
    }

    private void Log(string line) => _onLog?.Invoke(line);
}
