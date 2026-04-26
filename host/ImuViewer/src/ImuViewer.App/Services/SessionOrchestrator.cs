using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Transport;
using ImuViewer.Core.Wire;

namespace ImuViewer.App.Services;

/// <summary>
/// Owns the Bluetooth lifecycle for a single Hub: connect → BtsensorSession →
/// command client → frame routing → aggregator. The orchestrator does not
/// integrate the Madgwick filter itself; the UI tick pulls aggregated samples
/// and updates the filter on its own thread.
/// </summary>
public sealed class SessionOrchestrator : IAsyncDisposable
{
    private readonly IBluetoothTransport _transport;
    private readonly SensorAggregator _aggregator;
    private Stream? _stream;
    private BtsensorSession? _session;
    private BtsensorCommandClient? _client;
    private bool _disposed;

    public SessionOrchestrator(IBluetoothTransport transport, SensorAggregator aggregator)
    {
        _transport = transport;
        _aggregator = aggregator;
    }

    public bool IsConnected => _client is not null;

    public event Action<ImuFrame>? FrameReceived;

    public async Task ConnectAsync(string bdAddr, int channel, CancellationToken ct)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (IsConnected)
        {
            throw new InvalidOperationException("already connected");
        }
        _stream = await _transport.ConnectAsync(bdAddr, channel, ct).ConfigureAwait(false);
        _session = new BtsensorSession(_stream);
        _client = new BtsensorCommandClient(_session);
        _session.FrameReceived += OnFrame;
        _session.Start();
    }

    public async Task<BtsensorReply> SendAsync(string command, CancellationToken ct)
    {
        BtsensorCommandClient client = _client
            ?? throw new InvalidOperationException("not connected");
        return await client.SendAsync(command, ct).ConfigureAwait(false);
    }

    public Task<BtsensorReply> ImuOffAsync(CancellationToken ct) => SendAsync("IMU OFF", ct);
    public Task<BtsensorReply> ImuOnAsync(CancellationToken ct) => SendAsync("IMU ON", ct);
    public Task<BtsensorReply> SetOdrAsync(int hz, CancellationToken ct) => SendAsync($"SET ODR {hz}", ct);
    public Task<BtsensorReply> SetBatchAsync(int n, CancellationToken ct) => SendAsync($"SET BATCH {n}", ct);
    public Task<BtsensorReply> SetAccelFsrAsync(int g, CancellationToken ct) => SendAsync($"SET ACCEL_FSR {g}", ct);
    public Task<BtsensorReply> SetGyroFsrAsync(int dps, CancellationToken ct) => SendAsync($"SET GYRO_FSR {dps}", ct);

    public async Task DisconnectAsync()
    {
        if (!IsConnected)
        {
            return;
        }
        try
        {
            using CancellationTokenSource cts = new(TimeSpan.FromSeconds(2));
            await SendAsync("IMU OFF", cts.Token).ConfigureAwait(false);
        }
        catch
        {
            // Best effort.
        }
        await TeardownAsync().ConfigureAwait(false);
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        await TeardownAsync().ConfigureAwait(false);
        await _transport.DisposeAsync().ConfigureAwait(false);
    }

    private async Task TeardownAsync()
    {
        if (_session is not null)
        {
            _session.FrameReceived -= OnFrame;
        }
        _client?.Dispose();
        _client = null;
        if (_session is not null)
        {
            await _session.DisposeAsync().ConfigureAwait(false);
            _session = null;
        }
        if (_stream is not null)
        {
            await _stream.DisposeAsync().ConfigureAwait(false);
            _stream = null;
        }
    }

    private void OnFrame(ImuFrame frame)
    {
        _aggregator.OnFrame(frame);
        try
        {
            FrameReceived?.Invoke(frame);
        }
        catch
        {
            // Ignore subscriber errors; do not break the reader.
        }
    }
}
