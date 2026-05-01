using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Transport;
using ImuViewer.Core.Wire;

namespace ImuViewer.App.Services;

/// <summary>
/// Owns the Bluetooth lifecycle for a single Hub: connect → BtsensorSession →
/// command client → frame routing → aggregators. The orchestrator does not
/// integrate the Madgwick filter itself; the UI tick pulls aggregated samples
/// and updates the filter on its own thread.
/// </summary>
public sealed class SessionOrchestrator : IAsyncDisposable
{
    private readonly IBluetoothTransport _transport;
    private readonly SensorAggregator _aggregator;
    private readonly LegoSampleAggregator _legoAggregator;
    private Stream? _stream;
    private BtsensorSession? _session;
    private BtsensorCommandClient? _client;
    private bool _disposed;

    public SessionOrchestrator(IBluetoothTransport transport,
                               SensorAggregator aggregator,
                               LegoSampleAggregator legoAggregator)
    {
        _transport = transport;
        _aggregator = aggregator;
        _legoAggregator = legoAggregator;
    }

    public bool IsConnected => _client is not null;

    public event Action<BundleFrame>? BundleReceived;

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
        _session.BundleReceived += OnBundle;
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
    public Task<BtsensorReply> SensorOnAsync(CancellationToken ct) => SendAsync("SENSOR ON", ct);
    public Task<BtsensorReply> SensorOffAsync(CancellationToken ct) => SendAsync("SENSOR OFF", ct);
    public Task<BtsensorReply> SetOdrAsync(int hz, CancellationToken ct) => SendAsync($"SET ODR {hz}", ct);
    public Task<BtsensorReply> SetAccelFsrAsync(int g, CancellationToken ct) => SendAsync($"SET ACCEL_FSR {g}", ct);
    public Task<BtsensorReply> SetGyroFsrAsync(int dps, CancellationToken ct) => SendAsync($"SET GYRO_FSR {dps}", ct);

    public Task<BtsensorReply> SetSensorModeAsync(
        LegoClassId classId, int mode, CancellationToken ct) =>
        SendAsync($"SENSOR MODE {ClassToken(classId)} {mode}", ct);

    public Task<BtsensorReply> SensorSendAsync(
        LegoClassId classId, int mode, ReadOnlySpan<byte> payload, CancellationToken ct)
    {
        // Build "SENSOR SEND <class> <mode> <hex...>".  Each byte ends up
        // as a 2-char lowercase hex token; the parser accepts space-
        // separated tokens of even length.
        char[] hex = new char[payload.Length * 3];
        for (int i = 0; i < payload.Length; i++)
        {
            byte b = payload[i];
            hex[i * 3]     = ToHex((b >> 4) & 0xF);
            hex[i * 3 + 1] = ToHex(b & 0xF);
            hex[i * 3 + 2] = ' ';
        }
        string hexStr = payload.Length == 0
            ? string.Empty
            : new string(hex, 0, hex.Length - 1);
        return SendAsync($"SENSOR SEND {ClassToken(classId)} {mode} {hexStr}", ct);
    }

    public Task<BtsensorReply> SensorPwmAsync(
        LegoClassId classId, IReadOnlyList<int> channels, CancellationToken ct)
    {
        if (channels is null || channels.Count == 0)
        {
            throw new ArgumentException("channels must not be empty", nameof(channels));
        }

        string args = string.Join(' ', channels);
        return SendAsync($"SENSOR PWM {ClassToken(classId)} {args}", ct);
    }

    private static char ToHex(int nibble) =>
        (char)(nibble < 10 ? '0' + nibble : 'a' + nibble - 10);

    private static string ClassToken(LegoClassId classId) => classId switch
    {
        LegoClassId.Color      => "color",
        LegoClassId.Ultrasonic => "ultrasonic",
        LegoClassId.Force      => "force",
        LegoClassId.MotorM     => "motor_m",
        LegoClassId.MotorR     => "motor_r",
        LegoClassId.MotorL     => "motor_l",
        _ => ((int)classId).ToString(),
    };

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
            _session.BundleReceived -= OnBundle;
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

    private void OnBundle(BundleFrame frame)
    {
        _aggregator.OnBundle(frame);
        _legoAggregator.OnTlvBatch(frame.Header.TickTsUs, frame.Tlvs);
        try
        {
            BundleReceived?.Invoke(frame);
        }
        catch
        {
            // Ignore subscriber errors; do not break the reader.
        }
    }
}
