using System.Collections.ObjectModel;
using System.Numerics;
using System.Runtime.InteropServices;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuViewer.App.Services;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Filters;
using ImuViewer.Core.Transport;

namespace ImuViewer.App.ViewModels;

public sealed partial class MainViewModel : ObservableObject, IAsyncDisposable
{
    private readonly IBluetoothPortEnumerator _portEnumerator;
    private readonly SessionOrchestrator _orchestrator;
    private readonly SensorAggregator _aggregator;
    private readonly MadgwickFilter _filter;
    private readonly FpsCounter _fps = new();

    public MainViewModel(
        IBluetoothPortEnumerator portEnumerator,
        SessionOrchestrator orchestrator,
        SensorAggregator aggregator,
        MadgwickFilter filter)
    {
        _portEnumerator = portEnumerator;
        _orchestrator = orchestrator;
        _aggregator = aggregator;
        _filter = filter;
        _orchestrator.FrameReceived += _ => _fps.Mark();
    }

    [ObservableProperty]
    private ObservableCollection<BluetoothPort> _ports = new();

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanConnect))]
    private BluetoothPort? _selectedPort;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanConnect))]
    [NotifyPropertyChangedFor(nameof(CanDisconnect))]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    private bool _isConnected;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    private bool _isImuOn;

    [ObservableProperty]
    private string _statusText = "ready";

    [ObservableProperty]
    private Quaternion _orientation = Quaternion.Identity;

    [ObservableProperty]
    private Vector3 _accelG;

    [ObservableProperty]
    private Vector3 _gyroDps;

    [ObservableProperty]
    private double _measuredFps;

    [ObservableProperty]
    private int _odrHz = 833;

    [ObservableProperty]
    private int _batch = 13;

    [ObservableProperty]
    private int _accelFsrG = 8;

    [ObservableProperty]
    private int _gyroFsrDps = 2000;

    [ObservableProperty]
    private float _madgwickBeta = 0.1f;

    public bool CanConnect => !IsConnected && SelectedPort is not null;
    public bool CanDisconnect => IsConnected;
    public bool CanToggleImu => IsConnected;

    partial void OnMadgwickBetaChanged(float value)
    {
        _filter.Beta = value;
    }

    /// <summary>
    /// Pulls the latest aggregated sample, runs the Madgwick filter at fixed
    /// dt = 1/60 s (Issue #60 requirement), and pushes the result to the UI.
    /// Called by MainWindow's 60 Hz DispatcherTimer.
    /// </summary>
    public void Tick()
    {
        if (_aggregator.TryConsumeNext(out AggregatedSample sample))
        {
            _filter.Update(sample.AccelG, sample.GyroRadS, 1f / 60f);
            AccelG = sample.AccelG;
            GyroDps = sample.GyroDps;
        }
        Orientation = _filter.Orientation;
        MeasuredFps = _fps.Compute();
    }

    [RelayCommand]
    public async Task RefreshPortsAsync(CancellationToken ct)
    {
        StatusText = "scanning paired devices...";
        try
        {
            IReadOnlyList<BluetoothPort> found = await _portEnumerator.GetPairedPortsAsync(ct);
            Ports.Clear();
            foreach (BluetoothPort p in found)
            {
                Ports.Add(p);
            }
            StatusText = $"{Ports.Count} paired";
        }
        catch (Exception ex)
        {
            StatusText = "scan failed: " + ex.Message;
        }
    }

    [RelayCommand]
    public async Task ConnectAsync(CancellationToken ct)
    {
        if (SelectedPort is null)
        {
            return;
        }
        if (!RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
        {
            StatusText = "only Linux RFCOMM is implemented in the PoC";
            return;
        }
        try
        {
            StatusText = $"connecting {SelectedPort.BdAddr}...";
            await _orchestrator.ConnectAsync(SelectedPort.BdAddr, channel: 1, ct);
            await EnsureReplyAsync(_orchestrator.ImuOffAsync(ct), "IMU OFF");
            await EnsureReplyAsync(_orchestrator.SetOdrAsync(OdrHz, ct), $"SET ODR {OdrHz}");
            await EnsureReplyAsync(_orchestrator.SetBatchAsync(Batch, ct), $"SET BATCH {Batch}");
            await EnsureReplyAsync(_orchestrator.SetAccelFsrAsync(AccelFsrG, ct), $"SET ACCEL_FSR {AccelFsrG}");
            await EnsureReplyAsync(_orchestrator.SetGyroFsrAsync(GyroFsrDps, ct), $"SET GYRO_FSR {GyroFsrDps}");
            IsConnected = true;
            IsImuOn = false;
            StatusText = "connected";
        }
        catch (Exception ex)
        {
            StatusText = "connect failed: " + ex.Message;
            await _orchestrator.DisconnectAsync();
            IsConnected = false;
        }
    }

    [RelayCommand]
    public async Task DisconnectAsync(CancellationToken ct)
    {
        StatusText = "disconnecting...";
        await _orchestrator.DisconnectAsync();
        IsConnected = false;
        IsImuOn = false;
        _fps.Reset();
        StatusText = "disconnected";
    }

    [RelayCommand]
    public async Task ImuOnAsync(CancellationToken ct)
    {
        try
        {
            BtsensorReply r = await _orchestrator.ImuOnAsync(ct);
            if (r.IsOk)
            {
                IsImuOn = true;
                StatusText = "IMU streaming";
            }
            else
            {
                StatusText = "IMU ON: " + r;
            }
        }
        catch (Exception ex)
        {
            StatusText = "IMU ON failed: " + ex.Message;
        }
    }

    [RelayCommand]
    public async Task ImuOffAsync(CancellationToken ct)
    {
        try
        {
            BtsensorReply r = await _orchestrator.ImuOffAsync(ct);
            IsImuOn = false;
            StatusText = r.IsOk ? "IMU stopped" : "IMU OFF: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "IMU OFF failed: " + ex.Message;
        }
    }

    [RelayCommand]
    public void ResetOrientation()
    {
        _filter.Reset();
        Orientation = _filter.Orientation;
    }

    public async ValueTask DisposeAsync()
    {
        await _orchestrator.DisposeAsync();
    }

    private static async Task EnsureReplyAsync(Task<BtsensorReply> task, string desc)
    {
        BtsensorReply r = await task;
        if (!r.IsOk)
        {
            throw new InvalidOperationException($"{desc} -> {r}");
        }
    }
}
