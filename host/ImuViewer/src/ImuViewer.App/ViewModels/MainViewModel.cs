using System.Collections.ObjectModel;
using System.Globalization;
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
    /// <summary>~2 seconds of samples at the default ODR (833 Hz).</summary>
    private const int CalibrationSampleCount = 1500;

    /// <summary>
    /// Fallback dt used for the very first sample (no previous timestamp) and
    /// when the timestamp delta is non-positive or larger than 100 ms (covers
    /// uint32 wrap and frame drops).
    /// </summary>
    private const float FallbackDt = 1f / 833f;

    private const float MaxReasonableDt = 0.1f;

    private readonly IBluetoothPortEnumerator _portEnumerator;
    private readonly SessionOrchestrator _orchestrator;
    private readonly SensorAggregator _aggregator;
    private readonly MadgwickFilter _filter;
    private readonly FpsCounter _fps = new();

    private Vector3 _gyroBiasRadS = Vector3.Zero;
    private Vector3 _calibrationSum = Vector3.Zero;
    private int _calibrationSamplesRemaining;
    private uint _previousSampleTsUs;
    private bool _hasPreviousSampleTs;

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
    [NotifyPropertyChangedFor(nameof(OrientationText))]
    private Quaternion _orientation = Quaternion.Identity;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(AccelGText))]
    private Vector3 _accelG;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(GyroDpsText))]
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

    /// <summary>
    /// Default β for per-sample integration at chip ODR. The original 0.1
    /// was tuned for a 60 Hz integration rate; running ~14× faster needs
    /// roughly √(833/60) ≈ 3.7× lower β to keep a similar gain density.
    /// </summary>
    [ObservableProperty]
    private float _madgwickBeta = 0.03f;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CalibrationStatusText))]
    private bool _isCalibrating;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CalibrationStatusText))]
    private bool _isGyroBiasCalibrated;

    public bool CanConnect => !IsConnected && SelectedPort is not null;
    public bool CanDisconnect => IsConnected;
    public bool CanToggleImu => IsConnected;

    /// <summary>
    /// Workaround for Avalonia binding paths not resolving fields on
    /// System.Numerics.Quaternion (W/X/Y/Z are public fields, not properties).
    /// </summary>
    public string OrientationText =>
        $"W={Fmt3(Orientation.W)}  X={Fmt3(Orientation.X)}  Y={Fmt3(Orientation.Y)}  Z={Fmt3(Orientation.Z)}";

    public string AccelGText =>
        $"{Fmt3(AccelG.X)} / {Fmt3(AccelG.Y)} / {Fmt3(AccelG.Z)}";

    public string GyroDpsText =>
        $"{Fmt1(GyroDps.X)} / {Fmt1(GyroDps.Y)} / {Fmt1(GyroDps.Z)}";

    public string CalibrationStatusText
    {
        get
        {
            if (IsCalibrating)
            {
                return "calibrating gyro — keep Hub still";
            }
            if (!IsGyroBiasCalibrated)
            {
                return "gyro not calibrated (Cube may drift)";
            }
            const float radToDps = 180f / MathF.PI;
            return $"bias: {Fmt2(_gyroBiasRadS.X * radToDps)} / {Fmt2(_gyroBiasRadS.Y * radToDps)} / {Fmt2(_gyroBiasRadS.Z * radToDps)} dps";
        }
    }

    partial void OnMadgwickBetaChanged(float value)
    {
        _filter.Beta = value;
    }

    /// <summary>
    /// Drains all samples currently queued in the aggregator and runs Madgwick
    /// once per sample with the actual inter-sample dt recovered from the Hub
    /// timestamps. Called by MainWindow's 60 Hz DispatcherTimer; on a typical
    /// frame stream (BATCH=13, ODR=833) this drains ~14 samples per tick.
    /// </summary>
    public void Tick()
    {
        AggregatedSample? lastSample = null;
        while (_aggregator.TryRead(out AggregatedSample sample))
        {
            float dt = ComputeDt(sample.TimestampUs);

            if (IsCalibrating)
            {
                _calibrationSum += sample.GyroRadS;
                _calibrationSamplesRemaining--;
                if (_calibrationSamplesRemaining <= 0)
                {
                    _gyroBiasRadS = _calibrationSum / CalibrationSampleCount;
                    IsCalibrating = false;
                    IsGyroBiasCalibrated = true;
                    StatusText = "gyro calibrated";
                    _filter.Reset();
                }
            }
            else
            {
                Vector3 corrected = sample.GyroRadS - _gyroBiasRadS;
                _filter.Update(sample.AccelG, corrected, dt);
            }

            lastSample = sample;
        }

        if (lastSample is not null)
        {
            AccelG = lastSample.AccelG;
            GyroDps = lastSample.GyroDps;
        }
        Orientation = _filter.Orientation;
        MeasuredFps = _fps.Compute();
    }

    private float ComputeDt(uint sampleTsUs)
    {
        if (!_hasPreviousSampleTs)
        {
            _previousSampleTsUs = sampleTsUs;
            _hasPreviousSampleTs = true;
            return FallbackDt;
        }
        // uint32 subtraction wraps cleanly across the 32-bit boundary.
        uint deltaUs = unchecked(sampleTsUs - _previousSampleTsUs);
        _previousSampleTsUs = sampleTsUs;
        if (deltaUs == 0)
        {
            return FallbackDt;
        }
        float dt = deltaUs / 1_000_000f;
        if (dt > MaxReasonableDt)
        {
            return FallbackDt;
        }
        return dt;
    }

    private void ResetSampleClock()
    {
        _hasPreviousSampleTs = false;
        _previousSampleTsUs = 0;
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
            ResetSampleClock();
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
        ResetSampleClock();
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

    /// <summary>
    /// Capture <see cref="CalibrationSampleCount"/> aggregated gyro samples,
    /// average them, and use the result as a per-axis bias subtracted before
    /// each Madgwick step. The user must hold the Hub stationary during the
    /// ~2-second window. Yaw is unobservable from the accelerometer alone, so
    /// without this step gyro Z DC offset accumulates as visible Cube spin.
    /// </summary>
    [RelayCommand]
    public void CalibrateGyro()
    {
        _calibrationSum = Vector3.Zero;
        _calibrationSamplesRemaining = CalibrationSampleCount;
        IsCalibrating = true;
        StatusText = "calibrating gyro — keep Hub still";
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

    private static string Fmt1(float v) => v.ToString("+0.0;-0.0;0.0", CultureInfo.InvariantCulture);
    private static string Fmt2(float v) => v.ToString("+0.00;-0.00;0.00", CultureInfo.InvariantCulture);
    private static string Fmt3(float v) => v.ToString("+0.000;-0.000;0.000", CultureInfo.InvariantCulture);
}
