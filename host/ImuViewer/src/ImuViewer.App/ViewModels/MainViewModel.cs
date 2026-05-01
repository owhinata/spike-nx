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
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Transport;
using ImuViewer.Core.Wire;

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
    private readonly LegoSampleAggregator _legoAggregator;
    private readonly MadgwickFilter _filter;
    private readonly GyroBiasTracker _biasTracker;
    private readonly FpsCounter _fps = new();

    private readonly Dictionary<LegoClassId, SensorPanelViewModel> _panelByClass;

    private Vector3 _calibrationSum = Vector3.Zero;
    private int _calibrationSamplesRemaining;
    private uint _previousSampleTsUs;
    private bool _hasPreviousSampleTs;

    public MainViewModel(
        IBluetoothPortEnumerator portEnumerator,
        SessionOrchestrator orchestrator,
        SensorAggregator aggregator,
        LegoSampleAggregator legoAggregator,
        MadgwickFilter filter,
        GyroBiasTracker biasTracker)
    {
        _portEnumerator = portEnumerator;
        _orchestrator = orchestrator;
        _aggregator = aggregator;
        _legoAggregator = legoAggregator;
        _filter = filter;
        _biasTracker = biasTracker;
        _orchestrator.BundleReceived += _ => _fps.Mark();

        // Pre-populate the six sensor panels in fixed enum order so the
        // UI is stable across attach/detach.  Panels are read-only —
        // the write side lives in SensorWriteViewModel on the sidebar.
        SensorPanelViewModel[] panels = new SensorPanelViewModel[]
        {
            new(LegoClassId.Color),
            new(LegoClassId.Ultrasonic),
            new(LegoClassId.Force),
            new(LegoClassId.MotorM),
            new(LegoClassId.MotorR),
            new(LegoClassId.MotorL),
        };
        SensorPanels = panels;
        _panelByClass = panels.ToDictionary(p => p.ClassId);

        SensorWrite = new SensorWriteViewModel(orchestrator);

        _legoAggregator.StatusChanged += OnLegoStatusChanged;
        _legoAggregator.SampleReceived += OnLegoSampleReceived;
        _legoAggregator.PortChanged += OnLegoPortChanged;
    }

    /// <summary>Single-class write panel for the sidebar.</summary>
    public SensorWriteViewModel SensorWrite { get; }

    private void OnLegoStatusChanged(LegoClassId classId, LegoSampleAggregator.ClassState state)
    {
        if (_panelByClass.TryGetValue(classId, out SensorPanelViewModel? panel))
        {
            panel.ApplyStatus(state);
        }
    }

    private void OnLegoSampleReceived(LegoClassId classId, LegoSamplePoint sample)
    {
        if (_panelByClass.TryGetValue(classId, out SensorPanelViewModel? panel))
        {
            panel.AppendSample(sample);
        }
    }

    private void OnLegoPortChanged(LegoClassId classId, LegoSampleAggregator.ClassState state)
    {
        if (_panelByClass.TryGetValue(classId, out SensorPanelViewModel? panel))
        {
            panel.ResetPlot();
        }
    }

    /// <summary>Six fixed panels (Color/Ultrasonic/Force/MotorM/MotorR/MotorL).</summary>
    public IReadOnlyList<SensorPanelViewModel> SensorPanels { get; }

    [ObservableProperty]
    private ObservableCollection<BluetoothPort> _ports = new();

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanConnect))]
    private BluetoothPort? _selectedPort;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanConnect))]
    [NotifyPropertyChangedFor(nameof(CanDisconnect))]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    [NotifyPropertyChangedFor(nameof(CanToggleSensor))]
    private bool _isConnected;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    [NotifyPropertyChangedFor(nameof(CanEditImuConfig))]
    private bool _isImuOn;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanToggleSensor))]
    private bool _isSensorOn;

    public bool CanToggleSensor => IsConnected;

    partial void OnIsSensorOnChanged(bool value) => SensorWrite?.SetWriteEnabled(value);

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

    /// <summary>
    /// LSM6DSL-supported output data rates that btsensor accepts.  >833 Hz
    /// is rejected by the firmware (Issue #88) since the BUNDLE 100 Hz tick
    /// caps imu_sample_count at 8 and would otherwise drop most samples.
    /// </summary>
    public static int[] OdrOptions { get; } =
        new[] { 13, 26, 52, 104, 208, 416, 833 };

    /// <summary>LSM6DSL-supported accelerometer full-scale ranges, in g.</summary>
    public static int[] AccelFsrOptions { get; } = new[] { 2, 4, 8, 16 };

    /// <summary>LSM6DSL-supported gyroscope full-scale ranges, in dps.</summary>
    public static int[] GyroFsrOptions { get; } = new[] { 125, 250, 500, 1000, 2000 };

    [ObservableProperty]
    private int _odrHz = 833;

    [ObservableProperty]
    private int _accelFsrG = 8;

    [ObservableProperty]
    private int _gyroFsrDps = 2000;

    /// <summary>
    /// Stationary-detection per-axis gyro threshold, in dps. Mirrors
    /// <see cref="StationaryDetector.GyroEpsilonDps"/> via the partial change
    /// handler so live edits take effect on the next sample.
    /// </summary>
    [ObservableProperty]
    private float _gyroEpsilonDps = 5.0f;

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

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CalibrationStatusText))]
    private bool _isStationary;

    public bool CanConnect => !IsConnected && SelectedPort is not null;
    public bool CanDisconnect => IsConnected;
    public bool CanToggleImu => IsConnected;

    /// <summary>True when the IMU configuration inputs (ODR, FSRs) are
    /// editable. The Hub firmware only accepts SET ODR/ACCEL_FSR/GYRO_FSR
    /// while IMU is OFF, so the inputs are locked exactly while IMU is streaming.
    /// Edits made while connected and IMU OFF are pushed to the Hub by
    /// <see cref="ImuOnAsync"/> just before starting the stream.</summary>
    public bool CanEditImuConfig => !IsImuOn;

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
            const float radToDps = 180f / MathF.PI;
            Vector3 b = _biasTracker.BiasRadS;
            string biasPart = IsGyroBiasCalibrated || b != Vector3.Zero
                ? $"bias: {Fmt2(b.X * radToDps)} / {Fmt2(b.Y * radToDps)} / {Fmt2(b.Z * radToDps)} dps"
                : "bias: not yet measured";
            string autoPart = IsStationary ? "auto: tracking" : "auto: hold";
            return $"{biasPart} · {autoPart}";
        }
    }

    partial void OnMadgwickBetaChanged(float value)
    {
        _filter.Beta = value;
    }

    partial void OnGyroEpsilonDpsChanged(float value)
    {
        _biasTracker.Detector.GyroEpsilonDps = value;
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
                    _biasTracker.SetBias(_calibrationSum / CalibrationSampleCount);
                    IsCalibrating = false;
                    IsGyroBiasCalibrated = true;
                    StatusText = "gyro calibrated";
                    _filter.Reset();
                }
                // Skip orientation integration during the calibration window
                // — gyro hasn't been bias-corrected yet.
            }
            else
            {
                // Bias tracker subtracts the running bias and, when stationary,
                // pushes the bias toward the live gyro reading via LPF (Issue
                // #61). After enough stationary time the bias becomes usable
                // even without a manual Calibrate gyro press.
                Vector3 corrected = _biasTracker.Update(
                    sample.AccelG, sample.GyroRadS, dt);
                _filter.Update(sample.AccelG, corrected, dt);
            }

            lastSample = sample;
        }

        if (lastSample is not null)
        {
            AccelG = lastSample.AccelG;
            GyroDps = lastSample.GyroDps;
        }
        // Latch UI-visible flags from the tracker once per tick so XAML bindings
        // only update at 60 Hz.
        IsStationary = _biasTracker.IsStationary;
        if (!IsGyroBiasCalibrated && _biasTracker.IsStationary
            && _biasTracker.BiasRadS != Vector3.Zero)
        {
            IsGyroBiasCalibrated = true;
        }
        OnPropertyChanged(nameof(CalibrationStatusText));
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
            await EnsureReplyAsync(_orchestrator.SetAccelFsrAsync(AccelFsrG, ct), $"SET ACCEL_FSR {AccelFsrG}");
            await EnsureReplyAsync(_orchestrator.SetGyroFsrAsync(GyroFsrDps, ct), $"SET GYRO_FSR {GyroFsrDps}");
            IsConnected = true;
            IsImuOn = false;
            IsSensorOn = false;
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
        IsSensorOn = false;
        IsStationary = false;
        _fps.Reset();
        _biasTracker.Detector.Reset();
        ResetSampleClock();
        StatusText = "disconnected";
    }

    [RelayCommand]
    public async Task ImuOnAsync(CancellationToken ct)
    {
        try
        {
            // Resync the three config knobs before starting the stream — the
            // user may have edited them while connected & IMU OFF, and the
            // firmware only accepts SET while IMU is OFF.
            await EnsureReplyAsync(_orchestrator.SetOdrAsync(OdrHz, ct), $"SET ODR {OdrHz}");
            await EnsureReplyAsync(_orchestrator.SetAccelFsrAsync(AccelFsrG, ct), $"SET ACCEL_FSR {AccelFsrG}");
            await EnsureReplyAsync(_orchestrator.SetGyroFsrAsync(GyroFsrDps, ct), $"SET GYRO_FSR {GyroFsrDps}");

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
    public async Task SensorOnAsync(CancellationToken ct)
    {
        try
        {
            BtsensorReply r = await _orchestrator.SensorOnAsync(ct);
            if (r.IsOk)
            {
                IsSensorOn = true;
                StatusText = "SENSOR streaming";
            }
            else
            {
                StatusText = "SENSOR ON: " + r;
            }
        }
        catch (Exception ex)
        {
            StatusText = "SENSOR ON failed: " + ex.Message;
        }
    }

    [RelayCommand]
    public async Task SensorOffAsync(CancellationToken ct)
    {
        try
        {
            BtsensorReply r = await _orchestrator.SensorOffAsync(ct);
            IsSensorOn = false;
            StatusText = r.IsOk ? "SENSOR stopped" : "SENSOR OFF: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "SENSOR OFF failed: " + ex.Message;
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
