using System.Collections.ObjectModel;
using System.Globalization;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuViewer.App.Services;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Calibration;
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

    // Issue #139: BundleReceived runs on the reader thread (not the UI
    // thread).  Latch the three header fields (ODR, accel-FSR, gyro-FSR)
    // as one immutable record reference so a Tick that reads them sees
    // a consistent trio — three independent volatile writes would let a
    // racing reader observe an ODR from frame N+1 alongside an FSR
    // still at frame N (Codex 5th review).  Volatile.Write/Read on the
    // single reference is atomic per the .NET memory model.
    //
    // _liveSnapshot is null until the first BUNDLE arrives; Connect's
    // GET path sets the bound properties before streaming starts, so
    // the UI shows a valid value while we wait.
    private sealed record ImuLiveSnapshot(uint OdrHz, byte AccelFsrG,
                                          ushort GyroFsrDps);
    private ImuLiveSnapshot? _liveSnapshot;

    // Issue #139: set by BundleReceived, cleared by Tick().  We only
    // push the latched values onto the ObservableProperties when a
    // fresh BUNDLE has arrived since the last UI tick; without this
    // flag the stale snapshot would clobber any user edit made while
    // IMU streaming is OFF (no BUNDLE frames flowing → snapshot stays
    // pinned to the last good value and Tick would overwrite the
    // user's ComboBox selection on every 16 ms wakeup).
    //
    // Set after _liveSnapshot is published (release ordering) and read
    // before _liveSnapshot is consumed (acquire ordering) so a Tick
    // that sees _liveDirty=true is guaranteed to see the matching
    // snapshot.
    private volatile bool _liveDirty;

    // Set while ConnectAsync / ImuOnAsync push GET-derived values into
    // the [ObservableProperty]s so the OnXxxChanged partials skip the
    // round-trip SET that would otherwise fire on the same value the
    // firmware just told us about.
    private bool _suppressConfigSet;

    private readonly object _setLock = new();
    private CancellationTokenSource _setCts = new();

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
        _orchestrator.BundleReceived += frame =>
        {
            _fps.Mark();
            // Issue #139: publish the snapshot reference atomically,
            // then raise _liveDirty.  Tick consumes dirty first, then
            // reads the snapshot — so a true reading of _liveDirty
            // guarantees the matching snapshot is already visible
            // (release/acquire ordering via volatile bool + Volatile.*).
            Volatile.Write(ref _liveSnapshot, new ImuLiveSnapshot(
                frame.Header.ImuSampleRateHz,
                frame.Header.ImuAccelFsrG,
                frame.Header.ImuGyroFsrDps));
            _liveDirty = true;
        };

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
        Capture = new CaptureViewModel(orchestrator);

        _legoAggregator.StatusChanged += OnLegoStatusChanged;
        _legoAggregator.SampleReceived += OnLegoSampleReceived;
        _legoAggregator.PortChanged += OnLegoPortChanged;
    }

    /// <summary>Single-class write panel for the sidebar.</summary>
    public SensorWriteViewModel SensorWrite { get; }

    /// <summary>Phase 2.5 (#145) Capture panel.</summary>
    public CaptureViewModel Capture { get; }

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
    [NotifyPropertyChangedFor(nameof(CanEditImuConfig))]
    private bool _isConnected;

    partial void OnIsConnectedChanged(bool value)
    {
        // Forward connected state to the Capture panel so the Start
        // button enables / disables in lockstep with the BT session.
        Capture.IsConnected = value;
    }

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
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
    [NotifyPropertyChangedFor(nameof(OrientationWText))]
    [NotifyPropertyChangedFor(nameof(OrientationXyzText))]
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
    /// handler so live edits take effect on the next sample. The default
    /// assumes offline calibration (Issue #146) is applied; users running
    /// raw should raise this.
    /// </summary>
    [ObservableProperty]
    private float _gyroEpsilonDps = 0.5f;

    /// <summary>
    /// Default β for per-sample integration at chip ODR. The original 0.1
    /// was tuned for a 60 Hz integration rate; running ~14× faster needs
    /// a roughly proportional reduction. 0.05 balances accel correction
    /// speed against gyro smoothness when offline cal leaves the gyro
    /// near zero at rest.
    /// </summary>
    [ObservableProperty]
    private float _madgwickBeta = 0.05f;

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
    /// editable. Issue #139: the firmware now accepts SET while the IMU
    /// is active, so the inputs are unlocked any time we are connected.
    /// The partial <c>OnXxxChanged</c> handlers push edits straight to
    /// the driver, and the BUNDLE header round-trips the result back to
    /// the same property the next tick.</summary>
    public bool CanEditImuConfig => IsConnected;

    /// <summary>
    /// Workaround for Avalonia binding paths not resolving fields on
    /// System.Numerics.Quaternion (W/X/Y/Z are public fields, not properties).
    /// </summary>
    public string OrientationText =>
        $"W={Fmt3(Orientation.W)}  X={Fmt3(Orientation.X)}  Y={Fmt3(Orientation.Y)}  Z={Fmt3(Orientation.Z)}";

    /// <summary>Quaternion W component, formatted for the split
    /// Telemetry row that keeps the section narrow enough to match
    /// the other Expanders' widths.</summary>
    public string OrientationWText => Fmt3(Orientation.W);

    /// <summary>Quaternion X/Y/Z formatted as a single triple to
    /// match the accel / gyro rows visually.</summary>
    public string OrientationXyzText =>
        $"{Fmt3(Orientation.X)} / {Fmt3(Orientation.Y)} / {Fmt3(Orientation.Z)}";

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
    /// Issue #146: offline IMU calibration controls.  When
    /// <see cref="IsImuCalEnabled"/> is true and <see cref="ImuCalPath"/>
    /// resolves to a valid imu_cal.txt, the parsed cal is pushed to
    /// <see cref="SensorAggregator.Calibration"/> so every subsequent
    /// BUNDLE applies the matmul + bias before FSR scaling.  Both
    /// observables route through <see cref="ApplyImuCal"/>; the user
    /// can toggle the checkbox or pick a different file at runtime
    /// and the next BT bundle picks up the change immediately.
    /// </summary>
    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(ImuCalStatusText))]
    private bool _isImuCalEnabled;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(ImuCalStatusText))]
    private string _imuCalPath = "";

    [ObservableProperty]
    private string _imuCalStatusText = "no calibration loaded";

    partial void OnIsImuCalEnabledChanged(bool value) => ApplyImuCal();

    partial void OnImuCalPathChanged(string value) => ApplyImuCal();

    private void ApplyImuCal()
    {
        if (!IsImuCalEnabled)
        {
            _aggregator.Calibration = null;
            ImuCalStatusText = "disabled (raw values)";
            return;
        }
        if (string.IsNullOrWhiteSpace(ImuCalPath))
        {
            _aggregator.Calibration = null;
            ImuCalStatusText = "no file selected";
            return;
        }
        try
        {
            ImuCalibration cal = ImuCalibration.Load(ImuCalPath);
            _aggregator.Calibration = cal;
            ImuCalStatusText =
                $"loaded · FSR ±{cal.FsrGyDps}dps/±{cal.FsrXlG}g · " +
                $"ODR {cal.OdrHz}Hz · T {cal.AmbientTempC:0.0}°C";
        }
        catch (Exception ex)
        {
            _aggregator.Calibration = null;
            ImuCalStatusText = $"load failed: {ex.Message}";
        }
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

        // Phase 2.5 (#145): pull recorder stats onto the bound
        // observable properties for the Capture panel.
        Capture.Tick();

        // Issue #139: only push the latched config onto the bound
        // properties when a fresh BUNDLE arrived since the last tick.
        // Without the _liveDirty gate, the stored snapshot would
        // overwrite the user's ComboBox change while IMU streaming is
        // OFF (no BUNDLE flowing) — the user picks 833 Hz, the next
        // 16 ms tick clobbers it with the stale 416 from before, and
        // the subsequent IMU ON re-sends the old value.
        if (_liveDirty)
        {
            _liveDirty = false;
            ImuLiveSnapshot? snap = Volatile.Read(ref _liveSnapshot);
            if (snap is not null)
            {
                _suppressConfigSet = true;
                try
                {
                    if (snap.OdrHz != 0 && OdrHz != (int)snap.OdrHz)
                    {
                        OdrHz = (int)snap.OdrHz;
                    }
                    if (snap.AccelFsrG != 0 && AccelFsrG != snap.AccelFsrG)
                    {
                        AccelFsrG = snap.AccelFsrG;
                    }
                    if (snap.GyroFsrDps != 0 && GyroFsrDps != snap.GyroFsrDps)
                    {
                        GyroFsrDps = snap.GyroFsrDps;
                    }
                }
                finally
                {
                    _suppressConfigSet = false;
                }
            }
        }
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
            // Issue #139: clear any leftover live latch from a previous
            // session before the new Connect's GET path writes the bound
            // properties.  Without this a half-completed previous
            // Disconnect could leave _liveDirty=true, immediately
            // overwriting our just-fetched values on the next tick.
            _liveDirty = false;
            Volatile.Write(ref _liveSnapshot, null);
            StatusText = $"connecting {SelectedPort.BdAddr}...";
            await _orchestrator.ConnectAsync(SelectedPort.BdAddr, channel: 1, ct);
            await EnsureReplyAsync(_orchestrator.ImuOffAsync(ct), "IMU OFF");

            // Issue #139: read the firmware's current config instead of
            // pushing our defaults.  GET returns the driver-internal
            // enum idx; ImuConfigTables maps it to the physical value
            // that backs the ComboBox / UI.  The _suppressConfigSet
            // guard stops the property setter from re-issuing the SET
            // we would otherwise round-trip.
            int odrIdx = await GetIdxAsync(_orchestrator.GetOdrAsync(ct), "GET ODR");
            int xlIdx  = await GetIdxAsync(_orchestrator.GetAccelFsrAsync(ct), "GET ACCEL_FSR");
            int gyIdx  = await GetIdxAsync(_orchestrator.GetGyroFsrAsync(ct), "GET GYRO_FSR");
            _suppressConfigSet = true;
            try
            {
                OdrHz      = ImuConfigTables.OdrHz(odrIdx);
                AccelFsrG  = ImuConfigTables.AccelFsrG(xlIdx);
                GyroFsrDps = ImuConfigTables.GyroFsrDps(gyIdx);
            }
            finally
            {
                _suppressConfigSet = false;
            }

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
        // Phase 2.5 (#145): close any in-flight capture session before
        // tearing the BT stream down so the .bin file is flushed and
        // the stats reflect the final state.
        Capture.OnDisconnected();
        await _orchestrator.DisconnectAsync();
        IsConnected = false;
        IsImuOn = false;
        IsSensorOn = false;
        IsStationary = false;
        _fps.Reset();
        _biasTracker.Detector.Reset();
        ResetSampleClock();
        // Issue #139: drop the per-session BUNDLE config latch so the
        // next Connect's GET-derived values are not overwritten by a
        // stale dirty flag from the previous session (Codex 5th review).
        _liveDirty = false;
        Volatile.Write(ref _liveSnapshot, null);
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
        Capture.Dispose();
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

    // Issue #139: GET reply parser.  Firmware returns "OK <idx>" where
    // idx is a small uint; reject anything else so a transport glitch
    // does not silently stash garbage in the UI properties.
    private static async Task<int> GetIdxAsync(Task<BtsensorReply> task, string desc)
    {
        BtsensorReply r = await task;
        if (r is BtsensorReply.Ok ok && ok.Payload is { } payload
            && int.TryParse(payload, NumberStyles.Integer,
                            CultureInfo.InvariantCulture, out int value)
            && value >= 0)
        {
            return value;
        }

        throw new InvalidOperationException($"{desc} -> {r}");
    }

    // Issue #139: live SET wiring.  When the user picks a new value in
    // a ComboBox the ObservableProperty setter fires the corresponding
    // partial, which forwards the change to the firmware.  Suppress is
    // toggled while ConnectAsync / Tick() write the GET-derived or
    // BUNDLE-derived value back into the same property so the SET does
    // not loop back to the Hub.
    partial void OnOdrHzChanged(int value)
    {
        if (_suppressConfigSet || !IsConnected) return;
        FireConfigSet(_orchestrator.SetOdrAsync(value, _setCts.Token),
                      $"SET ODR {value}");
    }

    partial void OnAccelFsrGChanged(int value)
    {
        if (_suppressConfigSet || !IsConnected) return;
        FireConfigSet(_orchestrator.SetAccelFsrAsync(value, _setCts.Token),
                      $"SET ACCEL_FSR {value}");
    }

    partial void OnGyroFsrDpsChanged(int value)
    {
        if (_suppressConfigSet || !IsConnected) return;
        FireConfigSet(_orchestrator.SetGyroFsrAsync(value, _setCts.Token),
                      $"SET GYRO_FSR {value}");
    }

    private void FireConfigSet(Task<BtsensorReply> sendTask, string desc)
    {
        _ = Task.Run(async () =>
        {
            string? message = null;
            try
            {
                BtsensorReply r = await sendTask;
                if (!r.IsOk)
                {
                    message = $"{desc} -> {r}";
                }
            }
            catch (Exception ex)
            {
                message = $"{desc} failed: {ex.Message}";
            }

            if (message is not null)
            {
                // Issue #139: StatusText is bound to UI; route the
                // update through the Avalonia dispatcher so we never
                // raise PropertyChanged from the Task.Run worker
                // thread (Codex 5th review).
                string captured = message;
                Dispatcher.UIThread.Post(() => StatusText = captured);
            }
        });
    }

    private static string Fmt1(float v) => v.ToString("+0.0;-0.0;0.0", CultureInfo.InvariantCulture);
    private static string Fmt2(float v) => v.ToString("+0.00;-0.00;0.00", CultureInfo.InvariantCulture);
    private static string Fmt3(float v) => v.ToString("+0.000;-0.000;0.000", CultureInfo.InvariantCulture);
}
