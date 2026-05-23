using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuViewer.App.Services;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.Capture;
using ImuViewer.Core.Wire;

namespace ImuViewer.App.ViewModels;

/// <summary>
/// Phase 2.5 / Issue #145 sidebar panel.  Starts an `_IMU_CAP` session
/// on the Hub, opens a local `.bin` writer, and live-displays the
/// running stats from <see cref="ImuCaptureRecorder"/> so the operator
/// can decide whether the session is usable for Tedaldi calibration.
/// </summary>
/// <remarks>
/// The recorder is owned per-session — opened on Start, disposed on
/// Stop or on Disconnect.  Stats are read from the BTstack reader
/// thread, but the UI tick (driven by <see cref="MainViewModel.Tick"/>)
/// snapshots them onto observable properties so XAML bindings remain
/// on the UI thread.
///
/// The "seq drop = 0" acceptance criterion from the plan is exposed
/// as <see cref="IsSessionAcceptable"/> for visual feedback.  We do
/// not delete the `.bin` automatically on a bad session — the user
/// might still want to inspect it for diagnostics.
/// </remarks>
public sealed partial class CaptureViewModel : ObservableObject, IDisposable
{
    private readonly SessionOrchestrator _orchestrator;
    private ImuCaptureRecorder? _recorder;
    private FileStream? _file;
    private CancellationTokenSource _cts = new();
    /// <summary>
    /// Tied to the Hub's duration timer: when a capture starts with
    /// DurationSec > 0 we arm a matching DispatcherTimer on the host
    /// so the UI state (Start/Stop button enable, "saved N frames"
    /// finalisation, .bin Flush + Close) mirrors the Hub's auto-exit
    /// instead of waiting for the user to notice and click Stop.
    /// </summary>
    private DispatcherTimer? _durationTimer;
    private bool _disposed;

    public CaptureViewModel(SessionOrchestrator orchestrator)
    {
        _orchestrator = orchestrator;
        _orchestrator.ImuCapFrameReceived += OnImuCapFrame;

        // Default output path: timestamped file under the current
        // working directory so a `dotnet run` from a project folder
        // drops the capture next to the rest of the project's
        // artefacts.  Tedaldi pipeline expects raw 27 B frames, no
        // header.
        string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss",
            CultureInfo.InvariantCulture);
        OutputPath = Path.Combine(Directory.GetCurrentDirectory(),
            $"imu_cap_{ts}.bin");
    }

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanStart))]
    [NotifyPropertyChangedFor(nameof(CanStop))]
    private bool _isCapturing;

    /// <summary>
    /// Set to true when <see cref="MainViewModel"/> tells us the
    /// session is connected; without it `_IMU_CAP START` would just
    /// error out on the Hub.
    /// </summary>
    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanStart))]
    [NotifyPropertyChangedFor(nameof(CanStop))]
    private bool _isConnected;

    [ObservableProperty]
    private string _outputPath = string.Empty;

    /// <summary>
    /// 0 = run until the user hits Stop.  Tedaldi sessions are
    /// typically 240 s with ~10 static poses, so we default to that.
    /// </summary>
    [ObservableProperty]
    private uint _durationSec = 240;

    [ObservableProperty]
    private uint _frameCount;

    [ObservableProperty]
    private uint _dropCount;

    [ObservableProperty]
    private double _elapsedSec;

    [ObservableProperty]
    private double _temperatureC;

    [ObservableProperty]
    private string _statusText = "ready";

    public bool CanStart => IsConnected && !IsCapturing;
    public bool CanStop => IsConnected && IsCapturing;
    /// <summary>True while the Output file textbox + Browse button
    /// should accept edits (not during a capture session).</summary>
    public bool CanEditOutputPath => !IsCapturing;
    partial void OnIsCapturingChanged(bool value) =>
        OnPropertyChanged(nameof(CanEditOutputPath));

    /// <summary>
    /// "seq drop = 0" acceptance criterion (plan §B / §D).  Inspected
    /// by the UI to colour the stats panel red on a bad session.
    /// </summary>
    public bool IsSessionAcceptable => DropCount == 0;

    partial void OnDropCountChanged(uint value) =>
        OnPropertyChanged(nameof(IsSessionAcceptable));

    /// <summary>
    /// Pulled once per UI tick by MainViewModel so the bound stats
    /// reflect the cumulative recorder state without touching the
    /// observable properties from the BTstack reader thread.
    /// </summary>
    public void Tick()
    {
        if (_recorder is null)
        {
            return;
        }

        FrameCount   = _recorder.FrameCount;
        DropCount    = _recorder.DropCount;
        ElapsedSec   = _recorder.Elapsed.TotalSeconds;
        TemperatureC = 25.0 + _recorder.AverageTemperatureRaw / 256.0;
    }

    [RelayCommand]
    public async Task StartAsync(CancellationToken ct)
    {
        if (IsCapturing || !IsConnected)
        {
            return;
        }

        if (string.IsNullOrWhiteSpace(OutputPath))
        {
            StatusText = "output path is empty";
            return;
        }

        try
        {
            string? dir = Path.GetDirectoryName(OutputPath);
            if (!string.IsNullOrEmpty(dir))
            {
                Directory.CreateDirectory(dir);
            }
            _file = new FileStream(OutputPath, FileMode.Create,
                FileAccess.Write, FileShare.Read, bufferSize: 8192);
            _recorder = new ImuCaptureRecorder(_file, ownsStream: false);
        }
        catch (Exception ex)
        {
            StatusText = $"open failed: {ex.Message}";
            DisposeRecorder();
            return;
        }

        ResetCounters();

        BtsensorReply reply;
        try
        {
            reply = await _orchestrator.ImuCapStartAsync(DurationSec, ct);
        }
        catch (Exception ex)
        {
            StatusText = $"_IMU_CAP START failed: {ex.Message}";
            DisposeRecorder();
            return;
        }

        if (!reply.IsOk)
        {
            StatusText = $"_IMU_CAP START -> {reply}";
            DisposeRecorder();
            return;
        }

        IsCapturing = true;
        StatusText = DurationSec == 0
            ? $"capturing -> {OutputPath}"
            : $"capturing for {DurationSec}s -> {OutputPath}";

        // Mirror the Hub's duration timer so the UI auto-finalises
        // when the Hub auto-exits (a small grace period lets any
        // in-flight final frames land before we Stop).
        if (DurationSec > 0)
        {
            _durationTimer?.Stop();
            _durationTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromSeconds(DurationSec + 1),
            };
            _durationTimer.Tick += OnDurationElapsed;
            _durationTimer.Start();
        }
    }

    private async void OnDurationElapsed(object? sender, EventArgs e)
    {
        DisposeDurationTimer();
        if (!IsCapturing)
        {
            return;
        }

        try
        {
            // Best-effort STOP — the Hub's matching auto-exit has
            // already fired by now (idempotent on the Hub side), but
            // sending it again keeps the wire protocol symmetric and
            // releases any state we might be tracking.
            await _orchestrator.ImuCapStopAsync(CancellationToken.None);
        }
        catch
        {
            // Ignore; FinalizeSession still runs below.
        }

        FinalizeSession();
    }

    private void DisposeDurationTimer()
    {
        if (_durationTimer is not null)
        {
            _durationTimer.Stop();
            _durationTimer.Tick -= OnDurationElapsed;
            _durationTimer = null;
        }
    }

    [RelayCommand]
    public async Task StopAsync(CancellationToken ct)
    {
        if (!IsCapturing)
        {
            return;
        }

        // Manual Stop — cancel the duration-mirror timer first so it
        // doesn't fire after we've already finalised.
        DisposeDurationTimer();

        try
        {
            BtsensorReply reply = await _orchestrator.ImuCapStopAsync(ct);
            if (!reply.IsOk)
            {
                StatusText = $"_IMU_CAP STOP -> {reply}";
            }
        }
        catch (Exception ex)
        {
            StatusText = $"_IMU_CAP STOP failed: {ex.Message}";
        }

        FinalizeSession();
    }

    /// <summary>
    /// Called from <see cref="MainViewModel"/> on Disconnect — close
    /// the recorder cleanly without sending another `_IMU_CAP STOP`
    /// (the daemon teardown already handled it).
    /// </summary>
    public void OnDisconnected()
    {
        if (IsCapturing)
        {
            FinalizeSession();
        }

        IsConnected = false;
    }

    private void FinalizeSession()
    {
        // Snapshot the final stats before disposing the recorder so the
        // UI keeps showing the last value after Stop.
        if (_recorder is not null)
        {
            FrameCount   = _recorder.FrameCount;
            DropCount    = _recorder.DropCount;
            ElapsedSec   = _recorder.Elapsed.TotalSeconds;
            TemperatureC = 25.0 + _recorder.AverageTemperatureRaw / 256.0;
        }

        bool good = DropCount == 0 && FrameCount > 0;
        StatusText = good
            ? $"saved {FrameCount} frames ({ElapsedSec:F1}s), accept"
            : DropCount > 0
                ? $"saved {FrameCount} frames with {DropCount} drops — reject and retry"
                : "session ended with 0 frames";

        DisposeRecorder();
        IsCapturing = false;
    }

    private void DisposeRecorder()
    {
        _recorder?.Dispose();
        _recorder = null;
        _file?.Dispose();
        _file = null;
    }

    private void ResetCounters()
    {
        FrameCount   = 0;
        DropCount    = 0;
        ElapsedSec   = 0;
        TemperatureC = 0;
    }

    private void OnImuCapFrame(ImuCapFrame frame)
    {
        // Reader thread.  ImuCaptureRecorder is the single subscriber
        // so we update it directly without dispatching to the UI; the
        // UI ticks observable properties from Tick() on its own
        // thread.
        try
        {
            _recorder?.OnImuCapFrame(frame);
        }
        catch (Exception ex)
        {
            string captured = ex.Message;
            Dispatcher.UIThread.Post(() =>
            {
                StatusText = $"recorder error: {captured}";
                FinalizeSession();
            });
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        DisposeDurationTimer();
        _orchestrator.ImuCapFrameReceived -= OnImuCapFrame;
        DisposeRecorder();
        _cts.Dispose();
    }
}
