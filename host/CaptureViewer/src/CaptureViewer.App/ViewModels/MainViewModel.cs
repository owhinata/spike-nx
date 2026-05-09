using System.Collections.ObjectModel;
using System.Runtime.InteropServices;

using Avalonia.Threading;

using CaptureViewer.App.Services;
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Live;

using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace CaptureViewer.App.ViewModels;

/// <summary>
/// Top-level VM.  Holds the list of loaded captures, the selected
/// channel/field name, and the BT connection state.  The view binds
/// to <see cref="Loaded"/>, <see cref="LogLines"/>, and the various
/// commands.  Plot redraw is driven by a <see cref="PlotInvalidated"/>
/// event raised whenever Loaded / SelectedFieldName / per-row
/// IsVisible changes.
/// </summary>
public sealed partial class MainViewModel : ObservableObject, IAsyncDisposable
{
    /// <summary>Color palette for newly loaded captures.</summary>
    private static readonly uint[] Palette =
    [
        0xFF80B0FFu,    // soft blue
        0xFFFFB070u,    // amber
        0xFF80E090u,    // mint
        0xFFFF8090u,    // rose
        0xFFC080FFu,    // lavender
        0xFFFFE060u,    // gold
        0xFF60D0E0u,    // cyan
        0xFFE090C0u,    // pink
    ];

    private readonly BtConnection _btConnection;
    private readonly CaptureFileWriter _captureWriter;
    private int _liveCounter;

    public ObservableCollection<LoadedCaptureViewModel> Loaded { get; } = new();
    public ObservableCollection<string> LogLines { get; } = new();
    public ObservableCollection<string> AvailableFields { get; } = new();

    [ObservableProperty]
    private string _bdAddr = string.Empty;

    [ObservableProperty]
    private string _outputDirectory = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
        "captures");

    [ObservableProperty]
    private string? _selectedFieldName;

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private string _statusText = "Idle";

    /// <summary>
    /// Raised whenever the plot needs to redraw — new capture loaded,
    /// visibility toggled, field selection changed.  The view code-
    /// behind subscribes once and calls <c>AvaPlot.Refresh()</c>.
    /// </summary>
    public event Action? PlotInvalidated;

    public MainViewModel()
    {
        _captureWriter = new CaptureFileWriter(_outputDirectory);
        _btConnection = new BtConnection(OnLiveSession, AppendLog);
    }

    [RelayCommand]
    private async Task OpenFileAsync()
    {
        var top = Avalonia.Application.Current?.ApplicationLifetime
            is Avalonia.Controls.ApplicationLifetimes.IClassicDesktopStyleApplicationLifetime desktop
            ? desktop.MainWindow
            : null;
        if (top is null) return;

        var files = await top.StorageProvider.OpenFilePickerAsync(
            new Avalonia.Platform.Storage.FilePickerOpenOptions
            {
                Title = "Open .cap files",
                AllowMultiple = true,
                FileTypeFilter =
                [
                    new Avalonia.Platform.Storage.FilePickerFileType("Capture files")
                    {
                        Patterns = ["*.cap"],
                    },
                ],
            });

        foreach (var file in files)
        {
            try
            {
                var path = file.Path.LocalPath;
                var bytes = await File.ReadAllBytesAsync(path);
                AddCapture(CaptureFile.Parse(bytes), Path.GetFileName(path));
            }
            catch (Exception ex)
            {
                AppendLog($"Open failed: {ex.Message}");
            }
        }
    }

    [RelayCommand]
    private void ClearAll()
    {
        Loaded.Clear();
        AvailableFields.Clear();
        SelectedFieldName = null;
        StatusText = "Cleared";
        PlotInvalidated?.Invoke();
    }

    [RelayCommand(CanExecute = nameof(CanConnect))]
    private async Task ConnectAsync()
    {
        if (string.IsNullOrWhiteSpace(BdAddr))
        {
            StatusText = "BD address is empty";
            return;
        }

        if (!RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
        {
            StatusText = "Live BT receive is Linux-only in v1";
            AppendLog(StatusText);
            return;
        }

        try
        {
            StatusText = $"Connecting to {BdAddr}...";
            await _btConnection.ConnectAsync(BdAddr);
            IsConnected = true;
            StatusText = "Connected";
        }
        catch (Exception ex)
        {
            StatusText = $"Connect failed: {ex.Message}";
            AppendLog(StatusText);
        }
    }

    private bool CanConnect() => !IsConnected;

    [RelayCommand(CanExecute = nameof(CanDisconnect))]
    private async Task DisconnectAsync()
    {
        await _btConnection.DisposeAsync();
        IsConnected = false;
        StatusText = "Disconnected";
    }

    private bool CanDisconnect() => IsConnected;

    [RelayCommand(CanExecute = nameof(CanDisconnect))]
    private async Task TriggerCaptureModeAsync()
    {
        try
        {
            await _btConnection.TriggerCaptureModeAsync();
        }
        catch (Exception ex)
        {
            AppendLog($"MODE CAPTURE failed: {ex.Message}");
        }
    }

    /// <summary>
    /// Invoked by the per-row CSV button — pops a save-file picker
    /// and writes the selected capture out as CSV.  Wired into each
    /// <see cref="LoadedCaptureViewModel"/> at construction time so
    /// the row VM can bind to a local command and dodge Avalonia's
    /// cross-DataTemplate binding cast issue.
    /// </summary>
    private async Task ExportRowCsvAsync(LoadedCaptureViewModel row)
    {
        var top = Avalonia.Application.Current?.ApplicationLifetime
            is Avalonia.Controls.ApplicationLifetimes.IClassicDesktopStyleApplicationLifetime desktop
            ? desktop.MainWindow
            : null;
        if (top is null) return;

        var file = await top.StorageProvider.SaveFilePickerAsync(
            new Avalonia.Platform.Storage.FilePickerSaveOptions
            {
                Title = "Save CSV",
                SuggestedFileName =
                    $"{row.Capture.SchemaName}_{row.Capture.StartTimestampUs}.csv",
                DefaultExtension = "csv",
            });
        if (file is null) return;

        try
        {
            CsvExporter.Save(row.Capture, file.Path.LocalPath);
            AppendLog($"CSV saved to {file.Path.LocalPath}");
        }
        catch (Exception ex)
        {
            AppendLog($"CSV export failed: {ex.Message}");
        }
    }

    public async ValueTask DisposeAsync()
    {
        await _btConnection.DisposeAsync();
    }

    /// <summary>
    /// Wired to <see cref="BtConnection"/> — invoked on the receiver
    /// task thread.  We marshal back to the UI thread before mutating
    /// <see cref="Loaded"/> so the bound list view does not throw.
    /// </summary>
    private void OnLiveSession(SessionScan scan)
    {
        Dispatcher.UIThread.Post(() =>
        {
            try
            {
                var saved = _captureWriter.Save(scan.Capture);
                AppendLog($"Saved live capture to {saved}");
            }
            catch (Exception ex)
            {
                AppendLog($"Live save failed: {ex.Message}");
            }

            _liveCounter++;
            AddCapture(scan.Capture, $"live #{_liveCounter}");
        });
    }

    private void AddCapture(CaptureFile capture, string label)
    {
        var color = Palette[Loaded.Count % Palette.Length];
        Loaded.Add(new LoadedCaptureViewModel(capture, label, color, ExportRowCsvAsync));

        // Refresh the field selector with the union of all currently
        // loaded captures' field names (excluding ts_us, which is the
        // x-axis).
        var union = new SortedSet<string>(StringComparer.Ordinal);
        foreach (var lc in Loaded)
        {
            foreach (var f in lc.Capture.Fields)
            {
                if (f.Name != "ts_us") union.Add(f.Name);
            }
        }
        AvailableFields.Clear();
        foreach (var name in union) AvailableFields.Add(name);
        if (SelectedFieldName is null && AvailableFields.Count > 0)
        {
            SelectedFieldName = AvailableFields[0];
        }

        StatusText = $"Loaded {label} ({capture.SchemaName}, {capture.RecordCount} records)";
        PlotInvalidated?.Invoke();
    }

    private void AppendLog(string line)
    {
        Dispatcher.UIThread.Post(() =>
        {
            LogLines.Add($"{DateTime.Now:HH:mm:ss}  {line}");
            while (LogLines.Count > 200) LogLines.RemoveAt(0);
        });
    }

    partial void OnSelectedFieldNameChanged(string? value)
    {
        PlotInvalidated?.Invoke();
    }

    partial void OnIsConnectedChanged(bool value)
    {
        ConnectCommand.NotifyCanExecuteChanged();
        DisconnectCommand.NotifyCanExecuteChanged();
        TriggerCaptureModeCommand.NotifyCanExecuteChanged();
    }
}
