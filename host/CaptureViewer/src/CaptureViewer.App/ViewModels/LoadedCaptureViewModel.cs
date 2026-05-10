using System.Collections.ObjectModel;

using Avalonia.Media;

using CaptureViewer.Core.Capture;

using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace CaptureViewer.App.ViewModels;

/// <summary>
/// One row in the "loaded captures" sidebar.  Carries the parsed
/// <see cref="CaptureFile"/> plus per-row UI state (visibility,
/// display color, source label).  The plot view subscribes to
/// changes via INotifyPropertyChanged so toggling
/// <see cref="IsVisible"/> redraws.
///
/// Per-row commands (CSV export) live on this VM so the DataTemplate
/// can bind locally — `{Binding ExportCsvCommand}` — instead of
/// reaching across the DataTemplate scope back to the parent VM,
/// which Avalonia 11's compiled bindings cannot cast safely at
/// runtime when nested inside an ItemsControl.  The actual export
/// logic still lives in <see cref="MainViewModel"/>; the row holds a
/// thin handler delegate the parent injects at construction.
/// </summary>
public sealed partial class LoadedCaptureViewModel : ObservableObject
{
    private readonly Func<LoadedCaptureViewModel, Task>? _exportCsv;
    private readonly Action<LoadedCaptureViewModel>? _remove;

    public CaptureFile Capture { get; }

    /// <summary>
    /// Display name shown in the sidebar — file name for `.cap`
    /// files, "live #N" for sessions arrived over BT.
    /// </summary>
    public string Label { get; }

    /// <summary>
    /// Color used when this capture is plotted.  Assigned at
    /// construction so the sidebar swatch and the plotted line stay
    /// in sync without further binding gymnastics.
    /// </summary>
    public IBrush ColorBrush { get; }
    public uint ColorArgb { get; }

    [ObservableProperty]
    private bool _isVisible = true;

    /// <summary>
    /// Sidebar row collapse state.  When false, only the header
    /// (color swatch + visibility checkbox + label + remove button)
    /// is shown so the loaded-captures list stays compact even with
    /// 10+ captures loaded.  Defaults to collapsed — the label alone
    /// is enough at a glance, and the user expands a row only when
    /// they want the schema / record-count details or the CSV button.
    /// </summary>
    [ObservableProperty]
    private bool _isExpanded;

    public ObservableCollection<FieldVisibilityViewModel> Fields { get; } = new();

    public LoadedCaptureViewModel(
        CaptureFile capture,
        string label,
        uint colorArgb,
        Func<LoadedCaptureViewModel, Task>? exportCsv = null,
        Action<LoadedCaptureViewModel>? remove = null,
        Action? onFieldVisibilityChanged = null)
    {
        Capture = capture;
        Label = label;
        ColorArgb = colorArgb;
        ColorBrush = new SolidColorBrush(Color.FromUInt32(colorArgb));
        _exportCsv = exportCsv;
        _remove = remove;

        // Build per-field visibility toggles for everything except
        // ts_us (which is the X axis and never plotted as a series).
        foreach (var f in capture.Fields)
        {
            if (f.Name == "ts_us") continue;
            Fields.Add(new FieldVisibilityViewModel(f.Name, onFieldVisibilityChanged));
        }
    }

    public string SchemaName => Capture.SchemaName;
    public int RecordCount => Capture.RecordCount;
    public ulong StartTimestampUs => Capture.StartTimestampUs;

    [RelayCommand]
    private Task ExportCsv() =>
        _exportCsv?.Invoke(this) ?? Task.CompletedTask;

    [RelayCommand]
    private void Remove() => _remove?.Invoke(this);

    [RelayCommand]
    private void ToggleExpanded() => IsExpanded = !IsExpanded;
}
