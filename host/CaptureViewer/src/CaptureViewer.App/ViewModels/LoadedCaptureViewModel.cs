using Avalonia.Media;

using CaptureViewer.Core.Capture;

using CommunityToolkit.Mvvm.ComponentModel;

namespace CaptureViewer.App.ViewModels;

/// <summary>
/// One row in the "loaded captures" sidebar.  Carries the parsed
/// <see cref="CaptureFile"/> plus per-row UI state (visibility,
/// display color, source label).  The plot view subscribes to
/// changes via INotifyPropertyChanged so toggling
/// <see cref="IsVisible"/> redraws.
/// </summary>
public sealed partial class LoadedCaptureViewModel : ObservableObject
{
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

    public LoadedCaptureViewModel(CaptureFile capture, string label, uint colorArgb)
    {
        Capture = capture;
        Label = label;
        ColorArgb = colorArgb;
        ColorBrush = new SolidColorBrush(Color.FromUInt32(colorArgb));
    }

    public string SchemaName => Capture.SchemaName;
    public int RecordCount => Capture.RecordCount;
    public ulong StartTimestampUs => Capture.StartTimestampUs;
}
