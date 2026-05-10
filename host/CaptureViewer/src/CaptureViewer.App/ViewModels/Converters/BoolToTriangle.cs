using System.Globalization;

using Avalonia.Data.Converters;

namespace CaptureViewer.App.ViewModels.Converters;

/// <summary>
/// Maps a boolean expand/collapse state to a Unicode disclosure
/// triangle ("▼" expanded / "▶" collapsed) for use as a tiny toggle
/// button glyph in the loaded-captures list.
/// </summary>
public sealed class BoolToTriangle : IValueConverter
{
    public static readonly BoolToTriangle Instance = new();

    public object? Convert(object? value, Type targetType,
                           object? parameter, CultureInfo culture)
        => (value is bool expanded && expanded) ? "▼" : "▶";

    public object? ConvertBack(object? value, Type targetType,
                               object? parameter, CultureInfo culture)
        => Avalonia.Data.BindingOperations.DoNothing;
}
