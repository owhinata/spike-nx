using CommunityToolkit.Mvvm.ComponentModel;

namespace CaptureViewer.App.ViewModels;

/// <summary>
/// Per-capture per-field visibility toggle.  Each loaded capture
/// owns one of these per non-<c>ts_us</c> field in its schema; the
/// sidebar renders them as a row of WrapPanel-laid-out checkboxes
/// so the user can flip individual traces on/off without losing the
/// rest of the row.  Toggling fires the optional <c>onChanged</c>
/// callback so the parent VM can invalidate the plot.
/// </summary>
public sealed partial class FieldVisibilityViewModel : ObservableObject
{
    private readonly Action? _onChanged;

    public string Name { get; }

    /// <summary>Selectable Y-axis labels shown in the per-field combo
    /// (1..4 — paired one-to-one with the toolbar's Y-range boxes).</summary>
    public IReadOnlyList<int> AxisChoices { get; } = new[] { 1, 2, 3, 4 };

    [ObservableProperty]
    private bool _isChecked = true;

    /// <summary>Selected axis (1-based to match the UI label).
    /// Default is computed from the field name in
    /// <see cref="DefaultAxisFor"/>.</summary>
    [ObservableProperty]
    private int _axis = 1;

    public FieldVisibilityViewModel(string name, Action? onChanged = null)
    {
        Name = name;
        // Every newly loaded field starts on Y1.  Per-name defaults
        // tend to surprise the user (different capture types landing
        // on different axes); the explicit per-row ComboBox is the
        // single source of truth.
        _axis = 1;
        _onChanged = onChanged;
    }

    partial void OnIsCheckedChanged(bool value) => _onChanged?.Invoke();
    partial void OnAxisChanged(int value)       => _onChanged?.Invoke();
}
