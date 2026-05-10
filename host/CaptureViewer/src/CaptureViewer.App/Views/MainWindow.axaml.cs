using System.Collections.Specialized;

using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Markup.Xaml;
using Avalonia.Threading;

using CaptureViewer.App.ViewModels;
using CaptureViewer.Core;
using CaptureViewer.Core.Capture;

using ScottPlot;
using ScottPlot.Avalonia;

namespace CaptureViewer.App.Views;

public partial class MainWindow : Window
{
    /// <summary>
    /// Marker shapes used for the i-th visible capture in the overlay
    /// list.  Cycles so even with similar colors, captures stay
    /// distinguishable (and works in monochrome printouts of the
    /// rendered plot).
    /// </summary>
    private static readonly MarkerShape[] MarkerPalette =
    [
        MarkerShape.FilledCircle,
        MarkerShape.FilledSquare,
        MarkerShape.FilledTriangleUp,
        MarkerShape.FilledDiamond,
        MarkerShape.OpenCircle,
        MarkerShape.FilledTriangleDown,
    ];

    private AvaPlot? _avaPlot;
    private ScrollViewer? _logScroller;
    private IYAxis[] _yAxes = Array.Empty<IYAxis>();

    public MainWindow()
    {
        InitializeComponent();
        _avaPlot = this.FindControl<AvaPlot>("AvaPlot1");
        _logScroller = this.FindControl<ScrollViewer>("LogScroller");

        if (_avaPlot is not null)
        {
            // Stand up four left-stacked Y axes once.  Plot.Clear()
            // only drops plottables, so these references stay valid
            // across redraws.
            var p = _avaPlot.Plot;
            _yAxes = new IYAxis[]
            {
                p.Axes.Left,
                p.Axes.AddLeftAxis(),
                p.Axes.AddLeftAxis(),
                p.Axes.AddLeftAxis(),
            };
        }

        DataContextChanged += OnDataContextChanged;
        ConfigurePlotChrome();
        WirePlotDragDrop();
    }

    private void InitializeComponent() => AvaloniaXamlLoader.Load(this);

    private void OnDataContextChanged(object? sender, EventArgs e)
    {
        if (DataContext is MainViewModel vm)
        {
            vm.PlotInvalidated += RedrawPlot;
            vm.Loaded.CollectionChanged += (_, _) => RedrawPlot();
            vm.Loaded.CollectionChanged += (_, args) =>
            {
                if (args.NewItems is null) return;
                foreach (LoadedCaptureViewModel row in args.NewItems)
                {
                    row.PropertyChanged += (_, ev) =>
                    {
                        if (ev.PropertyName == nameof(LoadedCaptureViewModel.IsVisible))
                            RedrawPlot();
                    };
                }
            };
            vm.LogLines.CollectionChanged += OnLogLinesChanged;
            RedrawPlot();
        }
    }

    private void OnLogLinesChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        if (_logScroller is null) return;
        if (e.Action != NotifyCollectionChangedAction.Add) return;
        Dispatcher.UIThread.Post(() => _logScroller.ScrollToEnd(),
                                 DispatcherPriority.Background);
    }

    private void WirePlotDragDrop()
    {
        var host = this.FindControl<Grid>("PlotHost");
        if (host is null) return;
        host.AddHandler(DragDrop.DragOverEvent, OnDragOver);
        host.AddHandler(DragDrop.DropEvent, OnDrop);
    }

    private static void OnDragOver(object? sender, DragEventArgs e)
    {
        e.DragEffects = e.Data.Contains(DataFormats.Files)
            ? DragDropEffects.Copy : DragDropEffects.None;
    }

    private async void OnDrop(object? sender, DragEventArgs e)
    {
        if (DataContext is not MainViewModel vm) return;
        if (!e.Data.Contains(DataFormats.Files)) return;
        var files = e.Data.GetFiles();
        if (files is null) return;
        foreach (var item in files)
        {
            var path = item.Path.LocalPath;
            if (!path.EndsWith(".cap", StringComparison.OrdinalIgnoreCase)) continue;
            await vm.LoadCaptureFromPathAsync(path);
        }
    }

    private void ConfigurePlotChrome()
    {
        if (_avaPlot is null) return;
        var p = _avaPlot.Plot;

        // Pin the font to DejaVu Sans (see Issue #122 commit log —
        // ScottPlot's auto-font-detect fails inside an Avalonia
        // process and silently renders text with zero glyphs).
        p.Font.Set("DejaVu Sans");

        var fg = Color.FromHex("#d7d7d7");

        p.Add.Palette = new ScottPlot.Palettes.Penumbra();
        p.FigureBackground.Color = Color.FromHex("#181818");
        p.DataBackground.Color = Color.FromHex("#1f1f1f");
        p.Axes.Color(fg);
        p.Grid.MajorLineColor = Color.FromHex("#404040");
        p.Legend.BackgroundColor = Color.FromHex("#404040");
        p.Legend.FontColor = fg;
        p.Legend.OutlineColor = fg;

        // Larger fonts for hi-DPI laptop displays.
        p.Axes.Bottom.TickLabelStyle.FontSize = 13;
        p.Axes.Bottom.Label.FontSize = 14;
        p.Axes.Bottom.Label.Text = "time (s)";
        p.Legend.FontSize = 12;

        // Apply font + colour to every left-stacked Y axis we
        // created.  Axes.Color() above only repaints the standard
        // ones; the additional AddLeftAxis() instances need direct
        // assignment so their tick labels stay visible too.
        // Per-axis textual labels are intentionally left empty —
        // four "Y1..Y4" labels next to the tick numbers wasted
        // horizontal space without adding information.
        foreach (var ax in _yAxes)
        {
            ax.FrameLineStyle.Color = fg;
            ax.MajorTickStyle.Color = fg;
            ax.MinorTickStyle.Color = fg;
            ax.TickLabelStyle.ForeColor = fg;
            ax.Label.ForeColor = fg;
            ax.TickLabelStyle.FontSize = 13;
            ax.Label.FontSize = 14;
            ax.Label.Text = string.Empty;
        }
    }

    private void RedrawPlot()
    {
        if (_avaPlot is null) return;
        if (DataContext is not MainViewModel vm) return;

        var plot = _avaPlot.Plot;
        plot.Clear();
        ConfigurePlotChrome();

        ulong earliestStartTsUs = ulong.MaxValue;
        if (vm.TimeAxis == TimeAxisMode.Sequence)
        {
            foreach (var row in vm.Loaded)
            {
                if (row.IsVisible &&
                    row.Capture.StartTimestampUs < earliestStartTsUs)
                {
                    earliestStartTsUs = row.Capture.StartTimestampUs;
                }
            }
            if (earliestStartTsUs == ulong.MaxValue) earliestStartTsUs = 0;
        }

        var anyPlotted = false;
        var axisUsed = new bool[_yAxes.Length];
        var visibleIndex = 0;
        foreach (var row in vm.Loaded)
        {
            if (!row.IsVisible) continue;

            FieldDescriptor? tsField = null;
            foreach (var f in row.Capture.Fields)
            {
                if (f.Name == "ts_us") { tsField = f; break; }
            }
            if (tsField is null) continue;

            var offsetUs = vm.TimeAxis == TimeAxisMode.Sequence
                ? (double)(row.Capture.StartTimestampUs - earliestStartTsUs)
                : 0.0;

            var fieldIndex = 0;
            foreach (var fv in row.Fields)
            {
                fieldIndex++;
                if (!fv.IsChecked) continue;

                FieldDescriptor? targetField = null;
                foreach (var f in row.Capture.Fields)
                {
                    if (f.Name == fv.Name) { targetField = f; break; }
                }
                if (targetField is null) continue;

                var n = row.Capture.RecordCount;
                var xs = new double[n];
                var ys = new double[n];
                for (var i = 0; i < n; i++)
                {
                    var rec = row.Capture.Records(i).Span;
                    xs[i] = (offsetUs + FieldReader.ReadRawDouble(rec, tsField!)) / 1_000_000.0;
                    ys[i] = FieldReader.ReadScaledDouble(rec, targetField!);
                }

                var markerShape = MarkerPalette[(visibleIndex + fieldIndex) % MarkerPalette.Length];
                var markerSize = n switch
                {
                    <= 50 => 6f,
                    <= 200 => 4f,
                    _ => 3f,
                };

                var scatter = plot.Add.Scatter(xs, ys);
                scatter.LegendText = $"{row.Label} · {fv.Name}";
                scatter.LineColor = ArgbToScottColor(row.ColorArgb);
                scatter.LineWidth = 1.5f;
                scatter.MarkerShape = markerShape;
                scatter.MarkerColor = scatter.LineColor;
                scatter.MarkerSize = markerSize;

                // Bind to one of the four left-stacked Y axes per
                // the user's per-field selector (Axis is 1-based in
                // the UI, 0-based in our array).
                var ai = Math.Clamp(fv.Axis - 1, 0, _yAxes.Length - 1);
                scatter.Axes.YAxis = _yAxes[ai];
                axisUsed[ai] = true;
                anyPlotted = true;
            }
            visibleIndex++;
        }

        // Hide Y axes nobody is plotting onto, so the empty Y3 / Y4
        // ticks don't crowd the left margin.  Axis 1 stays visible
        // even when unused so the plot still shows a left frame.
        for (var i = 0; i < _yAxes.Length; i++)
        {
            _yAxes[i].IsVisible = (i == 0) || axisUsed[i];
        }

        plot.Axes.Title.Label.Text =
            anyPlotted ? "Capture overlay" : "(no visible field)";
        plot.Axes.Bottom.Label.Text = "time (s)";

        plot.Axes.AutoScale();

        // Apply user-supplied per-axis Y range overrides on top of
        // AutoScale — empty / unparseable boxes leave that axis at
        // autoscale.
        ApplyYRangeOverride(_yAxes[0], vm.Axis1Min, vm.Axis1Max);
        ApplyYRangeOverride(_yAxes[1], vm.Axis2Min, vm.Axis2Max);
        ApplyYRangeOverride(_yAxes[2], vm.Axis3Min, vm.Axis3Max);
        ApplyYRangeOverride(_yAxes[3], vm.Axis4Min, vm.Axis4Max);

        if (anyPlotted) plot.ShowLegend();
        _avaPlot.Refresh();
    }

    private static void ApplyYRangeOverride(IYAxis axis, string min, string max)
    {
        var hasMin = double.TryParse(min, System.Globalization.CultureInfo.InvariantCulture, out var dmin);
        var hasMax = double.TryParse(max, System.Globalization.CultureInfo.InvariantCulture, out var dmax);
        if (hasMin && hasMax && dmax > dmin)
        {
            axis.Range.Set(dmin, dmax);
        }
    }

    private static Color ArgbToScottColor(uint argb) =>
        new Color(
            red:   (byte)((argb >> 16) & 0xFF),
            green: (byte)((argb >>  8) & 0xFF),
            blue:  (byte)( argb        & 0xFF),
            alpha: (byte)((argb >> 24) & 0xFF));
}
