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
    private AvaPlot? _avaPlot;
    private ScrollViewer? _logScroller;

    public MainWindow()
    {
        InitializeComponent();
        _avaPlot = this.FindControl<AvaPlot>("AvaPlot1");
        _logScroller = this.FindControl<ScrollViewer>("LogScroller");
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
            // Hook each row added to Loaded so checkbox visibility
            // toggles invalidate the plot too.
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
            // Auto-scroll the log to the bottom whenever a new line
            // arrives.  Posting through the dispatcher ensures the
            // ScrollViewer has measured the new item before we ask it
            // to scroll.
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
            ? DragDropEffects.Copy
            : DragDropEffects.None;
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
            // Filter to .cap so the user does not accidentally drop a
            // random binary and confuse the parser; non-.cap files are
            // silently skipped.
            if (!path.EndsWith(".cap", StringComparison.OrdinalIgnoreCase))
                continue;
            await vm.LoadCaptureFromPathAsync(path);
        }
    }

    private void ConfigurePlotChrome()
    {
        if (_avaPlot is null) return;
        var p = _avaPlot.Plot;

        // Dark background + light foreground so the plot blends with
        // the rest of the dark theme.  Tick label color is the one
        // ScottPlot 5 takes a few extra calls to set — the
        // `Axes.Color()` shortcut covers tick frames + tick labels +
        // axis labels in one go on this version.
        p.FigureBackground.Color = Colors.Black;
        p.DataBackground.Color = new Color(20, 20, 20);
        p.Axes.Color(new Color(200, 200, 200));
        p.Grid.MajorLineColor = new Color(60, 60, 60);
        p.Grid.MinorLineColor = new Color(40, 40, 40);
        p.Axes.Title.Label.ForeColor = new Color(220, 220, 220);
        p.Axes.Bottom.Label.ForeColor = new Color(220, 220, 220);
        p.Axes.Left.Label.ForeColor = new Color(220, 220, 220);
        p.Axes.Bottom.Label.Text = "time (s)";
        p.Axes.Left.Label.Text = "value";
    }

    private void RedrawPlot()
    {
        if (_avaPlot is null) return;
        if (DataContext is not MainViewModel vm) return;

        var plot = _avaPlot.Plot;
        plot.Clear();

        var field = vm.SelectedFieldName;
        if (string.IsNullOrEmpty(field))
        {
            plot.Axes.Title.Label.Text = "(select a field)";
            plot.Axes.Left.Label.Text = "value";
            _avaPlot.Refresh();
            return;
        }

        // For sequence-mode timeline alignment, anchor at the earliest
        // visible capture's start_ts_us so all rows share a common t=0
        // and later runs shift right by their wall-clock delta.
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
        string? sampleUnit = null;
        foreach (var row in vm.Loaded)
        {
            if (!row.IsVisible) continue;

            FieldDescriptor? tsField = null;
            FieldDescriptor? targetField = null;
            foreach (var f in row.Capture.Fields)
            {
                if (f.Name == "ts_us") tsField = f;
                if (f.Name == field) targetField = f;
            }
            if (tsField is null || targetField is null) continue;
            sampleUnit ??= targetField.Unit;

            var n = row.Capture.RecordCount;
            var xs = new double[n];
            var ys = new double[n];

            // Sequence-mode shift: this capture's records all advance
            // by (start_ts_us - earliestStartTsUs) so back-to-back runs
            // line up on a common timeline.
            var offsetUs = vm.TimeAxis == TimeAxisMode.Sequence
                ? (double)(row.Capture.StartTimestampUs - earliestStartTsUs)
                : 0.0;

            for (var i = 0; i < n; i++)
            {
                var rec = row.Capture.Records(i).Span;
                xs[i] = (offsetUs + FieldReader.ReadRawDouble(rec, tsField!)) / 1_000_000.0;
                ys[i] = FieldReader.ReadScaledDouble(rec, targetField!);
            }

            var scatter = plot.Add.Scatter(xs, ys);
            scatter.LegendText = row.Label;
            scatter.LineColor = ArgbToScottColor(row.ColorArgb);
            scatter.MarkerColor = scatter.LineColor;
            scatter.MarkerSize = 3;
            anyPlotted = true;
        }

        plot.Axes.Title.Label.Text = anyPlotted ? field : $"(no visible capture has `{field}`)";
        plot.Axes.Left.Label.Text = string.IsNullOrEmpty(sampleUnit) ? "value" : sampleUnit;
        plot.Axes.AutoScale();
        if (anyPlotted)
        {
            plot.ShowLegend();
        }
        _avaPlot.Refresh();
    }

    private static Color ArgbToScottColor(uint argb) =>
        new Color(
            red:   (byte)((argb >> 16) & 0xFF),
            green: (byte)((argb >>  8) & 0xFF),
            blue:  (byte)( argb        & 0xFF),
            alpha: (byte)((argb >> 24) & 0xFF));
}
