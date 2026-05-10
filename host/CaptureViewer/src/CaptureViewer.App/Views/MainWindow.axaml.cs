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

        // Pin the font to DejaVu Sans.  ScottPlot's auto-font-detect
        // (https://scottplot.net/faq/dependencies/) silently fails
        // inside an Avalonia process — Avalonia configures SkiaSharp
        // with a font resolver that doesn't expose the system fonts
        // ScottPlot expects, so tick labels / axis labels render as
        // zero-glyph blanks even though the line + grid still draw.
        // DejaVu Sans is shipped with virtually every desktop Linux.
        p.Font.Set("DejaVu Sans");

        // Dark theme — recipe per https://scottplot.net/cookbook/5/Styling/
        p.Add.Palette = new ScottPlot.Palettes.Penumbra();
        p.FigureBackground.Color = Color.FromHex("#181818");
        p.DataBackground.Color = Color.FromHex("#1f1f1f");
        p.Axes.Color(Color.FromHex("#d7d7d7"));
        p.Grid.MajorLineColor = Color.FromHex("#404040");
        p.Legend.BackgroundColor = Color.FromHex("#404040");
        p.Legend.FontColor = Color.FromHex("#d7d7d7");
        p.Legend.OutlineColor = Color.FromHex("#d7d7d7");

        // Larger fonts for hi-DPI laptop displays.
        p.Axes.Bottom.TickLabelStyle.FontSize = 13;
        p.Axes.Left.TickLabelStyle.FontSize = 13;
        p.Axes.Bottom.Label.FontSize = 14;
        p.Axes.Left.Label.FontSize = 14;
        p.Legend.FontSize = 12;

        p.Axes.Bottom.Label.Text = "time (s)";
        p.Axes.Left.Label.Text = "value";
    }

    private void RedrawPlot()
    {
        if (_avaPlot is null) return;
        if (DataContext is not MainViewModel vm) return;

        var plot = _avaPlot.Plot;
        plot.Clear();
        ConfigurePlotChrome();

        var field = vm.SelectedFieldName;
        if (string.IsNullOrEmpty(field))
        {
            plot.Axes.Title.Label.Text = "(select a field)";
            plot.Axes.Left.Label.Text = "value";
            _avaPlot.Refresh();
            return;
        }

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
        if (anyPlotted) plot.ShowLegend();

        _avaPlot.Refresh();
    }

    private static Color ArgbToScottColor(uint argb) =>
        new Color(
            red:   (byte)((argb >> 16) & 0xFF),
            green: (byte)((argb >>  8) & 0xFF),
            blue:  (byte)( argb        & 0xFF),
            alpha: (byte)((argb >> 24) & 0xFF));
}
