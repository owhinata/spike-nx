using Avalonia.Controls;
using Avalonia.Markup.Xaml;

using CaptureViewer.App.ViewModels;
using CaptureViewer.Core;
using CaptureViewer.Core.Capture;

using ScottPlot;
using ScottPlot.Avalonia;

namespace CaptureViewer.App.Views;

public partial class MainWindow : Window
{
    private AvaPlot? _avaPlot;

    public MainWindow()
    {
        InitializeComponent();
        _avaPlot = this.FindControl<AvaPlot>("AvaPlot1");
        DataContextChanged += OnDataContextChanged;
        ConfigurePlotChrome();
    }

    private void InitializeComponent() => AvaloniaXamlLoader.Load(this);

    private void OnDataContextChanged(object? sender, EventArgs e)
    {
        if (DataContext is MainViewModel vm)
        {
            vm.PlotInvalidated += RedrawPlot;
            vm.Loaded.CollectionChanged += (_, _) => RedrawPlot();
            // Per-row visibility flips through PlotInvalidated already
            // (the row VM raises PropertyChanged; the Main VM does not
            // hook each row, so we tell the row to nudge us via
            // CheckBox-bound IsVisible -> our redraw-on-toggle
            // handler).  Hook each row when added so checkbox toggles
            // also invalidate the plot.
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
            RedrawPlot();
        }
    }

    private void ConfigurePlotChrome()
    {
        if (_avaPlot is null) return;
        _avaPlot.Plot.FigureBackground.Color = Colors.Black;
        _avaPlot.Plot.DataBackground.Color = new Color(20, 20, 20);
        _avaPlot.Plot.Axes.Color(new Color(160, 160, 160));
        _avaPlot.Plot.Grid.MajorLineColor = new Color(60, 60, 60);
        _avaPlot.Plot.Axes.Title.Label.ForeColor = new Color(220, 220, 220);
        _avaPlot.Plot.Axes.Bottom.Label.ForeColor = new Color(220, 220, 220);
        _avaPlot.Plot.Axes.Left.Label.ForeColor = new Color(220, 220, 220);
        _avaPlot.Plot.Axes.Bottom.Label.Text = "time (s)";
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
            _avaPlot.Refresh();
            return;
        }

        var anyPlotted = false;
        foreach (var row in vm.Loaded)
        {
            if (!row.IsVisible) continue;

            // Skip captures whose schema does not have this field.
            var hasField = false;
            foreach (var f in row.Capture.Fields)
            {
                if (f.Name == field) { hasField = true; break; }
            }
            if (!hasField) continue;

            // Build the (t_seconds, value) arrays.  ts_us is the
            // first field; convert to seconds for human-friendly axis.
            var n = row.Capture.RecordCount;
            var xs = new double[n];
            var ys = new double[n];
            FieldDescriptor? tsField = null;
            FieldDescriptor? targetField = null;
            foreach (var f in row.Capture.Fields)
            {
                if (f.Name == "ts_us") tsField = f;
                if (f.Name == field) targetField = f;
            }
            if (tsField is null || targetField is null) continue;

            for (var i = 0; i < n; i++)
            {
                var rec = row.Capture.Records(i).Span;
                xs[i] = FieldReader.ReadRawDouble(rec, tsField!) / 1_000_000.0;
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
