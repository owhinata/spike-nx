using System.Collections.Specialized;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Threading;
using ImuViewer.Core.LegoSensor;

namespace ImuViewer.App.Controls;

/// <summary>
/// Lightweight strip-chart for one sensor panel.  Renders one polyline
/// per channel onto a <see cref="Canvas"/>; auto-scales the Y axis to
/// observed min/max with a small margin.  No external dependency
/// (avoids OxyPlot.Avalonia / LiveCharts NuGets) — this is enough for
/// the ≤600-point × ≤4-channel × 60 Hz redraw load Issue B targets.
/// </summary>
public sealed class SensorPlotControl : Control
{
    public static readonly StyledProperty<IReadOnlyList<LegoSamplePoint>?>
        PointsProperty =
            AvaloniaProperty.Register<SensorPlotControl, IReadOnlyList<LegoSamplePoint>?>(
                nameof(Points));

    /// <summary>Bound to the panel's <c>RecentPoints</c> ObservableCollection.</summary>
    public IReadOnlyList<LegoSamplePoint>? Points
    {
        get => GetValue(PointsProperty);
        set => SetValue(PointsProperty, value);
    }

    private static readonly IBrush[] s_palette =
    {
        new SolidColorBrush(Color.FromRgb(0xFF, 0x70, 0x70)),
        new SolidColorBrush(Color.FromRgb(0x70, 0xFF, 0x70)),
        new SolidColorBrush(Color.FromRgb(0x70, 0xA0, 0xFF)),
        new SolidColorBrush(Color.FromRgb(0xFF, 0xC0, 0x70)),
    };

    private static readonly IBrush s_axisBrush =
        new SolidColorBrush(Color.FromArgb(0x80, 0x60, 0x60, 0x60));

    private static readonly IBrush s_background =
        new SolidColorBrush(Color.FromRgb(0x18, 0x18, 0x1C));

    private NotifyCollectionChangedEventHandler? _collectionHandler;
    private object? _hookedCollection;

    public SensorPlotControl()
    {
        ClipToBounds = true;
        MinHeight = 80;
    }

    protected override Size MeasureOverride(Size availableSize)
    {
        return new Size(
            double.IsInfinity(availableSize.Width) ? 200 : availableSize.Width,
            double.IsInfinity(availableSize.Height) ? 100 : availableSize.Height);
    }

    protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
    {
        base.OnPropertyChanged(change);
        if (change.Property == PointsProperty)
        {
            HookCollection(change.NewValue);
            InvalidateVisual();
        }
    }

    private void HookCollection(object? newCollection)
    {
        if (_hookedCollection is INotifyCollectionChanged old && _collectionHandler is not null)
        {
            old.CollectionChanged -= _collectionHandler;
        }
        _hookedCollection = newCollection;
        if (newCollection is INotifyCollectionChanged ncc)
        {
            _collectionHandler = (_, _) =>
                Dispatcher.UIThread.Post(InvalidateVisual);
            ncc.CollectionChanged += _collectionHandler;
        }
        else
        {
            _collectionHandler = null;
        }
    }

    public override void Render(DrawingContext ctx)
    {
        Rect rect = new(Bounds.Size);
        ctx.FillRectangle(s_background, rect);

        IReadOnlyList<LegoSamplePoint>? pts = Points;
        if (pts is null || pts.Count == 0)
        {
            return;
        }

        int channelCount = pts.Max(p => p.Values.Length);
        if (channelCount == 0)
        {
            return;
        }

        // Auto-scale Y across all channels and points.
        float min = float.MaxValue;
        float max = float.MinValue;
        foreach (LegoSamplePoint pt in pts)
        {
            for (int c = 0; c < pt.Values.Length; c++)
            {
                float v = pt.Values[c];
                if (v < min) min = v;
                if (v > max) max = v;
            }
        }
        if (!float.IsFinite(min) || !float.IsFinite(max) || min == max)
        {
            min -= 1;
            max += 1;
        }

        float margin = (max - min) * 0.08f;
        min -= margin;
        max += margin;

        const double padX = 4;
        const double padY = 4;
        double w = Math.Max(1, rect.Width - 2 * padX);
        double h = Math.Max(1, rect.Height - 2 * padY);
        int n = pts.Count;
        double dx = n > 1 ? w / (n - 1) : 0;

        // Zero line if span includes zero.
        if (min < 0 && max > 0)
        {
            double yZero = padY + (max - 0.0) / (max - min) * h;
            ctx.DrawLine(new Pen(s_axisBrush, 0.5),
                new Point(padX, yZero),
                new Point(padX + w, yZero));
        }

        for (int c = 0; c < channelCount; c++)
        {
            IPen pen = new Pen(s_palette[c % s_palette.Length], 1.2);
            Point? prev = null;
            for (int i = 0; i < n; i++)
            {
                if (pts[i].Values.Length <= c) continue;
                float v = pts[i].Values[c];
                double x = padX + i * dx;
                double y = padY + (max - v) / (max - min) * h;
                Point cur = new(x, y);
                if (prev is Point p)
                {
                    ctx.DrawLine(pen, p, cur);
                }
                prev = cur;
            }
        }
    }
}
