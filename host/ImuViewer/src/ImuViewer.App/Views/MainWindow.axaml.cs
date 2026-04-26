using Avalonia.Controls;
using Avalonia.Threading;
using ImuViewer.App.Services;
using ImuViewer.App.ViewModels;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Filters;
using ImuViewer.Core.Transport.Linux;

namespace ImuViewer.App.Views;

public partial class MainWindow : Window
{
    private readonly DispatcherTimer _tick;
    private readonly MainViewModel _viewModel;

    public MainWindow()
    {
        InitializeComponent();

        SensorAggregator aggregator = new();
        MadgwickFilter filter = new();
        LinuxRfcommTransport transport = new();
        SessionOrchestrator orchestrator = new(transport, aggregator);
        LinuxPortEnumerator enumerator = new();
        _viewModel = new MainViewModel(enumerator, orchestrator, aggregator, filter);
        DataContext = _viewModel;

        _tick = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1000.0 / 60.0) };
        _tick.Tick += (_, _) => _viewModel.Tick();
        _tick.Start();

        Closing += async (_, _) =>
        {
            _tick.Stop();
            await _viewModel.DisposeAsync();
        };
    }
}
