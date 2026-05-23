using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using ImuViewer.App.Services;
using ImuViewer.App.ViewModels;
using ImuViewer.Core.Aggregation;
using ImuViewer.Core.Filters;
using ImuViewer.Core.LegoSensor;
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
        LegoSampleAggregator legoAggregator = new();
        MadgwickFilter filter = new();
        GyroBiasTracker biasTracker = new();
        LinuxRfcommTransport transport = new();
        SessionOrchestrator orchestrator = new(transport, aggregator, legoAggregator);
        LinuxPortEnumerator enumerator = new();
        _viewModel = new MainViewModel(enumerator, orchestrator, aggregator,
                                        legoAggregator, filter, biasTracker);
        DataContext = _viewModel;

        _tick = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1000.0 / 60.0) };
        _tick.Tick += (_, _) => _viewModel.Tick();
        _tick.Start();

        // Sidebar height enforcement.  Avalonia 11 was letting the
        // sidebar ScrollViewer's allocated height exceed the window
        // client area, so the vertical scroll range fell short of
        // the last items in the StackPanel and Telemetry's "status"
        // row sat below the visible window.  Pin the wrapper border
        // height to the window's actual bounds on every layout pass.
        SizeChanged += (_, _) => UpdateSidebarHeight();
        // Initial sync — Bounds is available after the window opens.
        Opened += (_, _) => UpdateSidebarHeight();

        Closing += async (_, _) =>
        {
            _tick.Stop();
            await _viewModel.DisposeAsync();
        };
    }

    private void UpdateSidebarHeight()
    {
        Border? border = this.FindControl<Border>("SidebarBorder");
        if (border is not null)
        {
            border.Height = Bounds.Height;
        }
    }

    /// <summary>
    /// Phase 2.5 (#145): Browse button for the IMU Capture output
    /// file.  Lives in code-behind because Avalonia's StorageProvider
    /// is a top-level/Window resource, and the ViewModel intentionally
    /// has no Window reference so it stays unit-testable.  The View
    /// just hands the picked path back via the bound property.
    /// </summary>
    private async void OnBrowseOutputClicked(object? sender, RoutedEventArgs e)
    {
        if (DataContext is not MainViewModel vm)
        {
            return;
        }

        // Start the picker at the directory of the currently-set
        // OutputPath (so re-Browse stays in the user's chosen folder)
        // or fall back to the process working directory — matches
        // the default OutputPath the ViewModel computes at startup.
        string startDir = System.IO.Path.GetDirectoryName(vm.Capture.OutputPath)
                          is { Length: > 0 } d
            ? d
            : System.IO.Directory.GetCurrentDirectory();
        IStorageFolder? startFolder = null;
        try
        {
            startFolder = await StorageProvider.TryGetFolderFromPathAsync(startDir);
        }
        catch
        {
            // Provider may reject paths it can't resolve (e.g. running
            // under a sandboxed picker); leave startFolder null and
            // let the dialog pick its own default.
        }

        IStorageFile? file = await StorageProvider.SaveFilePickerAsync(
            new FilePickerSaveOptions
            {
                Title = "Save IMU_CAP recording",
                SuggestedFileName = System.IO.Path.GetFileName(vm.Capture.OutputPath),
                SuggestedStartLocation = startFolder,
                DefaultExtension = "bin",
                FileTypeChoices = new[]
                {
                    new FilePickerFileType("IMU_CAP recording (*.bin)")
                    {
                        Patterns = new[] { "*.bin" },
                    },
                    FilePickerFileTypes.All,
                },
            });

        if (file is null)
        {
            return;
        }

        // Prefer the local path so the ViewModel keeps working with
        // System.IO directly; fall back to the URI when the file
        // sits on a non-local provider (rare on the Linux PoC).
        string? localPath = file.TryGetLocalPath();
        vm.Capture.OutputPath = localPath ?? file.Path.ToString();
    }
}
