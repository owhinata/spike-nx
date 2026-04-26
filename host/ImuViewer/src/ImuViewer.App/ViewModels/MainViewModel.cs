using System.Collections.ObjectModel;
using System.Numerics;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuViewer.Core.Transport;

namespace ImuViewer.App.ViewModels;

public partial class MainViewModel : ObservableObject
{
    [ObservableProperty]
    private ObservableCollection<BluetoothPort> _ports = new();

    [ObservableProperty]
    private BluetoothPort? _selectedPort;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanConnect))]
    [NotifyPropertyChangedFor(nameof(CanDisconnect))]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    private bool _isConnected;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(CanToggleImu))]
    private bool _isImuOn;

    [ObservableProperty]
    private string _statusText = "ready";

    [ObservableProperty]
    private Quaternion _orientation = Quaternion.Identity;

    [ObservableProperty]
    private Vector3 _accelG;

    [ObservableProperty]
    private Vector3 _gyroDps;

    [ObservableProperty]
    private double _measuredFps;

    [ObservableProperty]
    private int _odrHz = 833;

    [ObservableProperty]
    private int _batch = 13;

    [ObservableProperty]
    private int _accelFsrG = 8;

    [ObservableProperty]
    private int _gyroFsrDps = 2000;

    [ObservableProperty]
    private float _madgwickBeta = 0.1f;

    public bool CanConnect => !IsConnected && SelectedPort is not null;
    public bool CanDisconnect => IsConnected;
    public bool CanToggleImu => IsConnected;

    partial void OnSelectedPortChanged(BluetoothPort? value)
    {
        OnPropertyChanged(nameof(CanConnect));
    }

    [RelayCommand]
    public virtual Task RefreshPortsAsync(CancellationToken ct) => Task.CompletedTask;

    [RelayCommand]
    public virtual Task ConnectAsync(CancellationToken ct) => Task.CompletedTask;

    [RelayCommand]
    public virtual Task DisconnectAsync(CancellationToken ct) => Task.CompletedTask;

    [RelayCommand]
    public virtual Task ImuOnAsync(CancellationToken ct) => Task.CompletedTask;

    [RelayCommand]
    public virtual Task ImuOffAsync(CancellationToken ct) => Task.CompletedTask;

    [RelayCommand]
    public virtual void ResetOrientation()
    {
        Orientation = Quaternion.Identity;
    }
}
