using System.Collections.Immutable;
using System.Globalization;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuViewer.App.Services;
using ImuViewer.Core.Btsensor;
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Wire;

namespace ImuViewer.App.ViewModels;

/// <summary>
/// Single-class write panel for the left sidebar.  The user picks
/// which class to control via <see cref="SelectedClass"/>; the rest
/// of the panel (mode list, PWM channel count, etc.) re-binds
/// accordingly.
/// </summary>
public sealed partial class SensorWriteViewModel : ObservableObject
{
    public sealed record ClassOption(LegoClassId Id, string Display)
    {
        public override string ToString() => Display;
    }

    public sealed record ModeOption(byte ModeId, string Display)
    {
        public override string ToString() => Display;
    }

    private readonly SessionOrchestrator _orchestrator;

    public SensorWriteViewModel(SessionOrchestrator orchestrator)
    {
        _orchestrator = orchestrator;
        AvailableClasses = Enum.GetValues<LegoClassId>()
            .Select(id => new ClassOption(id, ScaleTables.ClassName(id)))
            .ToImmutableArray();
        SelectedClass = AvailableClasses.FirstOrDefault();
        // OnSelectedClassChanged populates Modes / PWM via partial method.
    }

    public ImmutableArray<ClassOption> AvailableClasses { get; }

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(WriteHeader))]
    private ClassOption? _selectedClass;

    [ObservableProperty]
    private ImmutableArray<ModeOption> _availableModes = ImmutableArray<ModeOption>.Empty;

    [ObservableProperty]
    private ModeOption? _selectedMode;

    [ObservableProperty]
    private byte _sendModeId;

    [ObservableProperty]
    private string _sendHexInput = string.Empty;

    [ObservableProperty]
    private bool _pwmIsSupported;

    [ObservableProperty]
    private int _pwmChannelCount;

    [ObservableProperty] private bool _pwmCh0Visible;
    [ObservableProperty] private bool _pwmCh1Visible;
    [ObservableProperty] private bool _pwmCh2Visible;
    [ObservableProperty] private bool _pwmCh3Visible;

    [ObservableProperty] private int _pwmCh0;
    [ObservableProperty] private int _pwmCh1;
    [ObservableProperty] private int _pwmCh2;
    [ObservableProperty] private int _pwmCh3;

    [ObservableProperty]
    private bool _writeEnabled;

    [ObservableProperty]
    private string _statusText = string.Empty;

    public string WriteHeader => SelectedClass is null
        ? "Sensor write"
        : $"Sensor write — {SelectedClass.Display}";

    /// <summary>
    /// Called by MainViewModel when connect/SENSOR_ON state flips.
    /// </summary>
    public void SetWriteEnabled(bool enabled)
    {
        WriteEnabled = enabled;
    }

    partial void OnSelectedClassChanged(ClassOption? value)
    {
        if (value is null)
        {
            AvailableModes = ImmutableArray<ModeOption>.Empty;
            SelectedMode = null;
            PwmIsSupported = false;
            PwmChannelCount = 0;
            PwmCh0Visible = PwmCh1Visible = PwmCh2Visible = PwmCh3Visible = false;
            return;
        }

        AvailableModes = BuildModes(value.Id);
        SelectedMode = AvailableModes.FirstOrDefault();
        PwmChannelCount = PwmCountFor(value.Id);
        PwmIsSupported = PwmChannelCount > 0;
        PwmCh0Visible = PwmChannelCount >= 1;
        PwmCh1Visible = PwmChannelCount >= 2;
        PwmCh2Visible = PwmChannelCount >= 3;
        PwmCh3Visible = PwmChannelCount >= 4;
    }

    [RelayCommand]
    public async Task SetModeAsync()
    {
        if (SelectedClass is null || SelectedMode is null) return;
        try
        {
            BtsensorReply r = await _orchestrator.SetSensorModeAsync(
                SelectedClass.Id, SelectedMode.ModeId, CancellationToken.None);
            StatusText = r.IsOk ? $"MODE → {SelectedMode.Display}" : "MODE: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "MODE: " + ex.Message;
        }
    }

    [RelayCommand]
    public async Task SendDataAsync()
    {
        if (SelectedClass is null) return;
        byte[]? bytes = TryParseHex(SendHexInput);
        if (bytes is null || bytes.Length == 0)
        {
            StatusText = "SEND: invalid hex";
            return;
        }
        try
        {
            BtsensorReply r = await _orchestrator.SensorSendAsync(
                SelectedClass.Id, SendModeId, bytes, CancellationToken.None);
            StatusText = r.IsOk
                ? $"SEND mode={SendModeId} ({bytes.Length}B)"
                : "SEND: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "SEND: " + ex.Message;
        }
    }

    [RelayCommand]
    public async Task ApplyPwmAsync()
    {
        if (SelectedClass is null || PwmChannelCount == 0) return;
        int[] vals = PwmChannelCount switch
        {
            1 => new[] { PwmCh0 },
            2 => new[] { PwmCh0, PwmCh1 },
            3 => new[] { PwmCh0, PwmCh1, PwmCh2 },
            4 => new[] { PwmCh0, PwmCh1, PwmCh2, PwmCh3 },
            _ => Array.Empty<int>(),
        };
        if (vals.Length == 0) return;
        try
        {
            BtsensorReply r = await _orchestrator.SensorPwmAsync(
                SelectedClass.Id, vals, CancellationToken.None);
            StatusText = r.IsOk
                ? $"PWM → [{string.Join(',', vals)}]"
                : "PWM: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "PWM: " + ex.Message;
        }
    }

    private static ImmutableArray<ModeOption> BuildModes(LegoClassId classId)
    {
        if (!ScaleTables.ByClass.TryGetValue(classId, out ScaleTables.ClassSpec? cls))
        {
            return ImmutableArray<ModeOption>.Empty;
        }

        return cls.ModesByModeId
            .OrderBy(kv => kv.Key)
            .Select(kv => new ModeOption(kv.Key, $"{kv.Key}: {kv.Value.Label}"))
            .ToImmutableArray();
    }

    private static int PwmCountFor(LegoClassId classId) => classId switch
    {
        LegoClassId.Color      => 3,
        LegoClassId.Ultrasonic => 4,
        LegoClassId.MotorM     => 1,
        LegoClassId.MotorR     => 1,
        LegoClassId.MotorL     => 1,
        _ => 0,
    };

    private static byte[]? TryParseHex(string s)
    {
        if (string.IsNullOrWhiteSpace(s)) return null;
        string trimmed = new(s.Where(c => !char.IsWhiteSpace(c)).ToArray());
        if ((trimmed.Length & 1) != 0) return null;
        byte[] bytes = new byte[trimmed.Length / 2];
        for (int i = 0; i < bytes.Length; i++)
        {
            if (!byte.TryParse(trimmed.AsSpan(i * 2, 2),
                NumberStyles.HexNumber, CultureInfo.InvariantCulture, out byte b))
            {
                return null;
            }
            bytes[i] = b;
        }
        return bytes;
    }
}
