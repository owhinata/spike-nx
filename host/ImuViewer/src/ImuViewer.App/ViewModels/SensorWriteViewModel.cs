using System.Collections.Immutable;
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
/// of the panel (mode list, slider channel count / range / target
/// ioctl) re-binds accordingly.
///
/// Per-class slider routing (Issue #92):
/// <list type="bullet">
///   <item>COLOR — sliders write to LIGHT (mode 3) via SENSOR SEND
///         (0..100 INT8 PCT per channel).  SET_PWM is not used:
///         the kernel ioctl returns -ENOTSUP for COLOR because
///         "PWM" isn't the right semantic — see sensor.md §4.5</item>
///   <item>ULTRASONIC — eye-LED LIGHT mode via SET_PWM (current
///         backend, may also migrate to SEND later)</item>
///   <item>MOTOR_* — H-bridge duty (-100..+100) via SET_PWM
///         (returns -ENOTSUP until Issue #80 lands)</item>
///   <item>FORCE — no actuator, slider hidden</item>
/// </list>
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
        // OnSelectedClassChanged populates Modes / sliders via partial method.
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
    private bool _writeSliderVisible;

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

    [ObservableProperty] private int _writeSliderMin = -100;
    [ObservableProperty] private int _writeSliderMax = 100;

    [ObservableProperty]
    private string _writeSliderHeader = "Write";

    [ObservableProperty]
    private string _writeSliderApplyLabel = "Apply";

    [ObservableProperty]
    private bool _brakeIsSupported;

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
            WriteSliderVisible = false;
            PwmChannelCount = 0;
            PwmCh0Visible = PwmCh1Visible = PwmCh2Visible = PwmCh3Visible = false;
            BrakeIsSupported = false;
            return;
        }

        AvailableModes = BuildModes(value.Id);
        SelectedMode = AvailableModes.FirstOrDefault();
        PwmChannelCount = WriteChannelCount(value.Id);
        WriteSliderVisible = PwmChannelCount > 0;
        PwmCh0Visible = PwmChannelCount >= 1;
        PwmCh1Visible = PwmChannelCount >= 2;
        PwmCh2Visible = PwmChannelCount >= 3;
        PwmCh3Visible = PwmChannelCount >= 4;
        BrakeIsSupported = IsMotor(value.Id);

        // Class-specific slider range / labelling.
        switch (value.Id)
        {
            case LegoClassId.Color:
                WriteSliderMin = 0;
                WriteSliderMax = 100;
                WriteSliderHeader = "LIGHT  (mode 3, 0..100 % per LED)";
                WriteSliderApplyLabel = "Apply LIGHT (SEND mode 3)";
                break;
            case LegoClassId.Ultrasonic:
                WriteSliderMin = 0;
                WriteSliderMax = 100;
                WriteSliderHeader = "Eye LEDs  (mode 5, 0..100 % per LED)";
                WriteSliderApplyLabel = "Apply LIGHT (SEND mode 5)";
                break;
            case LegoClassId.MotorM:
            case LegoClassId.MotorR:
            case LegoClassId.MotorL:
                WriteSliderMin = -100;
                WriteSliderMax = 100;
                WriteSliderHeader = "Motor PWM  (-100..+100 %, signed duty)";
                WriteSliderApplyLabel = "Apply PWM";
                break;
            default:
                WriteSliderMin = -100;
                WriteSliderMax = 100;
                WriteSliderHeader = "Write";
                WriteSliderApplyLabel = "Apply";
                break;
        }
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

        byte? sendMode = LightSendModeFor(SelectedClass.Id);
        bool useSend = sendMode.HasValue;
        string verb  = useSend ? "SEND" : "PWM";

        try
        {
            BtsensorReply r = useSend
                ? await ApplyLightSendAsync(SelectedClass.Id, sendMode!.Value, vals)
                : await _orchestrator.SensorPwmAsync(
                    SelectedClass.Id, vals, CancellationToken.None);

            StatusText = r.IsOk
                ? FormatApplyOk(SelectedClass.Id, sendMode, vals)
                : verb + ": " + r;
        }
        catch (Exception ex)
        {
            StatusText = verb + ": " + ex.Message;
        }
    }

    /// <summary>
    /// Returns the LUMP LIGHT-mode index for classes whose "PWM" sliders
    /// have been re-routed through `SENSOR SEND` (Issue #92).  Returns
    /// `null` for classes that still use `SENSOR PWM` (motors via the
    /// H-bridge, once #80 lands).
    /// </summary>
    private static byte? LightSendModeFor(LegoClassId classId) => classId switch
    {
        LegoClassId.Color      => (byte)3,   // COLOR mode 3 LIGHT (3×INT8 PCT)
        LegoClassId.Ultrasonic => (byte)5,   // ULTRASONIC mode 5 LIGHT (4×INT8 PCT)
        _ => null,
    };

    private async Task<BtsensorReply> ApplyLightSendAsync(
        LegoClassId classId, byte mode, int[] vals)
    {
        // LIGHT-mode payload is INT8 PCT (0..100) per channel.
        // Clamp before serialising — kernel rejects out-of-range
        // values.  Routing through SENSOR SEND because the kernel
        // SET_PWM ioctl now returns -ENOTSUP for LIGHT-bearing
        // classes (Issue #92).
        byte[] payload = new byte[vals.Length];
        for (int i = 0; i < vals.Length; i++)
        {
            int v = vals[i];
            if (v < 0) v = 0;
            else if (v > 100) v = 100;
            payload[i] = (byte)v;
        }

        return await _orchestrator.SensorSendAsync(
            classId, mode, payload, CancellationToken.None);
    }

    /// <summary>
    /// MOTOR_* dedicated BRAKE button.  Sends `SENSOR PWM <class> 0`,
    /// which the kernel `stm32_legoport_pwm_set_duty(idx, 0)` interprets
    /// as BRAKE (pybricks-compatible semantics: duty=0 short-circuits
    /// the H-bridge to brake instead of coast).  Also snaps the slider
    /// back to 0 so the UI matches the new state.
    /// </summary>
    [RelayCommand]
    public async Task BrakeAsync()
    {
        if (SelectedClass is null || !IsMotor(SelectedClass.Id)) return;

        PwmCh0 = 0;

        try
        {
            BtsensorReply r = await _orchestrator.SensorPwmAsync(
                SelectedClass.Id, new[] { 0 }, CancellationToken.None);
            StatusText = r.IsOk ? "BRAKE (pwm=0)" : "BRAKE: " + r;
        }
        catch (Exception ex)
        {
            StatusText = "BRAKE: " + ex.Message;
        }
    }

    private static string FormatApplyOk(LegoClassId classId, byte? sendMode, int[] vals)
    {
        string joined = string.Join(',', vals);
        return sendMode.HasValue
            ? $"SEND mode={sendMode.Value} → [{joined}]"
            : $"PWM → [{joined}]";
    }

    private static bool IsMotor(LegoClassId classId) =>
        classId == LegoClassId.MotorM ||
        classId == LegoClassId.MotorR ||
        classId == LegoClassId.MotorL;

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

    private static int WriteChannelCount(LegoClassId classId) => classId switch
    {
        LegoClassId.Color      => 3,
        LegoClassId.Ultrasonic => 4,
        LegoClassId.MotorM     => 1,
        LegoClassId.MotorR     => 1,
        LegoClassId.MotorL     => 1,
        _ => 0,
    };
}
