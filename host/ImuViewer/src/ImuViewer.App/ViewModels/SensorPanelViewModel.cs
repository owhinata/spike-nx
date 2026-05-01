using System.Collections.ObjectModel;
using System.Globalization;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using ImuViewer.Core.LegoSensor;
using ImuViewer.Core.Wire;

namespace ImuViewer.App.ViewModels;

/// <summary>
/// Read-only display panel for one LEGO sensor class — title +
/// connection status + plot + last decoded values.  All write controls
/// (mode select / SEND / PWM) live in the left sidebar
/// (<see cref="SensorWriteViewModel"/>) so the panel grid stays
/// compact and uniform.
/// </summary>
public sealed partial class SensorPanelViewModel : ObservableObject
{
    /// <summary>Maximum points kept in <see cref="RecentPoints"/>.</summary>
    public const int MaxPlotPoints = 600;

    public LegoClassId ClassId { get; }
    public string Title { get; }

    public SensorPanelViewModel(LegoClassId classId)
    {
        ClassId = classId;
        Title = ScaleTables.ClassName(classId);
        ConnectionStatus = "未接続";
    }

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(StatusForeground))]
    private bool _isBound;

    [ObservableProperty]
    private string _connectionStatus;

    [ObservableProperty]
    private string _portLabel = "-";

    [ObservableProperty]
    private string _modeLabel = string.Empty;

    [ObservableProperty]
    private string _latestValuesText = string.Empty;

    [ObservableProperty]
    private int _channelCount;

    [ObservableProperty]
    private byte _ageDeci10ms;

    /// <summary>Bound recent samples (newest last).  Read only via UI thread.</summary>
    public ObservableCollection<LegoSamplePoint> RecentPoints { get; } = new();

    public string StatusForeground => IsBound ? "#80FF80" : "#808080";

    /// <summary>Re-render the panel using the supplied class state.</summary>
    public void ApplyStatus(LegoSampleAggregator.ClassState state)
    {
        Dispatcher.UIThread.Post(() =>
        {
            IsBound = state.IsBound;
            PortLabel = state.PortId is byte p ? ((char)('A' + p)).ToString() : "-";
            ModeLabel = state.ModeId is byte m
                ? $"Mode {m} ({ModeLabelOf(m)})"
                : string.Empty;
            AgeDeci10ms = state.Age10ms;
            ConnectionStatus = state.IsBound
                ? $"Port {PortLabel} · {ModeLabel} · {state.Age10ms * 10} ms ago"
                : "未接続";
        });
    }

    public void AppendSample(LegoSamplePoint sample)
    {
        Dispatcher.UIThread.Post(() =>
        {
            RecentPoints.Add(sample);
            while (RecentPoints.Count > MaxPlotPoints)
            {
                RecentPoints.RemoveAt(0);
            }
            ChannelCount = sample.Values.Length;
            LatestValuesText = FormatValues(sample);
        });
    }

    public void ResetPlot()
    {
        Dispatcher.UIThread.Post(() =>
        {
            RecentPoints.Clear();
            LatestValuesText = string.Empty;
        });
    }

    private string ModeLabelOf(byte modeId)
    {
        if (ScaleTables.ByClass.TryGetValue(ClassId, out ScaleTables.ClassSpec? cls)
            && cls.ModesByModeId.TryGetValue(modeId, out ScaleTables.ModeSpec? m))
        {
            return m.Label;
        }
        return "?";
    }

    private static string FormatValues(LegoSamplePoint sample)
    {
        if (sample.Values.IsDefaultOrEmpty || sample.Values.Length == 0)
        {
            return string.Empty;
        }

        string[] parts = new string[sample.Values.Length];
        for (int i = 0; i < sample.Values.Length; i++)
        {
            float v = sample.Values[i];
            string unit = i < sample.Units.Length ? sample.Units[i] : "";
            parts[i] = string.IsNullOrEmpty(unit)
                ? v.ToString("0.##", CultureInfo.InvariantCulture)
                : $"{v.ToString("0.##", CultureInfo.InvariantCulture)} {unit}";
        }
        return string.Join(" / ", parts);
    }
}
