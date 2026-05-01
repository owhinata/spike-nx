using System.Buffers.Binary;
using System.Collections.Immutable;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.LegoSensor;

/// <summary>
/// Hard-coded class × mode → physical-unit decode tables for LEGO
/// Powered Up sensors.  See <c>boards/spike-prime-hub/include/board_lump.h</c>
/// and <c>apps/sensor/sensor_main.c</c> on the Hub side.  Derived from
/// the LUMP INFO_FORMAT each device advertises at SYNC time — values
/// here are stable per device firmware.
/// </summary>
public static class ScaleTables
{
    public sealed record ChannelSpec(string Name, string Unit);

    public sealed record ModeSpec(
        string Label,
        ImmutableArray<ChannelSpec> Channels);

    public sealed record ClassSpec(
        string Name,
        ImmutableDictionary<byte, ModeSpec> ModesByModeId);

    private static ChannelSpec Ch(string n, string u = "") => new(n, u);

    private static ModeSpec Mode(string label, params ChannelSpec[] channels)
        => new(label, [.. channels]);

    /// <summary>
    /// Static table of supported classes / modes.  Lookups for unknown
    /// (class, mode) pairs fall back to <see cref="Decode"/>'s raw-byte
    /// behaviour.
    /// </summary>
    public static IReadOnlyDictionary<LegoClassId, ClassSpec> ByClass { get; } =
        new Dictionary<LegoClassId, ClassSpec>
        {
            [LegoClassId.Color] = new("Color",
                ImmutableDictionary.CreateRange(new Dictionary<byte, ModeSpec>
                {
                    [0] = Mode("COLOR",   Ch("Color")),
                    [1] = Mode("REFLT",   Ch("Reflected", "%")),
                    [2] = Mode("AMBI",    Ch("Ambient",   "%")),
                    [3] = Mode("LIGHT",   Ch("LED 0", "%"), Ch("LED 1", "%"), Ch("LED 2", "%")),
                    [4] = Mode("RREFL",   Ch("Raw lo"), Ch("Raw hi")),
                    [5] = Mode("RGB I",   Ch("R"), Ch("G"), Ch("B"), Ch("IR")),
                    [6] = Mode("HSV",     Ch("H", "°"), Ch("S"), Ch("V")),
                    [7] = Mode("SHSV",    Ch("H", "°"), Ch("S"), Ch("V"), Ch("Color")),
                })),

            [LegoClassId.Ultrasonic] = new("Ultrasonic",
                ImmutableDictionary.CreateRange(new Dictionary<byte, ModeSpec>
                {
                    [0] = Mode("DISTL",   Ch("Distance", "mm")),
                    [1] = Mode("DISTS",   Ch("Distance", "mm")),
                    [2] = Mode("SINGL",   Ch("Distance", "mm")),
                    [3] = Mode("LISTN",   Ch("Listen")),
                    [4] = Mode("TRAW",    Ch("Raw")),
                    [5] = Mode("LIGHT",   Ch("LED 0", "%"), Ch("LED 1", "%"), Ch("LED 2", "%"), Ch("LED 3", "%")),
                })),

            [LegoClassId.Force] = new("Force",
                ImmutableDictionary.CreateRange(new Dictionary<byte, ModeSpec>
                {
                    [0] = Mode("FORCE",   Ch("Force", "")),
                    [1] = Mode("TOUCH",   Ch("Touched")),
                    [2] = Mode("TAP",     Ch("Taps")),
                    [4] = Mode("FRAW",    Ch("Raw")),
                })),

            [LegoClassId.MotorM] = MotorClass("Motor M"),
            [LegoClassId.MotorR] = MotorClass("Motor R"),
            [LegoClassId.MotorL] = MotorClass("Motor L"),
        };

    private static ClassSpec MotorClass(string name) => new(name,
        ImmutableDictionary.CreateRange(new Dictionary<byte, ModeSpec>
        {
            [0] = Mode("POWER", Ch("Duty", "%")),
            [1] = Mode("SPEED", Ch("Speed", "%")),
            [2] = Mode("POS",   Ch("Position", "°")),
            [3] = Mode("APOS",  Ch("Abs pos", "°")),
            [4] = Mode("CALIB", Ch("Cal lo"), Ch("Cal hi")),
        }));

    /// <summary>
    /// Decode a TLV payload into per-channel float values plus their
    /// units / a label.  Falls back to raw byte values when the
    /// (class, mode) pair has no entry in the static table.
    /// </summary>
    public static (string Label, IReadOnlyList<float> Values, IReadOnlyList<string> Units)
        Decode(LegoClassId classId, byte modeId, LegoDataType dataType,
               byte numValues, ReadOnlySpan<byte> payload)
    {
        ModeSpec? spec = null;
        if (ByClass.TryGetValue(classId, out ClassSpec? cls)
            && cls.ModesByModeId.TryGetValue(modeId, out ModeSpec? m))
        {
            spec = m;
        }

        int count = numValues;
        if (count <= 0)
        {
            count = ValueCountFromPayload(dataType, payload.Length);
        }

        if (count <= 0)
        {
            return (spec?.Label ?? $"mode {modeId}",
                    Array.Empty<float>(), Array.Empty<string>());
        }

        float[] values = new float[count];
        for (int i = 0; i < count; i++)
        {
            values[i] = ReadValue(dataType, payload, i);
        }

        string[] units = new string[count];
        if (spec is not null)
        {
            for (int i = 0; i < count; i++)
            {
                units[i] = i < spec.Channels.Length ? spec.Channels[i].Unit : "";
            }
        }
        else
        {
            for (int i = 0; i < count; i++) units[i] = "";
        }

        return (spec?.Label ?? $"mode {modeId}", values, units);
    }

    private static int ValueCountFromPayload(LegoDataType type, int payloadBytes)
    {
        int stride = StrideOf(type);
        return stride > 0 ? payloadBytes / stride : 0;
    }

    private static int StrideOf(LegoDataType type) => type switch
    {
        LegoDataType.Int8  => 1,
        LegoDataType.Int16 => 2,
        LegoDataType.Int32 => 4,
        LegoDataType.Float => 4,
        _ => 0,
    };

    private static float ReadValue(LegoDataType type, ReadOnlySpan<byte> payload, int index)
    {
        int stride = StrideOf(type);
        int off = index * stride;
        if (off + stride > payload.Length)
        {
            return 0f;
        }

        return type switch
        {
            LegoDataType.Int8  => (sbyte)payload[off],
            LegoDataType.Int16 => BinaryPrimitives.ReadInt16LittleEndian(payload.Slice(off, 2)),
            LegoDataType.Int32 => BinaryPrimitives.ReadInt32LittleEndian(payload.Slice(off, 4)),
            LegoDataType.Float => BinaryPrimitives.ReadSingleLittleEndian(payload.Slice(off, 4)),
            _ => 0f,
        };
    }

    /// <summary>
    /// Channel labels for the given (class, mode), or sequential
    /// "ch0..chN-1" if unknown.  Used by the time-series plot.
    /// </summary>
    public static IReadOnlyList<string> ChannelLabels(
        LegoClassId classId, byte modeId, int fallbackCount)
    {
        if (ByClass.TryGetValue(classId, out ClassSpec? cls)
            && cls.ModesByModeId.TryGetValue(modeId, out ModeSpec? m)
            && m.Channels.Length > 0)
        {
            return m.Channels.Select(c => c.Name).ToArray();
        }

        if (fallbackCount <= 0)
        {
            return Array.Empty<string>();
        }

        string[] names = new string[fallbackCount];
        for (int i = 0; i < fallbackCount; i++) names[i] = $"ch{i}";
        return names;
    }

    public static string ClassName(LegoClassId classId) =>
        ByClass.TryGetValue(classId, out ClassSpec? c) ? c.Name : classId.ToString();
}
