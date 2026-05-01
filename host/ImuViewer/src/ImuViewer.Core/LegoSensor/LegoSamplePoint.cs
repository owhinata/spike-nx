using System.Collections.Immutable;

namespace ImuViewer.Core.LegoSensor;

/// <summary>
/// A single decoded LEGO sensor sample, ready for display.  Multiple
/// channel values share a single timestamp so the plot can render them
/// stacked.
/// </summary>
public sealed record LegoSamplePoint(
    /// <summary>Bundle <c>tick_ts_us</c> at which the TLV arrived.  µs, mod 2^32.</summary>
    uint TickTsUs,
    byte ModeId,
    string Label,
    ImmutableArray<float> Values,
    ImmutableArray<string> Units);
