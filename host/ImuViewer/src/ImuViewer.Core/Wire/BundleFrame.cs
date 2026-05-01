using System.Collections.Immutable;

namespace ImuViewer.Core.Wire;

/// <summary>
/// A complete BUNDLE frame: header + IMU subsection + TLV subsection
/// (always WireConstants.TlvCount entries, one per LegoClassId).
/// </summary>
public sealed record BundleFrame(
    BundleFrameHeader Header,
    ImmutableArray<ImuSample> ImuSamples,
    ImmutableArray<LegoTlv> Tlvs)
{
    /// <summary>
    /// Reconstruct the absolute timestamp (low 32 bits, µs) of the
    /// IMU sample at <paramref name="sampleIndex"/>.  Wraps cleanly across
    /// the 32-bit boundary (matches firmware mod-2^32 arithmetic).
    /// </summary>
    public uint AbsoluteSampleTimestampUs(int sampleIndex) =>
        unchecked(Header.TickTsUs + ImuSamples[sampleIndex].TimestampDeltaUs);
}
