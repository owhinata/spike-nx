using System.Collections.Immutable;

namespace ImuViewer.Core.Wire;

public sealed record ImuFrame(ImuFrameHeader Header, ImmutableArray<ImuSample> Samples)
{
    public ulong AbsoluteTimestampUs(int sampleIndex) =>
        unchecked((uint)(Header.FirstSampleTimestampUs + Samples[sampleIndex].TimestampDeltaUs));
}
