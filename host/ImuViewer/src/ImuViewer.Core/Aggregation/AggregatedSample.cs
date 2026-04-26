using System.Numerics;

namespace ImuViewer.Core.Aggregation;

public sealed record AggregatedSample(
    Vector3 AccelG,
    Vector3 GyroRadS,
    Vector3 GyroDps,
    uint FrameTimestampUs,
    int FrameSampleCount,
    long Seq);
