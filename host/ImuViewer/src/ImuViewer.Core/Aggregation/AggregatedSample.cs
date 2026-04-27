using System.Numerics;

namespace ImuViewer.Core.Aggregation;

/// <summary>
/// Single LSM6DSL sample expressed in world-frame physical units, ready to feed
/// into the orientation filter. <see cref="TimestampUs"/> is the absolute
/// CLOCK_BOOTTIME microsecond reading (mod 2^32) for the moment the sample was
/// latched on the Hub.
/// </summary>
public sealed record AggregatedSample(
    Vector3 AccelG,
    Vector3 GyroRadS,
    Vector3 GyroDps,
    uint TimestampUs);
