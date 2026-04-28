using System.Numerics;

namespace ImuViewer.Core.Coordinates;

/// <summary>
/// Rotates Hub-published vectors into the application's world frame
/// (right-handed, X forward / Y left / Z up). As of Issue #67 the
/// SPIKE Prime Hub LSM6DSL driver already publishes Hub body frame
/// (chip frame Y/Z negated on the Hub), so the default identity now
/// matches the wire contract. Kept around as an extension point for
/// future boards whose mount orientation differs from the Hub's.
/// </summary>
public sealed class ChipToWorldTransform
{
    public static ChipToWorldTransform Identity { get; } = new(Quaternion.Identity);

    public Quaternion MountOffset { get; }

    public ChipToWorldTransform(Quaternion mountOffset)
    {
        MountOffset = mountOffset;
    }

    public Vector3 Apply(Vector3 chipFrameVector) =>
        Vector3.Transform(chipFrameVector, MountOffset);
}
