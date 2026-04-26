using System.Numerics;

namespace ImuViewer.Core.Coordinates;

/// <summary>
/// Rotates LSM6DSL chip-frame vectors into the application's world frame
/// (right-handed, X forward / Y left / Z up). The default identity is a
/// placeholder until the SPIKE Prime Hub mount orientation is calibrated.
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
