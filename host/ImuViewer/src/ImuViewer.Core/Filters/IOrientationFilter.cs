using System.Numerics;

namespace ImuViewer.Core.Filters;

public interface IOrientationFilter
{
    Quaternion Orientation { get; }

    void Update(Vector3 accelG, Vector3 gyroRadS, float dt);

    void Reset();
}
