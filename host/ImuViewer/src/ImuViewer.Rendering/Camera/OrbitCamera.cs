using System.Numerics;

namespace ImuViewer.Rendering.Camera;

/// <summary>
/// Spherical-coordinate orbit camera for the RViz-style world view.
/// </summary>
public sealed class OrbitCamera
{
    public Vector3 Target { get; set; } = Vector3.Zero;
    public float Distance { get; private set; } = 5f;

    /// <summary>Yaw in radians, around world up (Z+).</summary>
    public float Yaw { get; private set; } = MathF.PI * 0.25f;

    /// <summary>Pitch in radians, from horizontal plane (positive looks down).</summary>
    public float Pitch { get; private set; } = MathF.PI * 0.25f;

    public float MinDistance { get; set; } = 0.5f;
    public float MaxDistance { get; set; } = 100f;
    public float MinPitch { get; set; } = -MathF.PI * 0.49f;
    public float MaxPitch { get; set; } = MathF.PI * 0.49f;

    public Vector3 Up { get; set; } = Vector3.UnitZ;

    public void Orbit(float deltaYawRad, float deltaPitchRad)
    {
        Yaw += deltaYawRad;
        Pitch = Math.Clamp(Pitch + deltaPitchRad, MinPitch, MaxPitch);
    }

    public void Zoom(float factor)
    {
        Distance = Math.Clamp(Distance * factor, MinDistance, MaxDistance);
    }

    public void Pan(Vector3 deltaWorld)
    {
        Target += deltaWorld;
    }

    public Vector3 Position
    {
        get
        {
            float cp = MathF.Cos(Pitch);
            float sp = MathF.Sin(Pitch);
            float cy = MathF.Cos(Yaw);
            float sy = MathF.Sin(Yaw);
            // Spherical: x = cos(pitch) * cos(yaw), y = cos(pitch) * sin(yaw), z = sin(pitch)
            return Target + Distance * new Vector3(cp * cy, cp * sy, sp);
        }
    }

    public Matrix4x4 GetViewMatrix()
    {
        return Matrix4x4.CreateLookAt(Position, Target, Up);
    }

    public Matrix4x4 GetProjectionMatrix(float aspect, float nearPlane = 0.05f, float farPlane = 200f)
    {
        return Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4f, aspect, nearPlane, farPlane);
    }
}
