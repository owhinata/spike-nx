using Silk.NET.OpenGL;

namespace ImuViewer.Rendering.Meshes;

internal sealed class AxisGizmo : IDisposable
{
    private readonly PrimitiveMesh _x;
    private readonly PrimitiveMesh _y;
    private readonly PrimitiveMesh _z;

    public AxisGizmo(GL gl, float length = 1f)
    {
        ReadOnlySpan<uint> idx = [0, 1];
        _x = new PrimitiveMesh(gl, [0, 0, 0, length, 0, 0], idx, PrimitiveType.Lines);
        _y = new PrimitiveMesh(gl, [0, 0, 0, 0, length, 0], idx, PrimitiveType.Lines);
        _z = new PrimitiveMesh(gl, [0, 0, 0, 0, 0, length], idx, PrimitiveType.Lines);
    }

    public void DrawX() => _x.Draw();
    public void DrawY() => _y.Draw();
    public void DrawZ() => _z.Draw();

    public void Dispose()
    {
        _x.Dispose();
        _y.Dispose();
        _z.Dispose();
    }
}
