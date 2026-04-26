using Silk.NET.OpenGL;

namespace ImuViewer.Rendering.Meshes;

internal sealed class CubeMesh : IDisposable
{
    private readonly PrimitiveMesh _solid;
    private readonly PrimitiveMesh _edges;

    public CubeMesh(GL gl, float halfSize = 0.5f)
    {
        float h = halfSize;
        // 8 cube vertices.
        ReadOnlySpan<float> v =
        [
            -h, -h, -h,    // 0
             h, -h, -h,    // 1
             h,  h, -h,    // 2
            -h,  h, -h,    // 3
            -h, -h,  h,    // 4
             h, -h,  h,    // 5
             h,  h,  h,    // 6
            -h,  h,  h,    // 7
        ];
        ReadOnlySpan<uint> tris =
        [
            // -Z face
            0, 2, 1,  0, 3, 2,
            // +Z face
            4, 5, 6,  4, 6, 7,
            // -Y face
            0, 1, 5,  0, 5, 4,
            // +Y face
            3, 7, 6,  3, 6, 2,
            // -X face
            0, 4, 7,  0, 7, 3,
            // +X face
            1, 2, 6,  1, 6, 5,
        ];
        ReadOnlySpan<uint> lines =
        [
            0, 1, 1, 2, 2, 3, 3, 0, // bottom
            4, 5, 5, 6, 6, 7, 7, 4, // top
            0, 4, 1, 5, 2, 6, 3, 7, // verticals
        ];

        _solid = new PrimitiveMesh(gl, v, tris, PrimitiveType.Triangles);
        _edges = new PrimitiveMesh(gl, v, lines, PrimitiveType.Lines);
    }

    public void DrawSolid() => _solid.Draw();
    public void DrawEdges() => _edges.Draw();

    public void Dispose()
    {
        _solid.Dispose();
        _edges.Dispose();
    }
}
