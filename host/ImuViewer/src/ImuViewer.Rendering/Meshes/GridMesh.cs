using Silk.NET.OpenGL;

namespace ImuViewer.Rendering.Meshes;

internal sealed class GridMesh : IDisposable
{
    private readonly PrimitiveMesh _mesh;

    public GridMesh(GL gl, int halfCells = 5, float spacing = 1f)
    {
        int total = halfCells * 2 + 1;
        // Each line: 2 endpoints * 3 floats. (#lines along X) + (#lines along Y) = 2*total.
        List<float> verts = new(2 * total * 2 * 3);
        float extent = halfCells * spacing;
        for (int i = -halfCells; i <= halfCells; i++)
        {
            float a = i * spacing;
            // Lines parallel to X axis (varying Y).
            verts.AddRange([-extent, a, 0f]);
            verts.AddRange([extent, a, 0f]);
            // Lines parallel to Y axis (varying X).
            verts.AddRange([a, -extent, 0f]);
            verts.AddRange([a, extent, 0f]);
        }
        uint[] indices = new uint[verts.Count / 3];
        for (uint i = 0; i < indices.Length; i++) { indices[i] = i; }

        _mesh = new PrimitiveMesh(gl, verts.ToArray(), indices, PrimitiveType.Lines);
    }

    public void Draw() => _mesh.Draw();

    public void Dispose() => _mesh.Dispose();
}
