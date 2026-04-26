using Silk.NET.OpenGL;

namespace ImuViewer.Rendering.Meshes;

/// <summary>
/// Owns a VAO/VBO/EBO triple for a static mesh of position-only vertices.
/// </summary>
internal sealed class PrimitiveMesh : IDisposable
{
    private readonly GL _gl;
    private readonly uint _vao;
    private readonly uint _vbo;
    private readonly uint _ebo;
    private readonly int _indexCount;
    private readonly PrimitiveType _primitive;

    public PrimitiveMesh(GL gl, ReadOnlySpan<float> vertices, ReadOnlySpan<uint> indices, PrimitiveType primitive)
    {
        _gl = gl;
        _primitive = primitive;
        _indexCount = indices.Length;

        _vao = gl.GenVertexArray();
        gl.BindVertexArray(_vao);

        _vbo = gl.GenBuffer();
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vbo);
        unsafe
        {
            fixed (float* pv = vertices)
            {
                gl.BufferData(BufferTargetARB.ArrayBuffer,
                    (nuint)(vertices.Length * sizeof(float)), pv, BufferUsageARB.StaticDraw);
            }
        }

        _ebo = gl.GenBuffer();
        gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, _ebo);
        unsafe
        {
            fixed (uint* pi = indices)
            {
                gl.BufferData(BufferTargetARB.ElementArrayBuffer,
                    (nuint)(indices.Length * sizeof(uint)), pi, BufferUsageARB.StaticDraw);
            }
        }

        gl.EnableVertexAttribArray(0);
        unsafe
        {
            gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), (void*)0);
        }

        gl.BindVertexArray(0);
    }

    public void Draw()
    {
        _gl.BindVertexArray(_vao);
        unsafe
        {
            _gl.DrawElements(_primitive, (uint)_indexCount, DrawElementsType.UnsignedInt, (void*)0);
        }
        _gl.BindVertexArray(0);
    }

    public void Dispose()
    {
        _gl.DeleteBuffer(_vbo);
        _gl.DeleteBuffer(_ebo);
        _gl.DeleteVertexArray(_vao);
    }
}
