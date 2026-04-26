using Avalonia.OpenGL;
using Silk.NET.Core.Contexts;

namespace ImuViewer.App.Controls;

/// <summary>
/// Bridges Avalonia's <see cref="GlInterface"/> proc-address loader to Silk.NET's
/// <see cref="INativeContext"/> so we can call <c>GL.GetApi(this)</c> from
/// <see cref="OpenGlSceneControl"/>.
/// </summary>
internal sealed class AvaloniaSilkContext : INativeContext
{
    private readonly GlInterface _gl;

    public AvaloniaSilkContext(GlInterface gl)
    {
        _gl = gl;
    }

    public nint GetProcAddress(string proc, int? slot = null) => _gl.GetProcAddress(proc);

    public bool TryGetProcAddress(string proc, out nint addr, int? slot = null)
    {
        addr = _gl.GetProcAddress(proc);
        return addr != 0;
    }

    public void Dispose() { }
}
