using System.Numerics;
using System.Reflection;
using System.Text;
using Silk.NET.OpenGL;

namespace ImuViewer.Rendering.Shaders;

internal sealed class ShaderProgram : IDisposable
{
    private readonly GL _gl;
    public uint Handle { get; }

    private ShaderProgram(GL gl, uint handle)
    {
        _gl = gl;
        Handle = handle;
    }

    public static ShaderProgram FromEmbeddedResources(GL gl, string vertResourceName, string fragResourceName)
    {
        string vertSrc = ReadResource(vertResourceName);
        string fragSrc = ReadResource(fragResourceName);
        return FromSource(gl, vertSrc, fragSrc);
    }

    public static ShaderProgram FromSource(GL gl, string vertSrc, string fragSrc)
    {
        uint vert = Compile(gl, ShaderType.VertexShader, vertSrc);
        uint frag = Compile(gl, ShaderType.FragmentShader, fragSrc);
        uint prog = gl.CreateProgram();
        gl.AttachShader(prog, vert);
        gl.AttachShader(prog, frag);
        gl.LinkProgram(prog);
        gl.GetProgram(prog, ProgramPropertyARB.LinkStatus, out int linked);
        if (linked == 0)
        {
            string log = gl.GetProgramInfoLog(prog);
            gl.DeleteProgram(prog);
            gl.DeleteShader(vert);
            gl.DeleteShader(frag);
            throw new InvalidOperationException($"shader link failed: {log}");
        }
        gl.DetachShader(prog, vert);
        gl.DetachShader(prog, frag);
        gl.DeleteShader(vert);
        gl.DeleteShader(frag);
        return new ShaderProgram(gl, prog);
    }

    public void Use() => _gl.UseProgram(Handle);

    public unsafe void SetMatrix4(string name, Matrix4x4 matrix)
    {
        int loc = _gl.GetUniformLocation(Handle, name);
        if (loc < 0) return;
        _gl.UniformMatrix4(loc, 1, false, (float*)&matrix);
    }

    public void SetVector3(string name, Vector3 value)
    {
        int loc = _gl.GetUniformLocation(Handle, name);
        if (loc < 0) return;
        _gl.Uniform3(loc, value.X, value.Y, value.Z);
    }

    public void Dispose()
    {
        _gl.DeleteProgram(Handle);
    }

    private static uint Compile(GL gl, ShaderType type, string source)
    {
        uint id = gl.CreateShader(type);
        gl.ShaderSource(id, source);
        gl.CompileShader(id);
        gl.GetShader(id, ShaderParameterName.CompileStatus, out int status);
        if (status == 0)
        {
            string log = gl.GetShaderInfoLog(id);
            gl.DeleteShader(id);
            throw new InvalidOperationException($"{type} compile failed: {log}");
        }
        return id;
    }

    private static string ReadResource(string name)
    {
        Assembly asm = typeof(ShaderProgram).Assembly;
        string fullName = asm.GetManifestResourceNames()
            .FirstOrDefault(n => n.EndsWith("." + name, StringComparison.Ordinal) || n.EndsWith(name, StringComparison.Ordinal))
            ?? throw new InvalidOperationException($"shader resource not found: {name}");
        using Stream s = asm.GetManifestResourceStream(fullName)
            ?? throw new InvalidOperationException($"shader resource stream null: {fullName}");
        using StreamReader r = new(s, Encoding.UTF8);
        return r.ReadToEnd();
    }
}
