using System.Numerics;
using ImuViewer.Rendering.Camera;
using ImuViewer.Rendering.Meshes;
using ImuViewer.Rendering.Shaders;
using Silk.NET.OpenGL;

namespace ImuViewer.Rendering;

/// <summary>
/// Draws an RViz-style scene: world grid + RGB world axes + a Cube whose
/// orientation is supplied each frame, with a Cube-local axis gizmo overlay.
/// </summary>
/// <remarks>
/// All GL resources are owned by this object. Init / Render / Dispose must be
/// called on the GL thread.
/// </remarks>
public sealed class OpenGlSceneRenderer : IDisposable
{
    private GL? _gl;
    private ShaderProgram? _shader;
    private CubeMesh? _cube;
    private GridMesh? _grid;
    private AxisGizmo? _worldAxes;
    private AxisGizmo? _cubeAxes;

    public Vector3 ClearColor { get; set; } = new(0.13f, 0.13f, 0.14f);
    public Vector3 GridColor { get; set; } = new(0.30f, 0.30f, 0.30f);
    public Vector3 CubeColor { get; set; } = new(0.85f, 0.85f, 0.85f);
    public Vector3 CubeEdgeColor { get; set; } = new(0.05f, 0.05f, 0.05f);

    public void Init(GL gl)
    {
        _gl = gl;
        _shader = ShaderProgram.FromEmbeddedResources(gl, "basic.vert", "basic.frag");
        _cube = new CubeMesh(gl, halfSize: 0.5f);
        _grid = new GridMesh(gl, halfCells: 5, spacing: 1f);
        _worldAxes = new AxisGizmo(gl, length: 1.5f);
        _cubeAxes = new AxisGizmo(gl, length: 0.7f);

        gl.Enable(EnableCap.DepthTest);
        gl.Enable(EnableCap.LineSmooth);
    }

    public void Render(int viewportWidth, int viewportHeight, OrbitCamera camera, Quaternion cubeOrientation)
    {
        GL gl = _gl ?? throw new InvalidOperationException("Init must be called first.");
        ShaderProgram shader = _shader!;
        CubeMesh cube = _cube!;
        GridMesh grid = _grid!;
        AxisGizmo worldAxes = _worldAxes!;
        AxisGizmo cubeAxes = _cubeAxes!;

        gl.Viewport(0, 0, (uint)Math.Max(1, viewportWidth), (uint)Math.Max(1, viewportHeight));
        gl.ClearColor(ClearColor.X, ClearColor.Y, ClearColor.Z, 1f);
        gl.Clear((uint)(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit));

        float aspect = viewportWidth / (float)Math.Max(1, viewportHeight);
        Matrix4x4 view = camera.GetViewMatrix();
        Matrix4x4 proj = camera.GetProjectionMatrix(aspect);
        Matrix4x4 vp = view * proj;

        shader.Use();

        // World grid.
        shader.SetMatrix4("uMvp", vp);
        shader.SetVector3("uColor", GridColor);
        grid.Draw();

        // World axes.
        shader.SetVector3("uColor", new Vector3(1f, 0f, 0f)); worldAxes.DrawX();
        shader.SetVector3("uColor", new Vector3(0f, 1f, 0f)); worldAxes.DrawY();
        shader.SetVector3("uColor", new Vector3(0f, 0.6f, 1f)); worldAxes.DrawZ();

        // Cube body.
        Matrix4x4 cubeModel = Matrix4x4.CreateFromQuaternion(cubeOrientation);
        Matrix4x4 cubeMvp = cubeModel * vp;
        shader.SetMatrix4("uMvp", cubeMvp);
        shader.SetVector3("uColor", CubeColor);
        cube.DrawSolid();

        // Cube edges.
        shader.SetVector3("uColor", CubeEdgeColor);
        cube.DrawEdges();

        // Cube-local axes.
        shader.SetVector3("uColor", new Vector3(1f, 0f, 0f)); cubeAxes.DrawX();
        shader.SetVector3("uColor", new Vector3(0f, 1f, 0f)); cubeAxes.DrawY();
        shader.SetVector3("uColor", new Vector3(0f, 0.6f, 1f)); cubeAxes.DrawZ();
    }

    public void Dispose()
    {
        _cube?.Dispose();
        _grid?.Dispose();
        _worldAxes?.Dispose();
        _cubeAxes?.Dispose();
        _shader?.Dispose();
        _cube = null;
        _grid = null;
        _worldAxes = null;
        _cubeAxes = null;
        _shader = null;
        _gl = null;
    }
}
