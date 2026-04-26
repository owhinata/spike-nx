using System.Numerics;
using Avalonia;
using Avalonia.Input;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using ImuViewer.Rendering;
using ImuViewer.Rendering.Camera;
using Silk.NET.OpenGL;

namespace ImuViewer.App.Controls;

/// <summary>
/// Avalonia-hosted OpenGL view that delegates rendering to the Silk.NET-based
/// <see cref="OpenGlSceneRenderer"/>. Exposes <see cref="Orientation"/> as a
/// styled property so MainViewModel can bind the Madgwick output here.
/// </summary>
public sealed class OpenGlSceneControl : OpenGlControlBase
{
    public static readonly StyledProperty<Quaternion> OrientationProperty =
        AvaloniaProperty.Register<OpenGlSceneControl, Quaternion>(nameof(Orientation), Quaternion.Identity);

    private readonly OpenGlSceneRenderer _renderer = new();
    private readonly OrbitCamera _camera = new();
    private GL? _gl;
    private AvaloniaSilkContext? _ctx;
    private Point? _lastPointer;
    private bool _isDragging;
    private bool _isShiftDragging;

    public Quaternion Orientation
    {
        get => GetValue(OrientationProperty);
        set => SetValue(OrientationProperty, value);
    }

    static OpenGlSceneControl()
    {
        OrientationProperty.Changed.AddClassHandler<OpenGlSceneControl>((c, _) => c.RequestNextFrameRendering());
    }

    public OpenGlSceneControl()
    {
        Focusable = true;
    }

    protected override void OnOpenGlInit(GlInterface gl)
    {
        _ctx = new AvaloniaSilkContext(gl);
        _gl = GL.GetApi(_ctx);
        _renderer.Init(_gl);
    }

    protected override void OnOpenGlRender(GlInterface gl, int fb)
    {
        if (_gl is null)
        {
            return;
        }
        double scale = VisualRoot?.RenderScaling ?? 1.0;
        int pw = (int)Math.Max(1, Math.Round(Bounds.Width * scale));
        int ph = (int)Math.Max(1, Math.Round(Bounds.Height * scale));
        _renderer.Render(pw, ph, _camera, Orientation);
    }

    protected override void OnOpenGlDeinit(GlInterface gl)
    {
        _renderer.Dispose();
        _gl = null;
        _ctx = null;
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        _lastPointer = e.GetPosition(this);
        _isDragging = e.GetCurrentPoint(this).Properties.IsLeftButtonPressed;
        _isShiftDragging = _isDragging && (e.KeyModifiers & KeyModifiers.Shift) != 0;
        if (_isDragging) { e.Pointer.Capture(this); }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        _lastPointer = null;
        _isDragging = false;
        _isShiftDragging = false;
        e.Pointer.Capture(null);
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (!_isDragging || _lastPointer is null)
        {
            return;
        }
        Point now = e.GetPosition(this);
        Point last = _lastPointer.Value;
        double dx = now.X - last.X;
        double dy = now.Y - last.Y;
        _lastPointer = now;

        if (_isShiftDragging)
        {
            float panScale = _camera.Distance * 0.0025f;
            // Pan in screen-aligned axes projected back to world.
            Matrix4x4 view = _camera.GetViewMatrix();
            if (Matrix4x4.Invert(view, out Matrix4x4 invView))
            {
                Vector3 right = new(invView.M11, invView.M12, invView.M13);
                Vector3 up = new(invView.M21, invView.M22, invView.M23);
                _camera.Pan((-(float)dx) * panScale * right + (float)dy * panScale * up);
            }
        }
        else
        {
            _camera.Orbit((float)(-dx * 0.005), (float)(-dy * 0.005));
        }
        RequestNextFrameRendering();
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        float factor = e.Delta.Y > 0 ? 0.9f : 1.1f;
        _camera.Zoom(factor);
        RequestNextFrameRendering();
    }
}
