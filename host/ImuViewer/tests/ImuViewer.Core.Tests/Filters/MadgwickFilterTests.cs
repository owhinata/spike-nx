using System.Numerics;
using FluentAssertions;
using ImuViewer.Core.Filters;
using Xunit;

namespace ImuViewer.Core.Tests.Filters;

public class MadgwickFilterTests
{
    [Fact]
    public void Default_orientation_is_identity()
    {
        new MadgwickFilter().Orientation.Should().Be(Quaternion.Identity);
    }

    [Fact]
    public void Static_input_at_identity_stays_at_identity()
    {
        MadgwickFilter f = new();
        for (int i = 0; i < 60; i++)
        {
            f.Update(new Vector3(0, 0, 1), Vector3.Zero, 1f / 60f);
        }
        f.Orientation.W.Should().BeApproximately(1f, 1e-3f);
        f.Orientation.X.Should().BeApproximately(0f, 1e-3f);
        f.Orientation.Y.Should().BeApproximately(0f, 1e-3f);
        f.Orientation.Z.Should().BeApproximately(0f, 1e-3f);
    }

    [Fact]
    public void Pure_yaw_gyro_for_one_second_yields_quarter_turn_around_z()
    {
        // gyro = π/2 rad/s around Z for 1 s -> 90° yaw.
        MadgwickFilter f = new();
        Vector3 gyro = new(0, 0, MathF.PI / 2f);
        Vector3 accel = new(0, 0, 1f);
        for (int i = 0; i < 60; i++)
        {
            f.Update(accel, gyro, 1f / 60f);
        }

        // Expected: q = (cos(π/4), 0, 0, sin(π/4)) = (W=0.7071, X=0, Y=0, Z=0.7071)
        float expected = 1f / MathF.Sqrt(2f);
        f.Orientation.W.Should().BeApproximately(expected, 5e-3f);
        f.Orientation.Z.Should().BeApproximately(expected, 5e-3f);
        f.Orientation.X.Should().BeApproximately(0f, 5e-3f);
        f.Orientation.Y.Should().BeApproximately(0f, 5e-3f);
    }

    [Fact]
    public void Reset_returns_to_identity()
    {
        MadgwickFilter f = new();
        for (int i = 0; i < 30; i++)
        {
            f.Update(new Vector3(0, 0, 1), new Vector3(0, 0, 1f), 1f / 60f);
        }
        f.Orientation.Should().NotBe(Quaternion.Identity);

        f.Reset();
        f.Orientation.Should().Be(Quaternion.Identity);
    }

    [Fact]
    public void Zero_accel_norm_falls_back_to_pure_gyro_integration()
    {
        // Free-fall (accel norm 0) must not throw and should still integrate gyro.
        MadgwickFilter f = new();
        for (int i = 0; i < 60; i++)
        {
            f.Update(Vector3.Zero, new Vector3(0, 0, MathF.PI / 2f), 1f / 60f);
        }
        float expected = 1f / MathF.Sqrt(2f);
        f.Orientation.Z.Should().BeApproximately(expected, 5e-3f);
    }
}
