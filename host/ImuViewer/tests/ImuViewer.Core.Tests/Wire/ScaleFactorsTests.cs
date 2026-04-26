using FluentAssertions;
using ImuViewer.Core.Wire;
using Xunit;

namespace ImuViewer.Core.Tests.Wire;

public class ScaleFactorsTests
{
    [Theory]
    [InlineData(2, 4096, 0.250f)]   // 2 g FSR: 4096 LSB = 0.25 g
    [InlineData(4, 4096, 0.500f)]
    [InlineData(8, 4096, 1.000f)]   // datasheet sanity: 8 g, 4096 LSB = 1 g (1000 mg)
    [InlineData(16, 4096, 2.000f)]
    public void Accel_scaling_is_linear_int16_full_scale(int fsrG, short raw, float expectedG)
    {
        ScaleFactors.AccelG(raw, fsrG).Should().BeApproximately(expectedG, 1e-4f);
    }

    [Theory]
    [InlineData(125, 1000, 4.375f)]
    [InlineData(250, 1000, 8.750f)]
    [InlineData(500, 1000, 17.500f)]
    [InlineData(1000, 1000, 35.000f)]
    [InlineData(2000, 1000, 70.000f)]
    public void Gyro_scaling_matches_lsm6dsl_mdps_per_lsb(int fsrDps, short raw, float expectedDps)
    {
        // ±32768 LSB on the LSM6DSL gyro is NOT the same as ±FSR (datasheet Table 3
        // sensitivities: 4.375/8.75/17.5/35/70 mdps/LSB). Dividing raw by 32768 would
        // give ~13% low.
        ScaleFactors.GyroDps(raw, fsrDps).Should().BeApproximately(expectedDps, 1e-3f);
    }

    [Fact]
    public void Per_lsb_helpers_match_table()
    {
        ScaleFactors.AccelMgPerLsb(8).Should().BeApproximately(0.244140625f, 1e-6f);
        ScaleFactors.GyroMdpsPerLsb(2000).Should().BeApproximately(70f, 1e-4f);
    }
}
