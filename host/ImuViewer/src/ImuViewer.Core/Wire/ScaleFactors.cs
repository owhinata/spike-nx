namespace ImuViewer.Core.Wire;

/// <summary>
/// LSM6DSL raw int16 → physical units conversion factors.
/// </summary>
/// <remarks>
/// Accel uses signed 16-bit full scale: ±32768 LSB == ±FSR. mg/LSB = fsr_g * 1000 / 32768.
/// Gyro uses the LSM6DSL datasheet sensitivity table (Table 3): 2000 dps -> 70 mdps/LSB,
/// 1000 -> 35, 500 -> 17.5, 250 -> 8.75, 125 -> 4.375. The closed form is
/// mdps/LSB = fsr_dps * 0.035. Note that gyro is NOT signed-int16 full scale; ±32768 LSB
/// covers more than the nominal full scale on the LSM6DSL, so dividing by 32768 would
/// yield a ~13% low reading and bias the Madgwick yaw integration.
/// </remarks>
public static class ScaleFactors
{
    public static float AccelMgPerLsb(int fsrG) => fsrG * 1000f / 32768f;

    public static float GyroMdpsPerLsb(int fsrDps) => fsrDps * 0.035f;

    public static float AccelG(short raw, int fsrG) =>
        raw * (float)fsrG / 32768f;

    public static float GyroDps(short raw, int fsrDps) =>
        raw * fsrDps * 0.035f / 1000f;
}
