using System.Numerics;

namespace ImuViewer.Core.Wire;

public readonly record struct ImuSample(
    short RawAx,
    short RawAy,
    short RawAz,
    short RawGx,
    short RawGy,
    short RawGz,
    uint TimestampDeltaUs)
{
    public Vector3 ToAccelG(float accelMgPerLsb) =>
        new(
            RawAx * accelMgPerLsb / 1000f,
            RawAy * accelMgPerLsb / 1000f,
            RawAz * accelMgPerLsb / 1000f);

    public Vector3 ToGyroDps(float gyroMdpsPerLsb) =>
        new(
            RawGx * gyroMdpsPerLsb / 1000f,
            RawGy * gyroMdpsPerLsb / 1000f,
            RawGz * gyroMdpsPerLsb / 1000f);
}
