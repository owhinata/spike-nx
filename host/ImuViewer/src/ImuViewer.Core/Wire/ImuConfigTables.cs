namespace ImuViewer.Core.Wire;

/// <summary>
/// Lookup tables converting LSM6DSL driver-internal enum indices to
/// physical values.  Issue #139: the firmware GET ioctls and the
/// per-sample fields in <c>struct sensor_imu</c> both expose the raw
/// enum value (<c>lsm6dsl_odr_e</c>, <c>lsm6dsl_fsr_xl_e</c>,
/// <c>lsm6dsl_fsr_gy_e</c>) instead of physical Hz/g/dps, so the host
/// keeps its own copy of the driver tables to do the reverse mapping.
///
/// MUST match the enum values declared in
/// <c>boards/spike-prime-hub/src/lsm6dsl_uorb.c:124-154</c>.  The
/// sparse rows return 0 — callers should treat 0 as "unknown" and
/// fall back to whatever default suits the surface (UI shows "?",
/// math leaves output unscaled, etc.).
/// </summary>
public static class ImuConfigTables
{
    /// <summary><c>enum lsm6dsl_odr_e</c>: 0=OFF, 1=12.5Hz, 2=26Hz,
    /// 3=52Hz, 4=104Hz, 5=208Hz, 6=416Hz, 7=833Hz, 8=1660Hz, 9=3330Hz,
    /// 10=6660Hz.</summary>
    public static readonly int[] OdrIdxToHz =
        { 0, 13, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 };

    /// <summary><c>enum lsm6dsl_fsr_xl_e</c> (sparse): 0=2g, 1=16g,
    /// 2=4g, 3=8g.</summary>
    public static readonly int[] FsrXlIdxToG = { 2, 16, 4, 8 };

    /// <summary><c>enum lsm6dsl_fsr_gy_e</c> (sparse): 0=250, 1=125,
    /// 2=500, 4=1000, 6=2000 dps; indices 3/5/7 are unused and return
    /// 0.</summary>
    public static readonly int[] FsrGyIdxToDps =
        { 250, 125, 500, 0, 1000, 0, 2000 };

    /// <summary>Map an ODR enum idx to Hz; returns 0 for out-of-range idx.</summary>
    public static int OdrHz(int idx) =>
        (idx >= 0 && idx < OdrIdxToHz.Length) ? OdrIdxToHz[idx] : 0;

    /// <summary>Map an accel-FSR enum idx to g; returns 0 for out-of-range idx.</summary>
    public static int AccelFsrG(int idx) =>
        (idx >= 0 && idx < FsrXlIdxToG.Length) ? FsrXlIdxToG[idx] : 0;

    /// <summary>Map a gyro-FSR enum idx to dps; returns 0 for out-of-range idx.</summary>
    public static int GyroFsrDps(int idx) =>
        (idx >= 0 && idx < FsrGyIdxToDps.Length) ? FsrGyIdxToDps[idx] : 0;
}
