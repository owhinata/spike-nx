using System.Globalization;
using System.Numerics;

namespace ImuViewer.Core.Calibration;

/// <summary>
/// Offline IMU calibration produced by spike-nx <c>tools/imu_cal/</c>
/// (imu_tk_output_to_cfg.py). Read from a properties-format text file and
/// applied in raw-LSB space: <c>corrected[i] = sum_j(M[i][j] * (raw[j] - bias[j]))</c>.
/// </summary>
/// <remarks>
/// File schema (schema_version=1) example:
/// <code>
/// schema_version = 1
/// fsr_gy_dps = 1000
/// fsr_xl_g = 2
/// odr_hz = 107
/// ambient_temp_c = 28.1
/// gyro_bias_lsb_x1000  = 22406 65567 11412
/// accel_bias_lsb_x1000 = -390870 12782 80597
/// gyro_M_x1000  = 997 -1 18 5 988 -3 -1 -10 1002        // row-major 3x3
/// accel_M_x1000 = 1008 -2 9 0 1002 3 0 0 1002
/// </code>
/// Bias and matrix are stored as <c>x1000</c> fixed-point on disk; this class
/// converts to <see cref="float"/> at load time so the per-sample matmul is
/// plain Vector3 math.
/// </remarks>
public sealed class ImuCalibration
{
    public const int SupportedSchemaVersion = 1;

    public int FsrGyDps { get; }
    public int FsrXlG { get; }
    public int OdrHz { get; }
    public float AmbientTempC { get; }

    private readonly Vector3 _gyroBias;
    private readonly Vector3 _accelBias;
    private readonly float[] _gyroM;
    private readonly float[] _accelM;

    private ImuCalibration(
        int fsrGyDps,
        int fsrXlG,
        int odrHz,
        float ambientTempC,
        Vector3 gyroBias,
        Vector3 accelBias,
        float[] gyroM,
        float[] accelM)
    {
        FsrGyDps = fsrGyDps;
        FsrXlG = fsrXlG;
        OdrHz = odrHz;
        AmbientTempC = ambientTempC;
        _gyroBias = gyroBias;
        _accelBias = accelBias;
        _gyroM = gyroM;
        _accelM = accelM;
    }

    public Vector3 ApplyGyro(short rawX, short rawY, short rawZ) =>
        Apply(rawX, rawY, rawZ, _gyroBias, _gyroM);

    public Vector3 ApplyAccel(short rawX, short rawY, short rawZ) =>
        Apply(rawX, rawY, rawZ, _accelBias, _accelM);

    private static Vector3 Apply(short rawX, short rawY, short rawZ, Vector3 bias, float[] m)
    {
        float dx = rawX - bias.X;
        float dy = rawY - bias.Y;
        float dz = rawZ - bias.Z;
        return new Vector3(
            m[0] * dx + m[1] * dy + m[2] * dz,
            m[3] * dx + m[4] * dy + m[5] * dz,
            m[6] * dx + m[7] * dy + m[8] * dz);
    }

    public static ImuCalibration Load(string path)
    {
        Dictionary<string, string> kv = new(StringComparer.Ordinal);
        foreach (string raw in File.ReadAllLines(path))
        {
            string line = raw;
            int hash = line.IndexOf('#');
            if (hash >= 0)
            {
                line = line[..hash];
            }
            line = line.Trim();
            if (line.Length == 0)
            {
                continue;
            }
            int eq = line.IndexOf('=');
            if (eq < 0)
            {
                throw new FormatException($"{path}: malformed line (no '='): {raw}");
            }
            string key = line[..eq].Trim();
            string val = line[(eq + 1)..].Trim();
            kv[key] = val;
        }

        int schema = ParseInt(kv, "schema_version", path);
        if (schema != SupportedSchemaVersion)
        {
            throw new InvalidDataException(
                $"{path}: schema_version={schema}, supported={SupportedSchemaVersion}");
        }

        int fsrGy = ParseInt(kv, "fsr_gy_dps", path);
        int fsrXl = ParseInt(kv, "fsr_xl_g", path);
        int odr = ParseInt(kv, "odr_hz", path);
        float temp = ParseFloat(kv, "ambient_temp_c", path);

        Vector3 gyroBias = ParseVec3(kv, "gyro_bias_lsb_x1000", path, scale: 0.001f);
        Vector3 accelBias = ParseVec3(kv, "accel_bias_lsb_x1000", path, scale: 0.001f);
        float[] gyroM = ParseMat3(kv, "gyro_M_x1000", path, scale: 0.001f);
        float[] accelM = ParseMat3(kv, "accel_M_x1000", path, scale: 0.001f);

        return new ImuCalibration(fsrGy, fsrXl, odr, temp, gyroBias, accelBias, gyroM, accelM);
    }

    private static string Require(Dictionary<string, string> kv, string key, string path)
    {
        if (!kv.TryGetValue(key, out string? v))
        {
            throw new InvalidDataException($"{path}: missing key '{key}'");
        }
        return v;
    }

    private static int ParseInt(Dictionary<string, string> kv, string key, string path) =>
        int.Parse(Require(kv, key, path), CultureInfo.InvariantCulture);

    private static float ParseFloat(Dictionary<string, string> kv, string key, string path) =>
        float.Parse(Require(kv, key, path), CultureInfo.InvariantCulture);

    private static Vector3 ParseVec3(
        Dictionary<string, string> kv, string key, string path, float scale)
    {
        string[] toks = Require(kv, key, path)
            .Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (toks.Length != 3)
        {
            throw new InvalidDataException(
                $"{path}: '{key}' expects 3 values, got {toks.Length}");
        }
        return new Vector3(
            int.Parse(toks[0], CultureInfo.InvariantCulture) * scale,
            int.Parse(toks[1], CultureInfo.InvariantCulture) * scale,
            int.Parse(toks[2], CultureInfo.InvariantCulture) * scale);
    }

    private static float[] ParseMat3(
        Dictionary<string, string> kv, string key, string path, float scale)
    {
        string[] toks = Require(kv, key, path)
            .Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (toks.Length != 9)
        {
            throw new InvalidDataException(
                $"{path}: '{key}' expects 9 values, got {toks.Length}");
        }
        float[] m = new float[9];
        for (int i = 0; i < 9; i++)
        {
            m[i] = int.Parse(toks[i], CultureInfo.InvariantCulture) * scale;
        }
        return m;
    }
}
