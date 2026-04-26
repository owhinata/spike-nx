using System.Numerics;

namespace ImuViewer.Core.Filters;

/// <summary>
/// IMU-only (6 DOF) Madgwick orientation filter. Mirrors the reference C
/// implementation from Madgwick 2010, "An efficient orientation filter for
/// inertial and inertial/magnetic sensor arrays". Internal quaternion storage
/// uses (q0=w, q1=x, q2=y, q3=z) to match the paper.
/// </summary>
public sealed class MadgwickFilter : IOrientationFilter
{
    private float _q0 = 1f;
    private float _q1, _q2, _q3;

    public float Beta { get; set; } = 0.1f;

    public Quaternion Orientation => new(_q1, _q2, _q3, _q0);

    public void Reset()
    {
        _q0 = 1f;
        _q1 = 0f;
        _q2 = 0f;
        _q3 = 0f;
    }

    public void Update(Vector3 accelG, Vector3 gyroRadS, float dt)
    {
        float gx = gyroRadS.X;
        float gy = gyroRadS.Y;
        float gz = gyroRadS.Z;

        float q0 = _q0;
        float q1 = _q1;
        float q2 = _q2;
        float q3 = _q3;

        // Rate of change from gyroscope.
        float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Apply accel correction only if accel sample is usable (non-zero norm).
        float aNormSq = accelG.LengthSquared();
        if (aNormSq > 1e-12f)
        {
            float aRecipNorm = 1f / MathF.Sqrt(aNormSq);
            float ax = accelG.X * aRecipNorm;
            float ay = accelG.Y * aRecipNorm;
            float az = accelG.Z * aRecipNorm;

            float _2q0 = 2f * q0;
            float _2q1 = 2f * q1;
            float _2q2 = 2f * q2;
            float _2q3 = 2f * q3;
            float _4q0 = 4f * q0;
            float _4q1 = 4f * q1;
            float _4q2 = 4f * q2;
            float _8q1 = 8f * q1;
            float _8q2 = 8f * q2;
            float q0q0 = q0 * q0;
            float q1q1 = q1 * q1;
            float q2q2 = q2 * q2;
            float q3q3 = q3 * q3;

            float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float s1 = _4q1 * q3q3 - _2q3 * ax + 4f * q0q0 * q1 - _2q0 * ay
                       - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float s2 = 4f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                       - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float s3 = 4f * q1q1 * q3 - _2q1 * ax + 4f * q2q2 * q3 - _2q2 * ay;

            float sNormSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
            if (sNormSq > 1e-12f)
            {
                float sRecipNorm = 1f / MathF.Sqrt(sNormSq);
                qDot1 -= Beta * s0 * sRecipNorm;
                qDot2 -= Beta * s1 * sRecipNorm;
                qDot3 -= Beta * s2 * sRecipNorm;
                qDot4 -= Beta * s3 * sRecipNorm;
            }
        }

        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        if (qNormSq > 1e-12f)
        {
            float qRecipNorm = 1f / MathF.Sqrt(qNormSq);
            q0 *= qRecipNorm;
            q1 *= qRecipNorm;
            q2 *= qRecipNorm;
            q3 *= qRecipNorm;
        }

        _q0 = q0;
        _q1 = q1;
        _q2 = q2;
        _q3 = q3;
    }
}
