using System.Collections.Generic;
using Scripts.Path;
using UnityEngine;

namespace Scripts.Control
{
    public static class SpeedProfiler
    {
        public static float[] BuildSpeedProfile(
            IReadOnlyList<Vector3> path,
            float maxSpeed,
            float maxAccel,
            float maxDecel,
            float maxLateralAccel,
            float curveSafetyFactor)
        {
            int n = path?.Count ?? 0;
            float[] v = new float[n];
            if (n < 2)
            {
                return v;
            }

            const float eps = 1e-4f;
            float safeMaxSpeed = Mathf.Max(0f, maxSpeed);
            float safeMaxAccel = Mathf.Max(0f, maxAccel);
            float safeMaxDecel = Mathf.Max(0f, maxDecel);
            float safeLatAccel = Mathf.Max(0f, maxLateralAccel);
            float safeCurveFactor = Mathf.Max(0f, curveSafetyFactor);

            float[] kappa = PathUtils.ComputeCurvatureXZ(path);
            float[] vCurve = new float[n];
            for (int i = 0; i < n; i++)
            {
                float absK = Mathf.Abs(kappa[i]);
                if (absK < eps)
                {
                    vCurve[i] = safeMaxSpeed;
                }
                else
                {
                    float arg = Mathf.Max(0f, safeLatAccel / absK);
                    vCurve[i] = Mathf.Sqrt(arg) * safeCurveFactor;
                }

                vCurve[i] = Mathf.Clamp(vCurve[i], 0f, safeMaxSpeed);
            }

            v[n - 1] = 0f;
            for (int i = n - 2; i >= 0; i--)
            {
                float dist = Vector3.Distance(path[i], path[i + 1]);
                float brakeArg = v[i + 1] * v[i + 1] + 2f * safeMaxDecel * dist;
                float vMaxBrake = Mathf.Sqrt(Mathf.Max(0f, brakeArg));
                v[i] = Mathf.Min(vCurve[i], vMaxBrake);
            }

            for (int i = 0; i <= n - 2; i++)
            {
                float dist = Vector3.Distance(path[i], path[i + 1]);
                float accelArg = v[i] * v[i] + 2f * safeMaxAccel * dist;
                float vMaxAccelNext = Mathf.Sqrt(Mathf.Max(0f, accelArg));
                v[i + 1] = Mathf.Min(v[i + 1], vMaxAccelNext);
                v[i + 1] = Mathf.Min(v[i + 1], vCurve[i + 1]);
            }

            for (int i = 0; i < n; i++)
            {
                if (float.IsNaN(v[i]) || float.IsInfinity(v[i]))
                {
                    v[i] = 0f;
                }
            }

            return v;
        }
    }
}
