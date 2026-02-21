using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Path
{
    public static class PathUtils
    {
        public static int FindClosestIndex(IReadOnlyList<Vector3> path, Vector3 pos)
        {
            if (path == null || path.Count == 0)
            {
                return 0;
            }

            int best = 0;
            float bestDist = float.PositiveInfinity;
            for (int i = 0; i < path.Count; i++)
            {
                float d = (path[i] - pos).sqrMagnitude;
                if (d < bestDist)
                {
                    bestDist = d;
                    best = i;
                }
            }

            return best;
        }

        public static float[] ComputeCurvatureXZ(IReadOnlyList<Vector3> path)
        {
            int n = path == null ? 0 : path.Count;
            float[] kappa = new float[n];
            if (n < 3)
            {
                return kappa;
            }

            const float eps = 1e-4f;
            kappa[0] = 0f;
            kappa[n - 1] = 0f;

            for (int i = 1; i < n - 1; i++)
            {
                Vector3 a = path[i - 1];
                Vector3 b = path[i];
                Vector3 c = path[i + 1];

                float ax = a.x;
                float az = a.z;
                float bx = b.x;
                float bz = b.z;
                float cx = c.x;
                float cz = c.z;

                float abx = bx - ax;
                float abz = bz - az;
                float bcx = cx - bx;
                float bcz = cz - bz;
                float acx = cx - ax;
                float acz = cz - az;

                float ab = Mathf.Sqrt(abx * abx + abz * abz);
                float bc = Mathf.Sqrt(bcx * bcx + bcz * bcz);
                float ac = Mathf.Sqrt(acx * acx + acz * acz);
                if (ab < eps || bc < eps || ac < eps)
                {
                    kappa[i] = 0f;
                    continue;
                }

                float area2 = (bx - ax) * (cz - az) - (bz - az) * (cx - ax);
                float area = 0.5f * area2;
                float denom = ab * bc * ac;
                if (denom < eps)
                {
                    kappa[i] = 0f;
                    continue;
                }

                float ki = (4f * area) / denom;
                if (float.IsNaN(ki) || float.IsInfinity(ki))
                {
                    ki = 0f;
                }

                kappa[i] = ki;
            }

            return kappa;
        }
    }
}
