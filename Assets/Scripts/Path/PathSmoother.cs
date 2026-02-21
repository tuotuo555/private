using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Path
{
    public static class PathSmoother
    {
        public static List<Vector3> SmoothPolyline(IReadOnlyList<Vector3> raw, IReadOnlyList<Bounds> inflatedObstacles)
        {
            List<Vector3> result = new List<Vector3>();
            if (raw == null || raw.Count == 0)
            {
                return result;
            }

            if (raw.Count <= 2)
            {
                result.AddRange(raw);
                return result;
            }

            int anchor = 0;
            result.Add(raw[0]);

            while (anchor < raw.Count - 1)
            {
                int best = anchor + 1;
                for (int candidate = raw.Count - 1; candidate > anchor + 1; candidate--)
                {
                    if (HasLineOfSight(raw[anchor], raw[candidate], inflatedObstacles))
                    {
                        best = candidate;
                        break;
                    }
                }

                result.Add(raw[best]);
                anchor = best;
            }

            return result;
        }

        private static bool HasLineOfSight(Vector3 a, Vector3 b, IReadOnlyList<Bounds> inflatedObstacles)
        {
            for (int i = 0; i < inflatedObstacles.Count; i++)
            {
                if (SegmentIntersectsBoundsXZ(a, b, inflatedObstacles[i]))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool SegmentIntersectsBoundsXZ(Vector3 a, Vector3 b, Bounds bounds)
        {
            float x0 = a.x;
            float z0 = a.z;
            float x1 = b.x;
            float z1 = b.z;

            float dx = x1 - x0;
            float dz = z1 - z0;
            float t0 = 0f;
            float t1 = 1f;

            if (!Clip(-dx, x0 - bounds.min.x, ref t0, ref t1)) return false;
            if (!Clip(dx, bounds.max.x - x0, ref t0, ref t1)) return false;
            if (!Clip(-dz, z0 - bounds.min.z, ref t0, ref t1)) return false;
            if (!Clip(dz, bounds.max.z - z0, ref t0, ref t1)) return false;
            return true;
        }

        private static bool Clip(float p, float q, ref float t0, ref float t1)
        {
            if (Mathf.Abs(p) < 1e-6f)
            {
                return q >= 0f;
            }

            float r = q / p;
            if (p < 0f)
            {
                if (r > t1) return false;
                if (r > t0) t0 = r;
            }
            else
            {
                if (r < t0) return false;
                if (r < t1) t1 = r;
            }

            return true;
        }
    }
}
