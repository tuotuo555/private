using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Control
{
    public class PurePursuitFollower
    {
        public float ComputeSteer(
            IReadOnlyList<Vector3> path,
            Vector3 position,
            Vector3 forward,
            float speed,
            int currentClosestIndex,
            float lookaheadBase,
            float lookaheadPerSpeed,
            float wheelBase,
            float maxSteerDeg,
            out int closestIndex,
            out Vector3 lookaheadPoint)
        {
            closestIndex = FindClosestProgressIndex(
                path,
                position,
                currentClosestIndex,
                out int closestSegmentIndex,
                out float segmentT);
            float lookaheadDistance = Mathf.Max(1f, lookaheadBase + speed * lookaheadPerSpeed);
            lookaheadPoint = FindLookaheadPoint(path, closestSegmentIndex, segmentT, lookaheadDistance);

            Vector3 fwd = forward;
            fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-6f)
            {
                return 0f;
            }

            fwd.Normalize();
            Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
            Vector3 toTarget = lookaheadPoint - position;
            toTarget.y = 0f;

            float x = Vector3.Dot(toTarget, right);
            float z = Vector3.Dot(toTarget, fwd);
            float ld = Mathf.Max(0.5f, toTarget.magnitude);

            if (z < 0.01f)
            {
                return Mathf.Clamp(x >= 0f ? 1f : -1f, -1f, 1f);
            }

            float curvature = (2f * x) / (ld * ld);
            float steerRad = Mathf.Atan(wheelBase * curvature);
            float steerDeg = steerRad * Mathf.Rad2Deg;
            return Mathf.Clamp(steerDeg / Mathf.Max(1f, maxSteerDeg), -1f, 1f);
        }

        private static int FindClosestProgressIndex(
            IReadOnlyList<Vector3> path,
            Vector3 position,
            int currentClosestIndex,
            out int segmentIndex,
            out float segmentT)
        {
            segmentIndex = 0;
            segmentT = 0f;

            if (path == null || path.Count == 0)
            {
                return 0;
            }

            if (path.Count == 1)
            {
                return 0;
            }

            int startSegment = Mathf.Clamp(currentClosestIndex - 1, 0, path.Count - 2);
            int bestSegment = startSegment;
            float bestT = 0f;
            float bestDist = float.PositiveInfinity;

            for (int i = startSegment; i < path.Count - 1; i++)
            {
                Vector3 a = path[i];
                Vector3 b = path[i + 1];
                Vector3 ab = b - a;
                float abLenSq = ab.sqrMagnitude;
                float t = 0f;

                if (abLenSq > 1e-6f)
                {
                    t = Mathf.Clamp01(Vector3.Dot(position - a, ab) / abLenSq);
                }

                Vector3 projected = a + ab * t;
                float sqDist = (projected - position).sqrMagnitude;
                if (sqDist < bestDist)
                {
                    bestDist = sqDist;
                    bestSegment = i;
                    bestT = t;
                }
            }

            segmentIndex = bestSegment;
            segmentT = bestT;

            int candidateClosest = bestSegment + (bestT >= 0.5f ? 1 : 0);
            return Mathf.Clamp(Mathf.Max(currentClosestIndex, candidateClosest), 0, path.Count - 1);
        }

        private static Vector3 FindLookaheadPoint(
            IReadOnlyList<Vector3> path,
            int segmentIndex,
            float segmentT,
            float distanceAhead)
        {
            if (path == null || path.Count == 0)
            {
                return Vector3.zero;
            }

            if (path.Count == 1)
            {
                return path[0];
            }

            int idx = Mathf.Clamp(segmentIndex, 0, path.Count - 2);
            float t = Mathf.Clamp01(segmentT);
            Vector3 current = Vector3.Lerp(path[idx], path[idx + 1], t);
            float remaining = Mathf.Max(0.1f, distanceAhead);

            for (int i = idx; i < path.Count - 1; i++)
            {
                Vector3 segEnd = path[i + 1];
                float segLen = Vector3.Distance(current, segEnd);
                if (segLen >= remaining)
                {
                    float segT = remaining / Mathf.Max(1e-4f, segLen);
                    return Vector3.Lerp(current, segEnd, segT);
                }

                remaining -= segLen;
                current = segEnd;
            }

            return path[path.Count - 1];
        }
    }
}
