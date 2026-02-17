using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class PurePursuitFollower
{
    [Header("Lookahead")]
    public float lookaheadStraight = 8f;
    public float lookaheadCorner = 4.5f;
    public float minLookahead = 2f;

    [Header("Steering")]
    public float steerClamp = 1f;
    [Range(0f, 1f)] public float steerSmoothing = 0.2f;
    public float wheelBase = 2.6f;
    public float maxSteerDeg = 25f;

    [Header("Corner Detection")]
    public float cornerLookaheadDistance = 18f;
    public float cornerLowDeg = 8f;
    public float cornerHighDeg = 55f;

    [Header("Speed Mapping")]
    public float vSharp = 3f;
    public float vStraight = 12f;
    public float goalSlowRadius = 8f;
    public float stopDistance = 1f;

    public struct ControlOutput
    {
        public float steerCmd;
        public float targetSpeed;
        public bool hasPath;
        public bool reachedGoal;
        public int closestIndex;
        public Vector3 targetPoint;
    }

    private readonly List<Vector3> path = new List<Vector3>(512);
    private int closestIndex;
    private float prevSteer;

    public int PathCount => path.Count;

    public void ResetState()
    {
        closestIndex = 0;
        prevSteer = 0f;
    }

    public void SetPath(List<Vector3> worldPoints)
    {
        path.Clear();
        if (worldPoints == null)
        {
            ResetState();
            return;
        }

        for (int i = 0; i < worldPoints.Count; i++)
        {
            path.Add(worldPoints[i]);
        }

        ResetState();
    }

    public ControlOutput UpdateControl(Pose2D currentPose, float currentSpeed)
    {
        ControlOutput output = new ControlOutput
        {
            steerCmd = 0f,
            targetSpeed = 0f,
            hasPath = path.Count >= 2,
            reachedGoal = false,
            closestIndex = closestIndex,
            targetPoint = Vector3.zero
        };

        if (!output.hasPath)
        {
            return output;
        }

        FindClosest(currentPose.pos, out int segIdx, out Vector2 closestPoint);
        closestIndex = Mathf.Max(closestIndex, segIdx);

        float remaining = RemainingDistance(closestIndex, closestPoint);
        if (remaining <= Mathf.Max(0.05f, stopDistance))
        {
            output.reachedGoal = true;
            output.targetSpeed = 0f;
            output.steerCmd = 0f;
            output.closestIndex = closestIndex;
            output.targetPoint = path[path.Count - 1];
            return output;
        }

        float maxTurnDeg = UpcomingTurnDeg(closestIndex, closestPoint);
        float corner01 = Mathf.InverseLerp(cornerLowDeg, Mathf.Max(cornerLowDeg + 1f, cornerHighDeg), maxTurnDeg);

        float lookahead = Mathf.Lerp(lookaheadStraight, lookaheadCorner, corner01);
        lookahead = Mathf.Max(lookahead, Mathf.Max(0.1f, minLookahead));

        Vector2 target2 = PointAhead(closestIndex, closestPoint, lookahead);
        output.targetPoint = Pose2D.ToWorldXZ(target2, path[Mathf.Clamp(closestIndex, 0, path.Count - 1)].y);

        Vector2 delta = target2 - currentPose.pos;
        Vector2 right = Math2D.Right(currentPose.yawRad);

        float localX = Vector2.Dot(delta, right);
        float L = Mathf.Max(minLookahead, delta.magnitude);
        float kappa = 2f * localX / Mathf.Max(0.25f, L * L);

        float steerAngle = Mathf.Atan(wheelBase * kappa);
        float maxSteerRad = Mathf.Max(1f * Mathf.Deg2Rad, maxSteerDeg * Mathf.Deg2Rad);
        float steerRaw = Mathf.Clamp(steerAngle / maxSteerRad, -steerClamp, steerClamp);

        float blend = 1f - Mathf.Clamp01(steerSmoothing);
        float steerCmd = Mathf.Lerp(prevSteer, steerRaw, blend);
        prevSteer = steerCmd;

        float targetSpeed = Mathf.Lerp(vStraight, vSharp, corner01);

        if (remaining < Mathf.Max(0.1f, goalSlowRadius))
        {
            float slow01 = Mathf.Clamp01(remaining / Mathf.Max(0.1f, goalSlowRadius));
            targetSpeed *= slow01;
        }

        if (maxTurnDeg > cornerHighDeg)
        {
            targetSpeed = Mathf.Min(targetSpeed, vSharp);
        }

        targetSpeed = Mathf.Clamp(targetSpeed, 0f, Mathf.Max(vStraight, vSharp));

        output.steerCmd = steerCmd;
        output.targetSpeed = targetSpeed;
        output.closestIndex = closestIndex;
        return output;
    }

    private void FindClosest(Vector2 p, out int segIdx, out Vector2 closest)
    {
        segIdx = 0;
        closest = new Vector2(path[0].x, path[0].z);

        float best = float.PositiveInfinity;
        int i0 = Mathf.Clamp(closestIndex, 0, path.Count - 2);
        int i1 = Mathf.Clamp(closestIndex + 40, 0, path.Count - 2);

        for (int i = i0; i <= i1; i++)
        {
            Vector2 a = new Vector2(path[i].x, path[i].z);
            Vector2 b = new Vector2(path[i + 1].x, path[i + 1].z);
            Vector2 ab = b - a;
            float den = ab.sqrMagnitude;
            if (den < 1e-8f) continue;

            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / den);
            Vector2 c = a + t * ab;
            float d2 = (p - c).sqrMagnitude;
            if (d2 < best)
            {
                best = d2;
                segIdx = i;
                closest = c;
            }
        }
    }

    private float RemainingDistance(int segIdx, Vector2 closest)
    {
        float remain = 0f;

        Vector2 next = new Vector2(path[segIdx + 1].x, path[segIdx + 1].z);
        remain += Vector2.Distance(closest, next);

        for (int i = segIdx + 1; i < path.Count - 1; i++)
        {
            Vector2 a = new Vector2(path[i].x, path[i].z);
            Vector2 b = new Vector2(path[i + 1].x, path[i + 1].z);
            remain += Vector2.Distance(a, b);
        }

        return remain;
    }

    private float UpcomingTurnDeg(int segIdx, Vector2 closest)
    {
        if (path.Count < 3)
        {
            return 0f;
        }

        Vector2 currentDir = new Vector2(path[segIdx + 1].x, path[segIdx + 1].z) - closest;
        if (currentDir.sqrMagnitude < 1e-8f)
        {
            currentDir = new Vector2(path[segIdx + 1].x - path[segIdx].x, path[segIdx + 1].z - path[segIdx].z);
        }

        if (currentDir.sqrMagnitude < 1e-8f)
        {
            return 0f;
        }

        currentDir.Normalize();

        float maxTurn = 0f;
        float traveled = 0f;

        Vector2 prev = closest;
        for (int i = segIdx + 1; i < path.Count; i++)
        {
            Vector2 p = new Vector2(path[i].x, path[i].z);
            traveled += Vector2.Distance(prev, p);
            if (traveled > Mathf.Max(0.1f, cornerLookaheadDistance)) break;

            Vector2 dir = p - prev;
            if (dir.sqrMagnitude > 1e-8f)
            {
                dir.Normalize();
                float ang = Mathf.Abs(Math2D.SignedAngle(currentDir, dir)) * Mathf.Rad2Deg;
                if (ang > maxTurn) maxTurn = ang;
            }

            prev = p;
        }

        return maxTurn;
    }

    private Vector2 PointAhead(int segIdx, Vector2 fromPoint, float distance)
    {
        float left = Mathf.Max(0f, distance);
        Vector2 current = fromPoint;

        for (int i = segIdx + 1; i < path.Count; i++)
        {
            Vector2 next = new Vector2(path[i].x, path[i].z);
            float segLen = Vector2.Distance(current, next);
            if (segLen < 1e-8f)
            {
                current = next;
                continue;
            }

            if (left <= segLen)
            {
                float t = left / segLen;
                return Vector2.Lerp(current, next, t);
            }

            left -= segLen;
            current = next;
        }

        Vector3 last = path[path.Count - 1];
        return new Vector2(last.x, last.z);
    }
}
