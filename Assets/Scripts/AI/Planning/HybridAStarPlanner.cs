using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class HybridAStarPlanner
{
    [Header("Discretization")]
    [Min(0.1f)] public float cellSize = 0.5f;
    [Min(8)] public int thetaBins = 72;
    [Min(0.1f)] public float stepSize = 0.9f;
    [Min(100)] public int maxIterations = 50000;

    [Header("Vehicle")]
    public float maxSteerDeg = 25f;
    [Min(2)] public int steerSamples = 7;
    public bool allowReverse = true;
    [Min(0.1f)] public float wheelBase = 2.6f;

    [Header("Costs")]
    public float wSteer = 0.08f;
    public float wSteerChange = 0.2f;
    public float wReverse = 1.5f;

    [Header("Heuristic")]
    public float hHeadingWeight = 1f;

    [Header("Goal")]
    [Min(0.1f)] public float goalTolerance = 1.0f;
    [Min(0f)] public float goalHeadingToleranceDeg = 20f;
    public bool requireGoalHeading = true;

    [Header("Shot")]
    public float shotDistance = 6f;

    [Header("Map Bounds")]
    [Min(1f)] public float mapPadding = 20f;
    [Min(8)] public int minMapCells = 64;
    [Min(32)] public int maxMapCellsPerAxis = 1024;

    [Header("Collision")]
    public ObstacleQuery obstacleQuery;
    [Min(0.05f)] public float inflateRadius = 0.6f;
    public ObstacleQuery.FootprintProbe[] footprintProbes;

    [Header("Output")]
    [Min(0.1f)] public float outputSpacing = 0.5f;

    private struct Node
    {
        public Pose2D pose;
        public float g;
        public float f;
        public int parentIndex;
        public float steerRad;
        public int dir;
        public int key;
    }

    private readonly List<Node> nodes = new List<Node>(4096);
    private readonly PriorityQueue<int> open = new PriorityQueue<int>(4096);
    private readonly Dictionary<int, float> bestGByKey = new Dictionary<int, float>(4096);
    private readonly HashSet<int> closedKeys = new HashSet<int>();
    private readonly List<Pose2D> shotBuffer = new List<Pose2D>(128);
    private readonly List<Pose2D> resampleBuffer = new List<Pose2D>(2048);

    private Vector2 searchOrigin;
    private int searchWidth;
    private int searchHeight;

    public bool Plan(Pose2D start, Pose2D goal, List<Pose2D> outputPath)
    {
        if (outputPath == null)
        {
            return false;
        }

        outputPath.Clear();

        if (obstacleQuery == null)
        {
            return false;
        }

        ConfigureSearchBounds(start, goal);

        if (!TryEncodeKey(start.pos, start.yawRad, out int startKey))
        {
            return false;
        }

        if (!IsPoseValid(start) || !IsPoseValid(goal))
        {
            return false;
        }

        ClearWorkingState();

        Node startNode = new Node
        {
            pose = start,
            g = 0f,
            f = Heuristic(start, goal),
            parentIndex = -1,
            steerRad = 0f,
            dir = 1,
            key = startKey
        };

        nodes.Add(startNode);
        open.Push(0, startNode.f);
        bestGByKey[startKey] = 0f;

        float headingToleranceRad = Mathf.Max(0f, goalHeadingToleranceDeg) * Mathf.Deg2Rad;
        int iterations = 0;

        while (open.Count > 0 && iterations < Mathf.Max(100, maxIterations))
        {
            iterations++;

            if (!open.TryPopMin(out int currentIndex, out _))
            {
                break;
            }

            Node current = nodes[currentIndex];

            if (closedKeys.Contains(current.key))
            {
                continue;
            }

            if (!bestGByKey.TryGetValue(current.key, out float bestG) || current.g > bestG + 1e-4f)
            {
                continue;
            }

            if (IsGoal(current.pose, goal, headingToleranceRad))
            {
                BuildPath(currentIndex, outputPath);
                ResamplePath(outputPath, Mathf.Max(0.1f, outputSpacing));
                return true;
            }

            if (shotDistance > 0f && Math2D.Distance2D(current.pose, goal) <= shotDistance)
            {
                if (TryShot(current.pose, goal, shotBuffer))
                {
                    BuildPath(currentIndex, outputPath);
                    for (int i = 1; i < shotBuffer.Count; i++)
                    {
                        outputPath.Add(shotBuffer[i]);
                    }

                    ResamplePath(outputPath, Mathf.Max(0.1f, outputSpacing));
                    return true;
                }
            }

            closedKeys.Add(current.key);
            ExpandNode(currentIndex, current, goal);
        }

        return false;
    }

    private void ExpandNode(int currentIndex, Node current, Pose2D goal)
    {
        float maxSteerRad = Mathf.Max(0f, maxSteerDeg) * Mathf.Deg2Rad;
        int steerCount = Mathf.Max(2, steerSamples);

        for (int si = 0; si < steerCount; si++)
        {
            float t = (steerCount == 1) ? 0.5f : (float)si / (steerCount - 1);
            float steer = Mathf.Lerp(-maxSteerRad, maxSteerRad, t);

            ExpandPrimitive(currentIndex, current, goal, steer, 1);
            if (allowReverse)
            {
                ExpandPrimitive(currentIndex, current, goal, steer, -1);
            }
        }
    }

    private void ExpandPrimitive(int currentIndex, Node current, Pose2D goal, float steerRad, int dir)
    {
        if (!Propagate(current.pose, steerRad, dir, out Pose2D nextPose))
        {
            return;
        }

        if (!TryEncodeKey(nextPose.pos, nextPose.yawRad, out int nextKey))
        {
            return;
        }

        if (closedKeys.Contains(nextKey))
        {
            return;
        }

        float stepCost = Mathf.Abs(stepSize)
            + wSteer * Mathf.Abs(steerRad)
            + wSteerChange * Mathf.Abs(steerRad - current.steerRad)
            + (dir < 0 ? wReverse : 0f);

        float newG = current.g + stepCost;

        if (bestGByKey.TryGetValue(nextKey, out float oldG) && newG >= oldG - 1e-4f)
        {
            return;
        }

        bestGByKey[nextKey] = newG;

        Node next = new Node
        {
            pose = nextPose,
            g = newG,
            f = newG + Heuristic(nextPose, goal),
            parentIndex = currentIndex,
            steerRad = steerRad,
            dir = dir,
            key = nextKey
        };

        int nextIndex = nodes.Count;
        nodes.Add(next);
        open.Push(nextIndex, next.f);
    }

    private bool Propagate(Pose2D start, float steerRad, int dir, out Pose2D result)
    {
        float travel = Mathf.Abs(stepSize) * Mathf.Sign(dir);
        int subSteps = Mathf.Max(2, Mathf.CeilToInt(Mathf.Abs(travel) / Mathf.Max(0.15f, cellSize * 0.5f)));
        float ds = travel / subSteps;

        Pose2D pose = start;
        float beta = Mathf.Tan(steerRad) / Mathf.Max(0.01f, wheelBase);

        for (int i = 0; i < subSteps; i++)
        {
            float dYaw = beta * ds;
            float yawMid = pose.yawRad + 0.5f * dYaw;
            Vector2 step = Math2D.Forward(yawMid) * ds;

            pose.pos += step;
            pose.yawRad = Pose2D.WrapAngleRad(pose.yawRad + dYaw);

            if (!IsInsideBounds(pose.pos))
            {
                result = pose;
                return false;
            }

            if (!obstacleQuery.IsPoseCollisionFree(pose, Mathf.Max(0.01f, inflateRadius), footprintProbes))
            {
                result = pose;
                return false;
            }
        }

        result = pose;
        return true;
    }

    private bool IsGoal(Pose2D pose, Pose2D goal, float headingToleranceRad)
    {
        if (Math2D.Distance2D(pose, goal) > Mathf.Max(0.1f, goalTolerance))
        {
            return false;
        }

        if (!requireGoalHeading)
        {
            return true;
        }

        return Math2D.AngleDiffAbs(pose.yawRad, goal.yawRad) <= headingToleranceRad;
    }

    private float Heuristic(Pose2D pose, Pose2D goal)
    {
        float dist = Math2D.Distance2D(pose, goal);
        float heading = Math2D.AngleDiffAbs(pose.yawRad, goal.yawRad);
        return dist + hHeadingWeight * heading;
    }

    private bool TryShot(Pose2D from, Pose2D goal, List<Pose2D> outShot)
    {
        outShot.Clear();
        outShot.Add(from);

        float d = Vector2.Distance(from.pos, goal.pos);
        int samples = Mathf.Max(2, Mathf.CeilToInt(d / Mathf.Max(0.2f, cellSize * 0.5f)));
        float yawDelta = Pose2D.WrapAngleRad(goal.yawRad - from.yawRad);

        for (int i = 1; i <= samples; i++)
        {
            float t = (float)i / samples;
            Pose2D p = new Pose2D(Vector2.Lerp(from.pos, goal.pos, t), from.yawRad + yawDelta * t);

            if (!IsInsideBounds(p.pos) || !IsPoseValid(p))
            {
                outShot.Clear();
                return false;
            }

            outShot.Add(p);
        }

        return true;
    }

    private void BuildPath(int endNodeIndex, List<Pose2D> outputPath)
    {
        outputPath.Clear();
        int cur = endNodeIndex;

        for (int guard = 0; guard < 1000000 && cur >= 0; guard++)
        {
            Node n = nodes[cur];
            outputPath.Add(n.pose);
            cur = n.parentIndex;
        }

        outputPath.Reverse();
    }

    private void ResamplePath(List<Pose2D> path, float spacing)
    {
        if (path.Count < 2)
        {
            return;
        }

        float s = Mathf.Max(0.1f, spacing);
        resampleBuffer.Clear();
        resampleBuffer.Add(path[0]);

        float carry = s;

        for (int i = 1; i < path.Count; i++)
        {
            Pose2D a = path[i - 1];
            Pose2D b = path[i];
            Vector2 seg = b.pos - a.pos;
            float len = seg.magnitude;

            if (len < 1e-5f)
            {
                continue;
            }

            Vector2 dir = seg / len;
            float yawDelta = Pose2D.WrapAngleRad(b.yawRad - a.yawRad);
            float along = 0f;

            while (along + carry <= len)
            {
                along += carry;
                float t = along / len;
                Pose2D sample = new Pose2D(a.pos + dir * along, a.yawRad + yawDelta * t);
                resampleBuffer.Add(sample);
                carry = s;
            }

            carry -= (len - along);
            if (carry <= 1e-4f)
            {
                carry = s;
            }
        }

        Pose2D last = path[path.Count - 1];
        if (resampleBuffer.Count == 0 || Vector2.Distance(resampleBuffer[resampleBuffer.Count - 1].pos, last.pos) > 0.05f)
        {
            resampleBuffer.Add(last);
        }
        else
        {
            resampleBuffer[resampleBuffer.Count - 1] = last;
        }

        path.Clear();
        path.AddRange(resampleBuffer);
    }

    private bool IsPoseValid(Pose2D pose)
    {
        if (!IsInsideBounds(pose.pos))
        {
            return false;
        }

        return obstacleQuery.IsPoseCollisionFree(pose, Mathf.Max(0.01f, inflateRadius), footprintProbes);
    }

    private bool IsInsideBounds(Vector2 pos)
    {
        float cs = Mathf.Max(0.1f, cellSize);
        float maxX = searchOrigin.x + searchWidth * cs;
        float maxZ = searchOrigin.y + searchHeight * cs;

        return pos.x >= searchOrigin.x && pos.x < maxX && pos.y >= searchOrigin.y && pos.y < maxZ;
    }

    private bool TryEncodeKey(Vector2 pos, float yawRad, out int key)
    {
        key = -1;
        if (!IsInsideBounds(pos))
        {
            return false;
        }

        float cs = Mathf.Max(0.1f, cellSize);
        int ix = Mathf.FloorToInt((pos.x - searchOrigin.x) / cs);
        int iz = Mathf.FloorToInt((pos.y - searchOrigin.y) / cs);

        if (ix < 0 || iz < 0 || ix >= searchWidth || iz >= searchHeight)
        {
            return false;
        }

        float yaw = Pose2D.WrapAngleRad(yawRad);
        if (yaw < 0f)
        {
            yaw += Mathf.PI * 2f;
        }

        int bins = Mathf.Max(8, thetaBins);
        int itheta = Mathf.Clamp(Mathf.FloorToInt((yaw / (2f * Mathf.PI)) * bins), 0, bins - 1);
        key = ix + searchWidth * (iz + searchHeight * itheta);
        return true;
    }

    private void ConfigureSearchBounds(Pose2D start, Pose2D goal)
    {
        float cs = Mathf.Max(0.1f, cellSize);
        float pad = Mathf.Max(cs * 4f, mapPadding);

        float minX = Mathf.Min(start.pos.x, goal.pos.x) - pad;
        float maxX = Mathf.Max(start.pos.x, goal.pos.x) + pad;
        float minZ = Mathf.Min(start.pos.y, goal.pos.y) - pad;
        float maxZ = Mathf.Max(start.pos.y, goal.pos.y) + pad;

        searchOrigin = new Vector2(minX, minZ);
        searchWidth = Mathf.Clamp(Mathf.CeilToInt((maxX - minX) / cs), Mathf.Max(8, minMapCells), Mathf.Max(minMapCells, maxMapCellsPerAxis));
        searchHeight = Mathf.Clamp(Mathf.CeilToInt((maxZ - minZ) / cs), Mathf.Max(8, minMapCells), Mathf.Max(minMapCells, maxMapCellsPerAxis));
    }

    private void ClearWorkingState()
    {
        nodes.Clear();
        open.Clear();
        bestGByKey.Clear();
        closedKeys.Clear();
        shotBuffer.Clear();
    }
}
