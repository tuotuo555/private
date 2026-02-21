using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Planning
{
    public class GridAStarPlanner
    {
        public struct Config
        {
            public float CellSize;
            public bool AllowDiagonal;
            public int MaxIterations;
            public System.Func<Vector3, float, bool> PoseFree;
        }

        public struct PlanDebugInfo
        {
            public float cellSize;
            public int width;
            public int height;
            public int total;
            public int requestedStartX;
            public int requestedStartZ;
            public int requestedGoalX;
            public int requestedGoalZ;
            public int startX;
            public int startZ;
            public int goalX;
            public int goalZ;
            public bool startBlocked;
            public bool goalBlocked;
            public bool startFreeAnyHeading;
            public bool goalFreeAnyHeading;
            public bool startRelocated;
            public bool goalRelocated;
            public int startRelocationRadius;
            public int goalRelocationRadius;
            public bool startBlockedByInflated;
            public bool startBlockedByPose;
            public bool goalBlockedByInflated;
            public bool goalBlockedByPose;
            public int expansions;
            public int maxIterations;
            public bool hitIterationLimit;
            public int openCountAtEnd;
        }

        private struct Node
        {
            public int x;
            public int z;
            public float g;
            public float h;
            public int parentIndex;
            public bool closed;
            public bool opened;
        }

        public bool Plan(
            Vector3 start,
            Vector3 goal,
            Bounds bounds,
            IReadOnlyList<Bounds> inflatedObstacles,
            Config cfg,
            List<Vector3> outPath)
        {
            PlanDebugInfo debugInfo;
            return Plan(start, goal, bounds, inflatedObstacles, cfg, outPath, out debugInfo);
        }

        public bool Plan(
            Vector3 start,
            Vector3 goal,
            Bounds bounds,
            IReadOnlyList<Bounds> inflatedObstacles,
            Config cfg,
            List<Vector3> outPath,
            out PlanDebugInfo debugInfo)
        {
            outPath.Clear();
            float cell = Mathf.Max(0.5f, cfg.CellSize);
            int maxIterations = Mathf.Max(1000, cfg.MaxIterations);

            int width = Mathf.Max(2, Mathf.CeilToInt(bounds.size.x / cell));
            int height = Mathf.Max(2, Mathf.CeilToInt(bounds.size.z / cell));
            int total = width * height;

            debugInfo = new PlanDebugInfo
            {
                cellSize = cell,
                width = width,
                height = height,
                total = total,
                maxIterations = maxIterations
            };

            int ToIndex(int x, int z) => z * width + x;
            bool InRange(int x, int z) => x >= 0 && z >= 0 && x < width && z < height;

            Vector3 CellToWorld(int x, int z)
            {
                return new Vector3(
                    bounds.min.x + (x + 0.5f) * cell,
                    start.y,
                    bounds.min.z + (z + 0.5f) * cell);
            }

            bool IsBlocked(int x, int z, float theta)
            {
                bool byInflated;
                bool byPose;
                return IsBlockedDetailed(x, z, theta, out byInflated, out byPose);
            }

            bool IsBlockedDetailed(int x, int z, float theta, out bool blockedByInflated, out bool blockedByPose)
            {
                blockedByInflated = false;
                blockedByPose = false;
                Vector3 p = CellToWorld(x, z);
                for (int i = 0; i < inflatedObstacles.Count; i++)
                {
                    Bounds b = inflatedObstacles[i];
                    if (p.x >= b.min.x && p.x <= b.max.x && p.z >= b.min.z && p.z <= b.max.z)
                    {
                        blockedByInflated = true;
                        return true;
                    }
                }

                if (cfg.PoseFree != null && !cfg.PoseFree(p, theta))
                {
                    blockedByPose = true;
                    return true;
                }

                return false;
            }

            bool IsCellFreeAnyHeading(int x, int z)
            {
                Vector3 p = CellToWorld(x, z);
                for (int i = 0; i < inflatedObstacles.Count; i++)
                {
                    Bounds b = inflatedObstacles[i];
                    if (p.x >= b.min.x && p.x <= b.max.x && p.z >= b.min.z && p.z <= b.max.z)
                    {
                        return false;
                    }
                }

                if (cfg.PoseFree == null)
                {
                    return true;
                }

                const float step = 0.78539816339f; // 45 deg in radians
                for (int k = 0; k < 8; k++)
                {
                    if (cfg.PoseFree(p, k * step))
                    {
                        return true;
                    }
                }

                return false;
            }

            bool TryFindNearestFreeCell(int originX, int originZ, out int freeX, out int freeZ, out int ringRadius)
            {
                freeX = originX;
                freeZ = originZ;
                ringRadius = 0;

                if (IsCellFreeAnyHeading(originX, originZ))
                {
                    return true;
                }

                int maxRadius = Mathf.Max(width, height);
                bool found = false;
                float bestDistSq = float.PositiveInfinity;
                int bestX = freeX;
                int bestZ = freeZ;

                for (int r = 1; r <= maxRadius; r++)
                {
                    int minX = Mathf.Max(0, originX - r);
                    int maxX = Mathf.Min(width - 1, originX + r);
                    int minZ = Mathf.Max(0, originZ - r);
                    int maxZ = Mathf.Min(height - 1, originZ + r);

                    for (int x = minX; x <= maxX; x++)
                    {
                        ConsiderCandidate(x, minZ);
                        if (maxZ != minZ)
                        {
                            ConsiderCandidate(x, maxZ);
                        }
                    }

                    for (int z = minZ + 1; z <= maxZ - 1; z++)
                    {
                        ConsiderCandidate(minX, z);
                        if (maxX != minX)
                        {
                            ConsiderCandidate(maxX, z);
                        }
                    }

                    if (found)
                    {
                        freeX = bestX;
                        freeZ = bestZ;
                        ringRadius = r;
                        return true;
                    }
                }

                return false;

                void ConsiderCandidate(int cx, int cz)
                {
                    if (!IsCellFreeAnyHeading(cx, cz))
                    {
                        return;
                    }

                    float dx = cx - originX;
                    float dz = cz - originZ;
                    float distSq = dx * dx + dz * dz;
                    if (distSq < bestDistSq)
                    {
                        bestDistSq = distSq;
                        bestX = cx;
                        bestZ = cz;
                        found = true;
                    }
                }
            }

            int sx = Mathf.Clamp(Mathf.FloorToInt((start.x - bounds.min.x) / cell), 0, width - 1);
            int sz = Mathf.Clamp(Mathf.FloorToInt((start.z - bounds.min.z) / cell), 0, height - 1);
            int gx = Mathf.Clamp(Mathf.FloorToInt((goal.x - bounds.min.x) / cell), 0, width - 1);
            int gz = Mathf.Clamp(Mathf.FloorToInt((goal.z - bounds.min.z) / cell), 0, height - 1);

            debugInfo.requestedStartX = sx;
            debugInfo.requestedStartZ = sz;
            debugInfo.requestedGoalX = gx;
            debugInfo.requestedGoalZ = gz;

            Vector3 startToGoal = goal - start;
            float initialTheta = startToGoal.sqrMagnitude > 1e-6f ? Mathf.Atan2(startToGoal.x, startToGoal.z) : 0f;
            debugInfo.startBlocked = IsBlockedDetailed(sx, sz, initialTheta, out debugInfo.startBlockedByInflated, out debugInfo.startBlockedByPose);
            debugInfo.goalBlocked = IsBlockedDetailed(gx, gz, initialTheta, out debugInfo.goalBlockedByInflated, out debugInfo.goalBlockedByPose);
            debugInfo.startFreeAnyHeading = IsCellFreeAnyHeading(sx, sz);
            debugInfo.goalFreeAnyHeading = IsCellFreeAnyHeading(gx, gz);

            if (!debugInfo.startFreeAnyHeading)
            {
                if (!TryFindNearestFreeCell(sx, sz, out sx, out sz, out int startRadius))
                {
                    debugInfo.openCountAtEnd = 0;
                    return false;
                }

                debugInfo.startRelocated = true;
                debugInfo.startRelocationRadius = startRadius;
            }

            if (!debugInfo.goalFreeAnyHeading)
            {
                if (!TryFindNearestFreeCell(gx, gz, out gx, out gz, out int goalRadius))
                {
                    debugInfo.openCountAtEnd = 0;
                    return false;
                }

                debugInfo.goalRelocated = true;
                debugInfo.goalRelocationRadius = goalRadius;
            }

            debugInfo.startX = sx;
            debugInfo.startZ = sz;
            debugInfo.goalX = gx;
            debugInfo.goalZ = gz;

            if (debugInfo.startRelocated || debugInfo.goalRelocated)
            {
                if (debugInfo.startRelocated)
                {
                    debugInfo.startBlocked = IsBlockedDetailed(sx, sz, initialTheta, out debugInfo.startBlockedByInflated, out debugInfo.startBlockedByPose);
                }

                if (debugInfo.goalRelocated)
                {
                    debugInfo.goalBlocked = IsBlockedDetailed(gx, gz, initialTheta, out debugInfo.goalBlockedByInflated, out debugInfo.goalBlockedByPose);
                }
            }

            Node[] nodes = new Node[total];
            for (int z = 0; z < height; z++)
            {
                for (int x = 0; x < width; x++)
                {
                    int idx = ToIndex(x, z);
                    nodes[idx].x = x;
                    nodes[idx].z = z;
                    nodes[idx].g = float.PositiveInfinity;
                    nodes[idx].h = 0f;
                    nodes[idx].parentIndex = -1;
                }
            }

            List<int> open = new List<int>(1024);
            int sidx = ToIndex(sx, sz);
            nodes[sidx].g = 0f;
            nodes[sidx].h = Heuristic(sx, sz, gx, gz);
            nodes[sidx].opened = true;
            open.Add(sidx);

            int[] dx4 = { 1, -1, 0, 0 };
            int[] dz4 = { 0, 0, 1, -1 };
            int[] dx8 = { 1, -1, 0, 0, 1, 1, -1, -1 };
            int[] dz8 = { 0, 0, 1, -1, 1, -1, 1, -1 };

            int expansions = 0;
            int goalIndex = -1;
            while (open.Count > 0 && expansions < maxIterations)
            {
                int bestOpenPos = 0;
                float bestF = float.PositiveInfinity;
                for (int i = 0; i < open.Count; i++)
                {
                    int oi = open[i];
                    float f = nodes[oi].g + nodes[oi].h;
                    if (f < bestF)
                    {
                        bestF = f;
                        bestOpenPos = i;
                    }
                }

                int current = open[bestOpenPos];
                open[bestOpenPos] = open[open.Count - 1];
                open.RemoveAt(open.Count - 1);

                if (nodes[current].closed)
                {
                    continue;
                }

                nodes[current].closed = true;
                expansions++;

                int cx = nodes[current].x;
                int cz = nodes[current].z;
                if (cx == gx && cz == gz)
                {
                    goalIndex = current;
                    break;
                }

                if (cfg.AllowDiagonal)
                {
                    for (int k = 0; k < 8; k++)
                    {
                        int nx = cx + dx8[k];
                        int nz = cz + dz8[k];
                        float theta = Mathf.Atan2(nx - cx, nz - cz);
                        if (!InRange(nx, nz) || IsBlocked(nx, nz, theta))
                        {
                            continue;
                        }

                        if (k >= 4)
                        {
                            if (IsBlocked(cx, nz, theta) || IsBlocked(nx, cz, theta))
                            {
                                continue;
                            }
                        }

                        int ni = ToIndex(nx, nz);
                        if (nodes[ni].closed)
                        {
                            continue;
                        }

                        float step = (k < 4) ? 1f : 1.41421356f;
                        float newG = nodes[current].g + step;
                        if (newG >= nodes[ni].g)
                        {
                            continue;
                        }

                        nodes[ni].g = newG;
                        nodes[ni].h = Heuristic(nx, nz, gx, gz);
                        nodes[ni].parentIndex = current;
                        if (!nodes[ni].opened)
                        {
                            nodes[ni].opened = true;
                            open.Add(ni);
                        }
                    }
                }
                else
                {
                    for (int k = 0; k < 4; k++)
                    {
                        int nx = cx + dx4[k];
                        int nz = cz + dz4[k];
                        float theta = Mathf.Atan2(nx - cx, nz - cz);
                        if (!InRange(nx, nz) || IsBlocked(nx, nz, theta))
                        {
                            continue;
                        }

                        int ni = ToIndex(nx, nz);
                        if (nodes[ni].closed)
                        {
                            continue;
                        }

                        float newG = nodes[current].g + 1f;
                        if (newG >= nodes[ni].g)
                        {
                            continue;
                        }

                        nodes[ni].g = newG;
                        nodes[ni].h = Heuristic(nx, nz, gx, gz);
                        nodes[ni].parentIndex = current;
                        if (!nodes[ni].opened)
                        {
                            nodes[ni].opened = true;
                            open.Add(ni);
                        }
                    }
                }
            }

            if (goalIndex < 0)
            {
                debugInfo.expansions = expansions;
                debugInfo.openCountAtEnd = open.Count;
                debugInfo.hitIterationLimit = expansions >= maxIterations;
                return false;
            }

            List<Vector3> reversed = new List<Vector3>(512);
            int cursor = goalIndex;
            while (cursor >= 0)
            {
                reversed.Add(CellToWorld(nodes[cursor].x, nodes[cursor].z));
                cursor = nodes[cursor].parentIndex;
            }

            outPath.Add(start);
            for (int i = reversed.Count - 1; i >= 0; i--)
            {
                outPath.Add(reversed[i]);
            }
            if (!debugInfo.goalRelocated)
            {
                outPath.Add(goal);
            }
            debugInfo.expansions = expansions;
            debugInfo.openCountAtEnd = open.Count;
            debugInfo.hitIterationLimit = false;
            return outPath.Count >= 2;
        }

        private static float Heuristic(int x0, int z0, int x1, int z1)
        {
            return Mathf.Abs(x1 - x0) + Mathf.Abs(z1 - z0);
        }
    }
}
