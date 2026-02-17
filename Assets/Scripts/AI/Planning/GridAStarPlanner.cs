using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class GridAStarPlanner
{
    public OccupancyGrid grid;
    public bool allowDiagonal = true;
    public bool pruneLineOfSight = true;
    [Min(100)] public int maxIterations = 200000;

    private struct Node
    {
        public int x;
        public int y;
        public float g;
        public float f;
        public int parentIndex;
        public int cellKey;
    }

    private readonly List<Node> nodes = new List<Node>(2048);
    private readonly PriorityQueue<int> open = new PriorityQueue<int>(2048);
    private readonly Dictionary<int, float> bestG = new Dictionary<int, float>(2048);
    private readonly HashSet<int> closed = new HashSet<int>();

    public bool Plan(Vector2 startWorld, Vector2 goalWorld, List<Vector2> outPath)
    {
        if (outPath == null || grid == null)
        {
            return false;
        }

        outPath.Clear();

        Vector2Int s = grid.WorldToCell(startWorld);
        Vector2Int g = grid.WorldToCell(goalWorld);

        if (!grid.InBounds(s.x, s.y) || !grid.InBounds(g.x, g.y)) return false;
        if (grid.IsBlocked(s.x, s.y) || grid.IsBlocked(g.x, g.y)) return false;

        nodes.Clear();
        open.Clear();
        bestG.Clear();
        closed.Clear();

        int startKey = CellKey(s.x, s.y);
        Node startNode = new Node
        {
            x = s.x,
            y = s.y,
            g = 0f,
            f = Heuristic(s.x, s.y, g.x, g.y),
            parentIndex = -1,
            cellKey = startKey
        };

        nodes.Add(startNode);
        open.Push(0, startNode.f);
        bestG[startKey] = 0f;

        int iterations = 0;
        while (open.Count > 0 && iterations < Mathf.Max(100, maxIterations))
        {
            iterations++;

            if (!open.TryPopMin(out int currentIdx, out _)) break;
            Node current = nodes[currentIdx];

            if (closed.Contains(current.cellKey)) continue;
            if (!bestG.TryGetValue(current.cellKey, out float bg) || current.g > bg + 1e-4f) continue;

            if (current.x == g.x && current.y == g.y)
            {
                Reconstruct(currentIdx, outPath);
                if (pruneLineOfSight)
                {
                    PrunePath(outPath);
                }

                return true;
            }

            closed.Add(current.cellKey);
            Expand(currentIdx, current, g.x, g.y);
        }

        return false;
    }

    private void Expand(int currentIdx, Node current, int gx, int gy)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dy == 0) continue;
                if (!allowDiagonal && Mathf.Abs(dx) + Mathf.Abs(dy) > 1) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                if (!grid.InBounds(nx, ny) || grid.IsBlocked(nx, ny)) continue;

                float step = (dx != 0 && dy != 0) ? 1.41421356f : 1f;
                float ng = current.g + step * grid.cellSize;
                int nKey = CellKey(nx, ny);

                if (bestG.TryGetValue(nKey, out float old) && ng >= old - 1e-4f) continue;

                bestG[nKey] = ng;
                Node n = new Node
                {
                    x = nx,
                    y = ny,
                    g = ng,
                    f = ng + Heuristic(nx, ny, gx, gy),
                    parentIndex = currentIdx,
                    cellKey = nKey
                };

                int nIdx = nodes.Count;
                nodes.Add(n);
                open.Push(nIdx, n.f);
            }
        }
    }

    private void Reconstruct(int endNodeIdx, List<Vector2> outPath)
    {
        outPath.Clear();
        int idx = endNodeIdx;
        for (int guard = 0; idx >= 0 && guard < 1000000; guard++)
        {
            Node n = nodes[idx];
            outPath.Add(grid.CellToWorld(n.x, n.y));
            idx = n.parentIndex;
        }

        outPath.Reverse();
    }

    private void PrunePath(List<Vector2> path)
    {
        if (path.Count < 3) return;

        List<Vector2> pruned = new List<Vector2>(path.Count);
        pruned.Add(path[0]);

        int anchor = 0;
        int test = 2;

        while (test < path.Count)
        {
            Vector2Int a = grid.WorldToCell(path[anchor]);
            Vector2Int c = grid.WorldToCell(path[test]);
            if (!HasLineOfSight(a, c))
            {
                pruned.Add(path[test - 1]);
                anchor = test - 1;
            }

            test++;
        }

        pruned.Add(path[path.Count - 1]);
        path.Clear();
        path.AddRange(pruned);
    }

    private bool HasLineOfSight(Vector2Int a, Vector2Int b)
    {
        int x0 = a.x;
        int y0 = a.y;
        int x1 = b.x;
        int y1 = b.y;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (!grid.InBounds(x0, y0) || grid.IsBlocked(x0, y0)) return false;
            if (x0 == x1 && y0 == y1) return true;

            int e2 = err << 1;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    private float Heuristic(int x, int y, int gx, int gy)
    {
        float dx = x - gx;
        float dy = y - gy;
        return Mathf.Sqrt(dx * dx + dy * dy) * grid.cellSize;
    }

    private int CellKey(int x, int y)
    {
        return x + grid.Width * y;
    }
}
