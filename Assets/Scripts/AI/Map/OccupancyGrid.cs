using UnityEngine;

[System.Serializable]
public class OccupancyGrid
{
    [Min(0.05f)] public float cellSize = 0.5f;
    public Vector2 origin;
    [Min(1)] public int width = 64;
    [Min(1)] public int height = 64;

    [SerializeField] private bool[] occupied;

    public int Width => width;
    public int Height => height;

    public void Resize(int newWidth, int newHeight)
    {
        width = Mathf.Max(1, newWidth);
        height = Mathf.Max(1, newHeight);
        occupied = new bool[width * height];
    }

    public Vector2Int WorldToCell(Vector2 world)
    {
        float cs = Mathf.Max(0.05f, cellSize);
        int x = Mathf.FloorToInt((world.x - origin.x) / cs);
        int y = Mathf.FloorToInt((world.y - origin.y) / cs);
        return new Vector2Int(x, y);
    }

    public Vector2 CellToWorld(int x, int y)
    {
        float cs = Mathf.Max(0.05f, cellSize);
        return origin + new Vector2((x + 0.5f) * cs, (y + 0.5f) * cs);
    }

    public bool InBounds(int x, int y)
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    public bool IsBlocked(int x, int y)
    {
        if (!InBounds(x, y)) return true;
        EnsureStorage();
        return occupied[IndexOf(x, y)];
    }

    public void SetBlocked(int x, int y, bool blocked)
    {
        if (!InBounds(x, y)) return;
        EnsureStorage();
        occupied[IndexOf(x, y)] = blocked;
    }

    public void Build(LayerMask obstacleMask, float checkY, float radius, int inflateCells = 0)
    {
        EnsureStorage(true);
        float r = Mathf.Max(0.01f, radius);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Vector2 world = CellToWorld(x, y);
                Vector3 center = new Vector3(world.x, checkY, world.y);
                occupied[IndexOf(x, y)] = Physics.CheckSphere(center, r, obstacleMask, QueryTriggerInteraction.Ignore);
            }
        }

        if (inflateCells <= 0)
        {
            return;
        }

        bool[] inflated = new bool[occupied.Length];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (!occupied[IndexOf(x, y)]) continue;

                for (int dy = -inflateCells; dy <= inflateCells; dy++)
                {
                    for (int dx = -inflateCells; dx <= inflateCells; dx++)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (!InBounds(nx, ny)) continue;
                        inflated[IndexOf(nx, ny)] = true;
                    }
                }
            }
        }

        occupied = inflated;
    }

    private int IndexOf(int x, int y)
    {
        return x + width * y;
    }

    private void EnsureStorage(bool reset = false)
    {
        int expected = Mathf.Max(1, width * height);
        if (reset || occupied == null || occupied.Length != expected)
        {
            occupied = new bool[expected];
        }
    }
}
