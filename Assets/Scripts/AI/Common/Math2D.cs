using UnityEngine;

public static class Math2D
{
    public static float Distance2D(Vector2 a, Vector2 b)
    {
        return Vector2.Distance(a, b);
    }

    public static float Distance2D(Pose2D a, Pose2D b)
    {
        return Vector2.Distance(a.pos, b.pos);
    }

    public static float Clamp01(float v)
    {
        return Mathf.Clamp01(v);
    }

    public static Vector2 Normalize2(Vector2 v)
    {
        return (v.sqrMagnitude > 1e-8f) ? v.normalized : Vector2.zero;
    }

    public static float SignedAngle(Vector2 from, Vector2 to)
    {
        float cross = from.x * to.y - from.y * to.x;
        float dot = Vector2.Dot(from, to);
        return Mathf.Atan2(cross, dot);
    }

    public static float AngleDiffAbs(float a, float b)
    {
        return Mathf.Abs(Pose2D.WrapAngleRad(a - b));
    }

    public static Vector2 ProjectOn(Vector2 v, Vector2 on)
    {
        float den = on.sqrMagnitude;
        if (den < 1e-8f)
        {
            return Vector2.zero;
        }

        return on * (Vector2.Dot(v, on) / den);
    }

    public static Vector2 Rotate(Vector2 v, float yawRad)
    {
        float s = Mathf.Sin(yawRad);
        float c = Mathf.Cos(yawRad);
        return new Vector2(c * v.x - s * v.y, s * v.x + c * v.y);
    }

    public static Vector2 Forward(float yawRad)
    {
        return new Vector2(Mathf.Sin(yawRad), Mathf.Cos(yawRad));
    }

    public static Vector2 Right(float yawRad)
    {
        return new Vector2(Mathf.Cos(yawRad), -Mathf.Sin(yawRad));
    }
}
