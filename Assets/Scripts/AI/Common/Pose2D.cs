using UnityEngine;

[System.Serializable]
public struct Pose2D
{
    public Vector2 pos;
    public float yawRad;

    public Pose2D(Vector2 pos, float yawRad)
    {
        this.pos = pos;
        this.yawRad = WrapAngleRad(yawRad);
    }

    public static Pose2D FromWorld(Vector3 worldPos)
    {
        return new Pose2D(new Vector2(worldPos.x, worldPos.z), 0f);
    }

    public static Pose2D FromWorld(Transform worldTransform)
    {
        Vector3 forward = worldTransform.forward;
        float yaw = Mathf.Atan2(forward.x, forward.z);
        return new Pose2D(new Vector2(worldTransform.position.x, worldTransform.position.z), yaw);
    }

    public Vector3 ToWorld(float y = 0f)
    {
        return new Vector3(pos.x, y, pos.y);
    }

    public static Vector3 ToWorldXZ(Vector2 p, float y = 0f)
    {
        return new Vector3(p.x, y, p.y);
    }

    public static float WrapAngleRad(float angle)
    {
        const float twoPi = Mathf.PI * 2f;
        angle %= twoPi;
        if (angle >= Mathf.PI) angle -= twoPi;
        if (angle < -Mathf.PI) angle += twoPi;
        return angle;
    }
}
