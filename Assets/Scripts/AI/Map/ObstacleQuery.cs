using UnityEngine;

public class ObstacleQuery : MonoBehaviour
{
    [System.Serializable]
    public struct FootprintProbe
    {
        public Vector2 localOffset;
        [Min(0.05f)] public float radiusScale;

        public FootprintProbe(Vector2 localOffset, float radiusScale = 1f)
        {
            this.localOffset = localOffset;
            this.radiusScale = radiusScale;
        }
    }

    public LayerMask obstacleMask = ~0;
    public float checkY = 0.1f;

    public bool IsPoseCollisionFree(Pose2D pose, float inflateRadius, params FootprintProbe[] probes)
    {
        float radius = Mathf.Max(0.01f, inflateRadius);
        Vector3 center = new Vector3(pose.pos.x, checkY, pose.pos.y);
        if (Physics.CheckSphere(center, radius, obstacleMask, QueryTriggerInteraction.Ignore))
        {
            return false;
        }

        if (probes == null || probes.Length == 0)
        {
            return true;
        }

        for (int i = 0; i < probes.Length; i++)
        {
            FootprintProbe probe = probes[i];
            Vector2 rotated = Math2D.Rotate(probe.localOffset, pose.yawRad);
            Vector3 probeCenter = new Vector3(pose.pos.x + rotated.x, checkY, pose.pos.y + rotated.y);
            float probeRadius = radius * Mathf.Max(0.05f, probe.radiusScale);
            if (Physics.CheckSphere(probeCenter, probeRadius, obstacleMask, QueryTriggerInteraction.Ignore))
            {
                return false;
            }
        }

        return true;
    }
}
