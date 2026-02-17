using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SafetyFilter
{
    [Header("Prediction Horizon")]
    [Min(0.1f)] public float horizon = 2.5f;
    [Min(0.1f)] public float staticRolloutTime = 0.3f;
    [Min(0.5f)] public float dSafe = 2.5f;
    [Min(1f)] public float neighborRadius = 20f;

    [Header("Sampling")]
    [Min(1)] public int steerSamples = 5;
    [Min(1)] public int accelSamples = 5;
    [Min(0f)] public float steerDelta = 0.25f;
    [Min(0f)] public float accelDelta = 2f;

    [Header("Vehicle Limits")]
    [Min(0.1f)] public float wheelBase = 2.6f;
    [Min(1f)] public float maxSteerDeg = 25f;
    [Min(0.1f)] public float maxAccel = 3f;
    [Min(0.1f)] public float maxBrake = 6f;

    [Header("Objective")]
    public float wAccel = 1f;
    public float wSteer = 1f;
    public float wProgress = 0.1f;
    public float wTtcPenalty = 0.7f;

    [Header("Stability")]
    [Range(0f, 1f)] public float damping = 0.2f;
    public float wSteerHysteresis = 0.15f;
    public float wAccelHysteresis = 0.15f;

    public struct FilterInput
    {
        public Pose2D pose;
        public Vector2 velocity;
        public float speed;
        public float steerRef;
        public float accelRef;
    }

    public struct FilterOutput
    {
        public float steer;
        public float throttle;
        public float brake;
        public float accel;
        public bool usedEmergencyBrake;
    }

    private bool hasLast;
    private float lastSteer;
    private float lastAccel;

    public void ResetState()
    {
        hasLast = false;
        lastSteer = 0f;
        lastAccel = 0f;
    }

    public FilterOutput Filter(
        FilterInput input,
        IAgentState self,
        List<IAgentState> neighbors,
        ObstacleQuery obstacleQuery,
        float obstacleInflateRadius,
        ObstacleQuery.FootprintProbe[] probes,
        float deltaTime)
    {
        float dt = Mathf.Max(0.01f, deltaTime);
        float steerRef = Mathf.Clamp(input.steerRef, -1f, 1f);
        float accelRef = Mathf.Clamp(input.accelRef, -Mathf.Max(0.1f, maxBrake), Mathf.Max(0.1f, maxAccel));

        int sCount = Mathf.Max(1, steerSamples);
        int aCount = Mathf.Max(1, accelSamples);

        bool foundSafe = false;
        float bestCost = float.PositiveInfinity;
        float bestSteer = steerRef;
        float bestAccel = -Mathf.Max(0.1f, maxBrake);

        for (int si = 0; si < sCount; si++)
        {
            float steerCandidate = Mathf.Clamp(steerRef + SampleOffset(si, sCount, steerDelta), -1f, 1f);

            for (int ai = 0; ai < aCount; ai++)
            {
                float accelCandidate = Mathf.Clamp(
                    accelRef + SampleOffset(ai, aCount, accelDelta),
                    -Mathf.Max(0.1f, maxBrake),
                    Mathf.Max(0.1f, maxAccel));

                if (!CandidateSafe(
                    input,
                    self,
                    neighbors,
                    steerCandidate,
                    accelCandidate,
                    obstacleQuery,
                    obstacleInflateRadius,
                    probes,
                    dt,
                    out float ttcPenalty,
                    out float progress))
                {
                    continue;
                }

                float cost = wAccel * Sq(accelCandidate - accelRef)
                    + wSteer * Sq(steerCandidate - steerRef)
                    - wProgress * progress
                    + wTtcPenalty * ttcPenalty;

                if (hasLast)
                {
                    cost += wSteerHysteresis * Sq(steerCandidate - lastSteer);
                    cost += wAccelHysteresis * Sq(accelCandidate - lastAccel);
                }

                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestSteer = steerCandidate;
                    bestAccel = accelCandidate;
                    foundSafe = true;
                }
            }
        }

        bool emergency = false;
        if (!foundSafe)
        {
            emergency = true;
            bestSteer = steerRef;
            bestAccel = -Mathf.Max(0.1f, maxBrake);
        }

        if (hasLast)
        {
            float blend = 1f - Mathf.Clamp01(damping);
            bestSteer = Mathf.Lerp(lastSteer, bestSteer, blend);
            bestAccel = Mathf.Lerp(lastAccel, bestAccel, blend);
        }

        bestSteer = Mathf.Clamp(bestSteer, -1f, 1f);
        bestAccel = Mathf.Clamp(bestAccel, -Mathf.Max(0.1f, maxBrake), Mathf.Max(0.1f, maxAccel));

        hasLast = true;
        lastSteer = bestSteer;
        lastAccel = bestAccel;

        FilterOutput output = new FilterOutput
        {
            steer = bestSteer,
            accel = bestAccel,
            usedEmergencyBrake = emergency
        };

        if (bestAccel >= 0f)
        {
            output.throttle = Mathf.Clamp01(bestAccel / Mathf.Max(0.1f, maxAccel));
            output.brake = 0f;
        }
        else
        {
            output.throttle = 0f;
            output.brake = Mathf.Clamp01(-bestAccel / Mathf.Max(0.1f, maxBrake));
        }

        return output;
    }

    private bool CandidateSafe(
        FilterInput input,
        IAgentState self,
        List<IAgentState> neighbors,
        float steerCmd,
        float accelCmd,
        ObstacleQuery obstacleQuery,
        float obstacleInflateRadius,
        ObstacleQuery.FootprintProbe[] probes,
        float dt,
        out float ttcPenalty,
        out float progress)
    {
        ttcPenalty = 0f;

        float lookaheadT = Mathf.Min(Mathf.Max(0.1f, horizon), 0.75f);
        float speedPred = Mathf.Max(0f, input.speed + accelCmd * lookaheadT);

        float steerRad = steerCmd * Mathf.Max(1f, maxSteerDeg) * Mathf.Deg2Rad;
        float yawRate = speedPred * Mathf.Tan(steerRad) / Mathf.Max(0.1f, wheelBase);
        float yawPred = Pose2D.WrapAngleRad(input.pose.yawRad + yawRate * lookaheadT);

        Vector2 fwdPred = Math2D.Forward(yawPred);
        Vector2 selfVelPred = fwdPred * speedPred;

        progress = Vector2.Dot(selfVelPred, Math2D.Forward(input.pose.yawRad));

        float safe2 = Mathf.Max(0.1f, dSafe) * Mathf.Max(0.1f, dSafe);

        if (neighbors != null)
        {
            for (int i = 0; i < neighbors.Count; i++)
            {
                IAgentState other = neighbors[i];
                if (other == null || other == self || !other.IsAgentActive)
                {
                    continue;
                }

                Vector2 p = other.PlanarPosition - input.pose.pos;
                if (p.sqrMagnitude > neighborRadius * neighborRadius)
                {
                    continue;
                }

                Vector2 v = other.PlanarVelocity - selfVelPred;
                float vv = v.sqrMagnitude;

                float tStar = 0f;
                if (vv > 1e-6f)
                {
                    tStar = Mathf.Clamp(-Vector2.Dot(p, v) / vv, 0f, Mathf.Max(0.1f, horizon));
                }

                Vector2 atClosest = p + v * tStar;
                float d2 = atClosest.sqrMagnitude;
                if (d2 < safe2)
                {
                    return false;
                }

                if (tStar > 0f && tStar < horizon)
                {
                    float d = Mathf.Sqrt(Mathf.Max(1e-6f, d2));
                    if (d < dSafe * 2f)
                    {
                        float nearWeight = Mathf.Clamp01((2f * dSafe - d) / Mathf.Max(0.1f, 2f * dSafe));
                        ttcPenalty += nearWeight / Mathf.Max(0.1f, tStar);
                    }
                }
            }
        }

        if (obstacleQuery != null)
        {
            Pose2D simPose = input.pose;
            float simSpeed = input.speed;
            float simDt = Mathf.Clamp(dt, 0.02f, 0.1f);
            int steps = Mathf.Max(1, Mathf.CeilToInt(staticRolloutTime / simDt));

            for (int i = 0; i < steps; i++)
            {
                simSpeed = Mathf.Max(0f, simSpeed + accelCmd * simDt);

                float simSteerRad = steerCmd * Mathf.Max(1f, maxSteerDeg) * Mathf.Deg2Rad;
                float beta = Mathf.Tan(simSteerRad) / Mathf.Max(0.1f, wheelBase);
                float dYaw = simSpeed * beta * simDt;
                float yawMid = simPose.yawRad + 0.5f * dYaw;

                simPose.pos += Math2D.Forward(yawMid) * (simSpeed * simDt);
                simPose.yawRad = Pose2D.WrapAngleRad(simPose.yawRad + dYaw);

                if (!obstacleQuery.IsPoseCollisionFree(simPose, Mathf.Max(0.01f, obstacleInflateRadius), probes))
                {
                    return false;
                }
            }
        }

        return true;
    }

    private static float SampleOffset(int index, int count, float delta)
    {
        if (count <= 1 || delta <= 0f)
        {
            return 0f;
        }

        float t = (float)index / (count - 1);
        return Mathf.Lerp(-delta, delta, t);
    }

    private static float Sq(float x)
    {
        return x * x;
    }
}
