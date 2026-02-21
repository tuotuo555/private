using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Control
{
    public class StanleyFollower
    {
        public void Configure(
            float gain,
            float eps,
            float wheelBase,
            float maxSteerDeg,
            float steerRatePerSec,
            float highSpeedSteerMin,
            float highSpeedSteerStart)
        {
        }

        public float ComputeSteer(
            IReadOnlyList<Vector3> path,
            Vector3 position,
            Vector3 forward,
            float speed,
            int currentIndex,
            float dt,
            out int closestIndex,
            out float crossTrackError)
        {
            closestIndex = currentIndex;
            crossTrackError = 0f;
            return 0f;
        }

        public void Reset()
        {
        }
    }
}
