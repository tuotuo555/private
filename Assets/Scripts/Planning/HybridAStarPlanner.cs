using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Planning
{
    public class HybridAStarPlanner
    {
        public struct Config
        {
            public float cellSize;
            public int thetaBins;
            public float stepSize;
            public float maxSteerDeg;
            public int steerSamples;
            public bool allowReverse;
            public float wheelBase;
            public float wSteer;
            public float wSteerChange;
            public float wReverse;
            public float goalTolerance;
            public int maxIterations;
            public LayerMask obstacleMask;
            public float footprintRadius;
        }

        public List<Vector3> Plan(Vector3 start, Vector3 startDir, Vector3 goal, Bounds bounds, Config cfg)
        {
            return null;
        }
    }
}
