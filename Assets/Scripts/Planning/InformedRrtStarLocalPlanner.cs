using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Planning
{
    public class InformedRrtStarLocalPlanner
    {
        public struct Config
        {
            public float StepSize;
            public float GoalRadius;
            public float GoalSampleChance;
            public int MaxIterations;
            public float MaxSteeringAngleDeg;
            public float ParentingMaxDistance;
            public float TimeBudgetPerFrameMs;
            public float GridCellSize;
            public float CapsuleRadius;
            public float LocalRangeLimitFactor;
            public int ChaikinIterations;
            public int GoalImprovementIterations;
        }

        public struct Request
        {
            public Vector3 startPos;
            public Vector3 startDir;
            public Vector3 goalPos;
            public Vector3 goalDir;
            public Bounds sampleBounds;
            public LayerMask obstacleMask;
            public float resampleSpacing;
            public Config config;
        }

        public IEnumerator PlanCoroutine(Request request, Action<List<Vector3>> onDone)
        {
            List<Vector3> path = new List<Vector3> { request.startPos, request.goalPos };
            onDone?.Invoke(path);
            yield break;
        }
    }
}
