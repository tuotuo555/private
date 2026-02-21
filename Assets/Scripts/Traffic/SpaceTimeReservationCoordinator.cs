using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

namespace Scripts.Traffic
{
    public class SpaceTimeReservationCoordinator : MonoBehaviour
    {
        private struct Disc
        {
            public Vector2 posXZ;
            public float radius;
            public int agentId;

            public Disc(Vector2 position, float r, int id)
            {
                posXZ = position;
                radius = r;
                agentId = id;
            }
        }

        private sealed class AgentTrajectory
        {
            public AIP1TrafficCar agent;
            public int agentId;
            public List<Vector3> pathWorld = new List<Vector3>();
            public float[] arcLengths;
            public float[] speedProfile;
            public float[] timestamps;
            public float releaseTime;
            public float nominalSpeed;
            public int globalIndex;
            public string groupName;
            public int indexWithinGroup;
            public int batchIndex;
        }

        public struct GateDecision
        {
            public float cap;
            public float releaseTime;
            public float scheduleTime;
            public float actualProgressMeters;
            public float allowedProgressMeters;
            public float progressErrorMeters;
            public int clampedPathIndex;
            public string reason;
        }

        private struct ScheduleSolveStats
        {
            public bool fallback;
            public int waitInsertions;
            public float totalWaitSec;
            public int checkedBins;
            public int conflictHits;
        }

        private static SpaceTimeReservationCoordinator _instance;

        [Header("Space Planning Reservation (Inspector)")]
        [Tooltip("Master switch for space-time reservation planning and runtime gating.")]
        public bool enableSpacePlanningReservation = true;
        [Tooltip("If enabled, release/member order uses 1,3,5.. then 2,4,6.. (one-based indexing).")]
        public bool oddThenEvenOrdering = true;

        [Header("Reservation")]
        [Tooltip("Reservation time-bin size.")]
        public float dtRes = 0.3f;
        [Tooltip("Disc radius reserved for each car in XZ.")]
        public float reserveRadius = 2.2f;
        [Tooltip("Forward reservation horizon in seconds.")]
        public float horizonSec = 25f;
        [Tooltip("Max accumulated wait inserted into a single schedule.")]
        public float maxTotalWaitSec = 12f;
        [Tooltip("How early is considered too early for a waypoint schedule time.")]
        public float gateTolerance = 0.35f;
        [Tooltip("Ahead distance threshold (m) above which runtime cap drops to crawl.")]
        public float gateAheadCrawlMeters = 0.05f;
        [Tooltip("Ahead distance threshold (m) above which runtime cap is forced to stop.")]
        public float gateAheadStopMeters = 0.5f;
        [Tooltip("Speed cap used while slightly ahead of allowed progress.")]
        public float gateAheadCrawlSpeed = 0.8f;
        [Tooltip("Temporal guard for allowed-progress sampling (positive is more conservative).")]
        public float gateProgressTimeGuardSec = 0.15f;

        [Header("Batch Release Ordering (Group-Aware)")]
        public bool enableBatchRelease = true;
        [Min(1f)] public int batchSize = 4;
        [FormerlySerializedAs("batchGapSec")]
        [Min(0f)] public float batchIntervalSec = 15f;
        public bool enableWithinBatchMicroStagger = true;
        [Min(0f)] public float withinBatchStaggerSec = 0.2f;
        [Tooltip("Max startup wait before forcing a one-time schedule build.")]
        [Min(0f)] public float forceBuildAfterSec = 1f;
        public bool logReleaseSchedule = true;

        [Header("Fallback")]
        [Tooltip("Used when a trajectory has no speed profile.")]
        public float defaultNominalSpeed = 8f;

        [Header("Debug")]
        [Tooltip("Enable extensive logs for reservation build and runtime gating.")]
        public bool debugVerboseLogs = false;
        [Tooltip("Periodic debug print interval to avoid spam.")]
        [Min(0.02f)] public float debugPrintIntervalSec = 0.2f;
        public bool debugLogGateDecisions = true;
        public bool debugLogScheduleBuild = true;
        public bool debugLogConflictEvents = true;
        public bool debugLogBatchComposition = true;
        public bool debugLogReservationCommits = true;
        [Tooltip("Run a one-time planned trajectory collision validation after schedules are built.")]
        public bool debugValidatePlannedCollisionFree = true;
        [Tooltip("Write debug logs to txt under Application.persistentDataPath.")]
        public bool debugWriteToFile = true;
        [Tooltip("Mirror debug logs to Unity Console.")]
        public bool debugEchoToConsole = false;
        [Tooltip("Clear debug file at session start/reinitialize.")]
        public bool debugClearFileOnAwake = true;
        public string debugFileName = "SpaceTimeReservationDebug.txt";
        [SerializeField] private string debugLogFilePathRuntime = string.Empty;

        public float scenarioStartTime { get; private set; }

        private readonly Dictionary<int, AgentTrajectory> _trajectoryByAgent = new Dictionary<int, AgentTrajectory>();
        private readonly Dictionary<int, List<Disc>> _occupied = new Dictionary<int, List<Disc>>();
        private readonly Dictionary<int, float> _nextGateDebugTimeByAgent = new Dictionary<int, float>();
        private bool _schedulesBuilt;
        private float _nextCoordinatorDebugTime;
        private bool _debugFileInitialized;
        private bool _pendingClearDebugFile;
        private string _debugFileNameResolved = string.Empty;
        private string _debugLogFilePath = string.Empty;

        public string DebugLogFilePath => _debugLogFilePath;

        public static SpaceTimeReservationCoordinator Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindFirstObjectByType<SpaceTimeReservationCoordinator>();
                    if (_instance == null)
                    {
                        GameObject go = new GameObject("SpaceTimeReservationCoordinator");
                        _instance = go.AddComponent<SpaceTimeReservationCoordinator>();
                    }
                }

                return _instance;
            }
        }

        public static bool HasInstance => _instance != null;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }

            _instance = this;
            scenarioStartTime = Time.timeSinceLevelLoad;
            if (debugVerboseLogs && debugLogScheduleBuild)
            {
                DebugOutput($"[SpaceTimeReservation][AWAKE] scenarioStartTime={scenarioStartTime:0.###}");
            }

            EnsureDebugFileReady();
        }

        public void ReinitializeDebugLogFile(bool clearExisting)
        {
            _pendingClearDebugFile = clearExisting;
            _debugFileInitialized = false;
            EnsureDebugFileReady();
        }

        public void RegisterTrajectory(
            AIP1TrafficCar agent,
            IReadOnlyList<Vector3> pathWorld,
            float[] speedProfile,
            float nominalSpeed,
            int globalIndex,
            string groupName,
            int indexWithinGroup)
        {
            if (agent == null || pathWorld == null || pathWorld.Count < 2)
            {
                return;
            }

            int agentId = agent.GetInstanceID();
            if (!_trajectoryByAgent.TryGetValue(agentId, out AgentTrajectory record))
            {
                record = new AgentTrajectory
                {
                    agent = agent,
                    agentId = agentId
                };
                _trajectoryByAgent.Add(agentId, record);
            }

            record.agent = agent;
            record.globalIndex = globalIndex;
            record.groupName = string.IsNullOrEmpty(groupName) ? "Ungrouped" : groupName;
            record.indexWithinGroup = Mathf.Max(0, indexWithinGroup);
            record.nominalSpeed = Mathf.Max(1f, nominalSpeed > 0f ? nominalSpeed : defaultNominalSpeed);
            record.pathWorld.Clear();
            for (int i = 0; i < pathWorld.Count; i++)
            {
                record.pathWorld.Add(pathWorld[i]);
            }

            record.arcLengths = BuildArcLengths(record.pathWorld);
            record.speedProfile = CopySpeedProfile(speedProfile, record.pathWorld.Count);
            record.timestamps = null;
            record.releaseTime = scenarioStartTime;
            record.batchIndex = 0;
            _schedulesBuilt = false;
            if (debugVerboseLogs && debugLogScheduleBuild)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][REGISTER] agent={agentId} group={record.groupName} idxWithinGroup={record.indexWithinGroup} " +
                    $"globalIdx={record.globalIndex} pathPts={record.pathWorld.Count} speedProfileLen={(record.speedProfile != null ? record.speedProfile.Length : 0)} " +
                    $"nominalSpeed={record.nominalSpeed:0.###}");
            }
        }

        public bool TryGetPlannedTrajectorySnapshot(
            AIP1TrafficCar agent,
            out Vector3[] pathWorld,
            out float[] scheduleTimes,
            out float releaseTime,
            out int batchIndex)
        {
            pathWorld = null;
            scheduleTimes = null;
            releaseTime = float.NaN;
            batchIndex = -1;
            if (agent == null)
            {
                return false;
            }

            TryBuildSchedules(Time.timeSinceLevelLoad);
            if (!_trajectoryByAgent.TryGetValue(agent.GetInstanceID(), out AgentTrajectory record))
            {
                return false;
            }

            if (record.pathWorld == null || record.pathWorld.Count == 0 ||
                record.timestamps == null || record.timestamps.Length == 0)
            {
                return false;
            }

            pathWorld = record.pathWorld.ToArray();
            scheduleTimes = new float[record.timestamps.Length];
            Array.Copy(record.timestamps, scheduleTimes, record.timestamps.Length);
            releaseTime = record.releaseTime;
            batchIndex = record.batchIndex;
            return true;
        }

        public bool TrySamplePlannedPositionAtTime(
            AIP1TrafficCar agent,
            float timeSample,
            out Vector3 plannedPosition,
            out int segmentIndex)
        {
            bool ok = TrySamplePlannedStateAtTime(
                agent,
                timeSample,
                out plannedPosition,
                out _,
                out segmentIndex,
                out _,
                out _);
            return ok;
        }

        public bool TrySamplePlannedStateAtTime(
            AIP1TrafficCar agent,
            float timeSample,
            out Vector3 plannedPosition,
            out float plannedSpeed,
            out int segmentIndex,
            out float segmentStartTime,
            out float segmentEndTime)
        {
            plannedPosition = Vector3.zero;
            plannedSpeed = 0f;
            segmentIndex = 0;
            segmentStartTime = float.NaN;
            segmentEndTime = float.NaN;
            if (agent == null)
            {
                return false;
            }

            TryBuildSchedules(timeSample);
            if (!_trajectoryByAgent.TryGetValue(agent.GetInstanceID(), out AgentTrajectory record))
            {
                return false;
            }

            if (record.pathWorld == null || record.pathWorld.Count == 0 ||
                record.timestamps == null || record.timestamps.Length == 0)
            {
                return false;
            }

            int pointCount = Mathf.Min(record.pathWorld.Count, record.timestamps.Length);
            if (pointCount <= 0)
            {
                return false;
            }

            int maxSegment = Mathf.Max(0, pointCount - 2);
            segmentIndex = Mathf.Clamp(FindSegmentIndexForTime(record.timestamps, timeSample), 0, maxSegment);
            plannedPosition = SamplePositionAtTime(record.pathWorld, record.timestamps, timeSample, ref segmentIndex);
            if (pointCount >= 2)
            {
                segmentStartTime = record.timestamps[segmentIndex];
                segmentEndTime = record.timestamps[Mathf.Min(segmentIndex + 1, record.timestamps.Length - 1)];
                float dist = Vector3.Distance(
                    record.pathWorld[segmentIndex],
                    record.pathWorld[Mathf.Min(segmentIndex + 1, record.pathWorld.Count - 1)]);
                float dt = Mathf.Max(0.02f, segmentEndTime - segmentStartTime);
                plannedSpeed = dist / dt;
            }

            return true;
        }

        public float GetSpeedCap(AIP1TrafficCar agent, int pathIndex, float now)
        {
            Vector3 worldPosition = agent != null ? agent.transform.position : Vector3.zero;
            return GetSpeedCap(agent, pathIndex, worldPosition, now);
        }

        public float GetSpeedCap(AIP1TrafficCar agent, int pathIndex, Vector3 worldPosition, float now)
        {
            return GetGateDecision(agent, pathIndex, worldPosition, now).cap;
        }

        public GateDecision GetGateDecision(AIP1TrafficCar agent, int pathIndex, float now)
        {
            Vector3 worldPosition = agent != null ? agent.transform.position : Vector3.zero;
            return GetGateDecision(agent, pathIndex, worldPosition, now);
        }

        public GateDecision GetGateDecision(AIP1TrafficCar agent, int pathIndex, Vector3 worldPosition, float now)
        {
            GateDecision decision = new GateDecision
            {
                cap = float.PositiveInfinity,
                releaseTime = float.NaN,
                scheduleTime = float.NaN,
                clampedPathIndex = pathIndex,
                reason = "GO"
            };

            if (agent == null)
            {
                decision.reason = "AGENT_NULL";
                return decision;
            }

            if (!enableSpacePlanningReservation)
            {
                decision.reason = "PLANNER_DISABLED";
                MaybeLogGateDecision(agent, now, decision);
                return decision;
            }

            TryBuildSchedules(now);
            int agentId = agent.GetInstanceID();
            if (!_trajectoryByAgent.TryGetValue(agentId, out AgentTrajectory record))
            {
                decision.reason = "TRAJECTORY_MISSING";
                MaybeLogGateDecision(agent, now, decision);
                return decision;
            }

            decision.releaseTime = record.releaseTime;
            if (!_schedulesBuilt || record.timestamps == null || record.timestamps.Length == 0)
            {
                decision.cap = 0f;
                decision.reason = "SCHEDULE_NOT_READY";
                MaybeLogGateDecision(agent, now, decision);
                return decision;
            }

            if (now < record.releaseTime)
            {
                decision.cap = 0f;
                decision.reason = "BEFORE_RELEASE";
                MaybeLogGateDecision(agent, now, decision);
                return decision;
            }

            int clampedIndex = Mathf.Clamp(pathIndex, 0, record.timestamps.Length - 1);
            float scheduledTime = ResolveScheduleTimeAtPosition(record, clampedIndex, worldPosition, out int resolvedSegment);
            if (float.IsNaN(scheduledTime))
            {
                scheduledTime = record.timestamps[clampedIndex];
                resolvedSegment = Mathf.Clamp(clampedIndex, 0, Mathf.Max(0, record.timestamps.Length - 2));
            }

            decision.clampedPathIndex = resolvedSegment;
            decision.scheduleTime = scheduledTime;
            if (now < scheduledTime - Mathf.Max(0f, gateTolerance))
            {
                decision.cap = 0f;
                decision.reason = "BEFORE_SCHEDULE";
                MaybeLogGateDecision(agent, now, decision);
                return decision;
            }

            decision.cap = float.PositiveInfinity;
            decision.reason = "GO";
            MaybeLogGateDecision(agent, now, decision);
            return decision;
        }

        private static float ResolveScheduleTimeAtPosition(
            AgentTrajectory record,
            int hintPathIndex,
            Vector3 worldPosition,
            out int resolvedSegment)
        {
            resolvedSegment = 0;
            if (record == null || record.pathWorld == null || record.timestamps == null)
            {
                return float.NaN;
            }

            int nodeCount = Mathf.Min(record.pathWorld.Count, record.timestamps.Length);
            if (nodeCount <= 0)
            {
                return float.NaN;
            }

            if (nodeCount == 1)
            {
                resolvedSegment = 0;
                return record.timestamps[0];
            }

            int maxSegment = nodeCount - 2;
            int hintSegment = Mathf.Clamp(hintPathIndex, 0, maxSegment);
            int start = Mathf.Clamp(hintSegment - 2, 0, maxSegment);
            int end = Mathf.Clamp(hintSegment + 2, 0, maxSegment);

            float bestSqDist = float.PositiveInfinity;
            int bestSegment = hintSegment;
            float bestAlpha = 0f;
            bool found = false;

            FindClosestPathSegmentInRange(
                record.pathWorld,
                worldPosition,
                start,
                end,
                ref bestSqDist,
                ref bestSegment,
                ref bestAlpha,
                ref found);

            if (!found)
            {
                FindClosestPathSegmentInRange(
                    record.pathWorld,
                    worldPosition,
                    0,
                    maxSegment,
                    ref bestSqDist,
                    ref bestSegment,
                    ref bestAlpha,
                    ref found);
            }

            resolvedSegment = Mathf.Clamp(bestSegment, 0, maxSegment);
            float t0 = record.timestamps[resolvedSegment];
            float t1 = record.timestamps[resolvedSegment + 1];
            if (t1 <= t0 + 1e-5f)
            {
                return t1;
            }

            return Mathf.Lerp(t0, t1, bestAlpha);
        }

        private static void FindClosestPathSegmentInRange(
            List<Vector3> path,
            Vector3 worldPosition,
            int startSegment,
            int endSegment,
            ref float bestSqDist,
            ref int bestSegment,
            ref float bestAlpha,
            ref bool found)
        {
            if (path == null || path.Count < 2 || endSegment < startSegment)
            {
                return;
            }

            Vector2 pos = new Vector2(worldPosition.x, worldPosition.z);
            for (int i = startSegment; i <= endSegment; i++)
            {
                Vector3 a3 = path[i];
                Vector3 b3 = path[i + 1];
                Vector2 a = new Vector2(a3.x, a3.z);
                Vector2 b = new Vector2(b3.x, b3.z);
                Vector2 ab = b - a;
                float lenSq = ab.sqrMagnitude;
                float alpha = 0f;
                if (lenSq > 1e-6f)
                {
                    alpha = Mathf.Clamp01(Vector2.Dot(pos - a, ab) / lenSq);
                }

                Vector2 projected = a + ab * alpha;
                float sqDist = (pos - projected).sqrMagnitude;
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    bestSegment = i;
                    bestAlpha = alpha;
                    found = true;
                }
            }
        }

        private void TryBuildSchedules(float now)
        {
            if (!enableSpacePlanningReservation || _schedulesBuilt || _trajectoryByAgent.Count == 0)
            {
                return;
            }

            int activeCars = AgentRegistry.AllAgents.Count;
            bool allRegistered = activeCars <= 0 || _trajectoryByAgent.Count >= activeCars;
            bool forceBuild = now - scenarioStartTime >= Mathf.Max(0f, forceBuildAfterSec);
            if (!allRegistered && !forceBuild)
            {
                if (debugVerboseLogs && debugLogScheduleBuild && ShouldLogCoordinator(now))
                {
                    DebugOutput(
                        $"[SpaceTimeReservation][WAIT_BUILD] now={now:0.00}s registered={_trajectoryByAgent.Count} activeCars={activeCars} " +
                        $"allRegistered={allRegistered} forceBuildAfter={forceBuildAfterSec:0.00}s");
                }
                return;
            }

            if (debugVerboseLogs && debugLogScheduleBuild)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][BUILD_TRIGGER] now={now:0.00}s registered={_trajectoryByAgent.Count} activeCars={activeCars} " +
                    $"allRegistered={allRegistered} forceBuild={forceBuild}");
            }
            BuildAllSchedules();
        }

        private void BuildAllSchedules()
        {
            _occupied.Clear();
            if (debugVerboseLogs && debugLogScheduleBuild)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][BUILD_START] agents={_trajectoryByAgent.Count} dtRes={dtRes:0.###} radius={reserveRadius:0.###} " +
                    $"horizon={horizonSec:0.###} waitBudget={maxTotalWaitSec:0.###} gateTolerance={gateTolerance:0.###}");
            }

            List<AgentTrajectory> orderedAgents = BuildReleaseOrderedAgents();
            for (int i = 0; i < orderedAgents.Count; i++)
            {
                AgentTrajectory record = orderedAgents[i];
                record.timestamps = BuildNominalTimestamps(record);

                ScheduleSolveStats solveStats = ResolveConflictsWithWaitInsertion(record);
                if (solveStats.fallback)
                {
                    DebugOutput($"[SpaceTimeReservation] Schedule fallback: wait budget exceeded for agent {record.agentId}", warning: true, forceConsole: true);
                }

                int committedBins = CommitReservations(record);
                if (debugVerboseLogs && debugLogScheduleBuild)
                {
                    float tStart = record.timestamps != null && record.timestamps.Length > 0 ? record.timestamps[0] : float.NaN;
                    float tEnd = record.timestamps != null && record.timestamps.Length > 0
                        ? record.timestamps[record.timestamps.Length - 1]
                        : float.NaN;
                    DebugOutput(
                        $"[SpaceTimeReservation][SCHEDULE_DONE] agent={record.agentId} batch={record.batchIndex} group={record.groupName} " +
                        $"release={record.releaseTime - scenarioStartTime:0.###}s pathPts={record.pathWorld.Count} tStart={tStart:0.###} tEnd={tEnd:0.###} " +
                        $"waitInsertions={solveStats.waitInsertions} totalWait={solveStats.totalWaitSec:0.###}s checkedBins={solveStats.checkedBins} " +
                        $"conflictHits={solveStats.conflictHits} committedBins={committedBins}");
                }

                if (logReleaseSchedule)
                {
                    float releaseRelative = record.releaseTime - scenarioStartTime;
                    DebugOutput(
                        $"[SpaceTimeReservation] agent {record.agentId} releaseTime={releaseRelative:0.###}s " +
                        $"batch={record.batchIndex} group={record.groupName}",
                        forceConsole: true);
                }
            }

            _schedulesBuilt = true;
            if (debugValidatePlannedCollisionFree)
            {
                ValidatePlannedCollisionFreeSummary();
            }

            if (debugVerboseLogs && debugLogScheduleBuild)
            {
                DebugOutput($"[SpaceTimeReservation][BUILD_END] occupiedBins={_occupied.Count}");
            }
        }

        private void ValidatePlannedCollisionFreeSummary()
        {
            List<AgentTrajectory> planned = _trajectoryByAgent.Values
                .Where(record =>
                    record != null &&
                    record.pathWorld != null &&
                    record.pathWorld.Count >= 2 &&
                    record.timestamps != null &&
                    record.timestamps.Length >= 2)
                .ToList();
            if (planned.Count < 2)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][PLAN_COLLISION_CHECK] collisionFree=True agents={planned.Count} reason=insufficient_agents",
                    forceConsole: true);
                return;
            }

            float binSize = Mathf.Max(0.05f, dtRes);
            float localHorizon = Mathf.Max(binSize, horizonSec);
            float start = float.PositiveInfinity;
            float end = float.NegativeInfinity;
            float[] tStartByAgent = new float[planned.Count];
            float[] tEndByAgent = new float[planned.Count];
            float[] horizonEndByAgent = new float[planned.Count];
            float[] uncoveredTailByAgent = new float[planned.Count];
            for (int i = 0; i < planned.Count; i++)
            {
                tStartByAgent[i] = float.NaN;
                tEndByAgent[i] = float.NaN;
                horizonEndByAgent[i] = float.NaN;
                uncoveredTailByAgent[i] = 0f;
            }

            for (int i = 0; i < planned.Count; i++)
            {
                float[] ts = planned[i].timestamps;
                if (ts == null || ts.Length == 0)
                {
                    continue;
                }

                float t0 = ts[0];
                float tN = ts[ts.Length - 1];
                float horizonEnd = Mathf.Min(t0 + localHorizon, tN);
                float uncoveredTail = Mathf.Max(0f, tN - horizonEnd);

                tStartByAgent[i] = t0;
                tEndByAgent[i] = tN;
                horizonEndByAgent[i] = horizonEnd;
                uncoveredTailByAgent[i] = uncoveredTail;

                start = Mathf.Min(start, t0);
                end = Mathf.Max(end, tN);
            }

            if (float.IsInfinity(start) || float.IsInfinity(end) || end < start)
            {
                DebugOutput(
                    "[SpaceTimeReservation][PLAN_COLLISION_CHECK] collisionFree=Unknown reason=invalid_time_window",
                    warning: true,
                    forceConsole: true);
                return;
            }

            int[] segmentCursors = new int[planned.Count];
            float radius = Mathf.Max(0.1f, reserveRadius);
            float minDistThreshold = radius * 2f;

            int tbStart = Mathf.FloorToInt(start / binSize);
            int tbEnd = Mathf.FloorToInt(end / binSize);

            long pairChecks = 0;
            int conflictCount = 0;
            int conflictsInsideBothHorizons = 0;
            int conflictsOutsideEitherHorizon = 0;
            int conflictsOutsideBothHorizons = 0;
            int conflictsOutsideAHorizon = 0;
            int conflictsOutsideBHorizon = 0;
            bool hasFirstConflict = false;
            float firstConflictTime = 0f;
            int firstA = 0;
            int firstB = 0;
            float firstDistance = 0f;
            string firstConflictReason = "n/a";

            List<int> activeIndices = new List<int>(planned.Count);
            List<Vector2> activePositions = new List<Vector2>(planned.Count);
            const int conflictDetailLimit = 16;
            int conflictDetailsLogged = 0;

            for (int tb = tbStart; tb <= tbEnd; tb++)
            {
                float timeSample = tb * binSize;
                activeIndices.Clear();
                activePositions.Clear();

                for (int i = 0; i < planned.Count; i++)
                {
                    AgentTrajectory record = planned[i];
                    float[] ts = record.timestamps;
                    if (ts == null || ts.Length < 2)
                    {
                        continue;
                    }

                    float t0 = ts[0];
                    float tN = ts[ts.Length - 1];
                    if (timeSample < t0 || timeSample > tN)
                    {
                        continue;
                    }

                    int cursor = segmentCursors[i];
                    Vector3 pos = SamplePositionAtTime(record.pathWorld, ts, timeSample, ref cursor);
                    segmentCursors[i] = cursor;
                    activeIndices.Add(i);
                    activePositions.Add(new Vector2(pos.x, pos.z));
                }

                for (int a = 0; a < activeIndices.Count - 1; a++)
                {
                    int idxA = activeIndices[a];
                    AgentTrajectory recA = planned[idxA];
                    Vector2 posA = activePositions[a];
                    for (int b = a + 1; b < activeIndices.Count; b++)
                    {
                        int idxB = activeIndices[b];
                        AgentTrajectory recB = planned[idxB];
                        pairChecks++;
                        Vector2 posB = activePositions[b];
                        float dist = Vector2.Distance(posA, posB);
                        if (dist >= minDistThreshold)
                        {
                            continue;
                        }

                        conflictCount++;
                        bool outsideA = timeSample > horizonEndByAgent[idxA] + 1e-4f;
                        bool outsideB = timeSample > horizonEndByAgent[idxB] + 1e-4f;
                        string reason;
                        if (!outsideA && !outsideB)
                        {
                            conflictsInsideBothHorizons++;
                            reason = "inside_both_horizons";
                        }
                        else
                        {
                            conflictsOutsideEitherHorizon++;
                            if (outsideA && outsideB)
                            {
                                conflictsOutsideBothHorizons++;
                                reason = "outside_both_horizons";
                            }
                            else if (outsideA)
                            {
                                conflictsOutsideAHorizon++;
                                reason = "outside_a_horizon";
                            }
                            else
                            {
                                conflictsOutsideBHorizon++;
                                reason = "outside_b_horizon";
                            }
                        }

                        if (!hasFirstConflict)
                        {
                            hasFirstConflict = true;
                            firstConflictTime = timeSample;
                            firstA = recA.agentId;
                            firstB = recB.agentId;
                            firstDistance = dist;
                            firstConflictReason = reason;
                        }

                        if (conflictDetailsLogged < conflictDetailLimit)
                        {
                            conflictDetailsLogged++;
                            DebugOutput(
                                $"[SpaceTimeReservation][PLAN_COLLISION_DETAIL] #{conflictDetailsLogged} tb={tb} " +
                                $"t={timeSample - scenarioStartTime:0.###}s reason={reason} dist={dist:0.###}/{minDistThreshold:0.###} " +
                                $"a={recA.agentId} posA=({posA.x:0.###},{posA.y:0.###}) aWin=[{tStartByAgent[idxA] - scenarioStartTime:0.###},{horizonEndByAgent[idxA] - scenarioStartTime:0.###}]s aPathEnd={tEndByAgent[idxA] - scenarioStartTime:0.###}s " +
                                $"b={recB.agentId} posB=({posB.x:0.###},{posB.y:0.###}) bWin=[{tStartByAgent[idxB] - scenarioStartTime:0.###},{horizonEndByAgent[idxB] - scenarioStartTime:0.###}]s bPathEnd={tEndByAgent[idxB] - scenarioStartTime:0.###}s");
                        }
                    }
                }
            }

            if (conflictCount == 0)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][PLAN_COLLISION_CHECK] collisionFree=True agents={planned.Count} bins={tbEnd - tbStart + 1} " +
                    $"pairChecks={pairChecks} threshold={minDistThreshold:0.###}",
                    forceConsole: true);
                return;
            }

            List<int> coverageGapAgents = Enumerable.Range(0, planned.Count)
                .Where(i => uncoveredTailByAgent[i] > binSize)
                .OrderByDescending(i => uncoveredTailByAgent[i])
                .ToList();
            const int coverageGapLogLimit = 12;
            DebugOutput(
                $"[SpaceTimeReservation][PLAN_COLLISION_COVERAGE] horizon={localHorizon:0.###}s agentsWithUncoveredTail={coverageGapAgents.Count} " +
                $"tailThreshold={binSize:0.###}s",
                warning: true,
                forceConsole: true);
            for (int k = 0; k < Mathf.Min(coverageGapLogLimit, coverageGapAgents.Count); k++)
            {
                int i = coverageGapAgents[k];
                AgentTrajectory rec = planned[i];
                DebugOutput(
                    $"[SpaceTimeReservation][PLAN_COLLISION_COVERAGE] rank={k + 1} agent={rec.agentId} " +
                    $"tStart={tStartByAgent[i] - scenarioStartTime:0.###}s horizonEnd={horizonEndByAgent[i] - scenarioStartTime:0.###}s " +
                    $"pathEnd={tEndByAgent[i] - scenarioStartTime:0.###}s uncoveredTail={uncoveredTailByAgent[i]:0.###}s",
                    warning: true,
                    forceConsole: true);
            }

            DebugOutput(
                $"[SpaceTimeReservation][PLAN_COLLISION_CHECK] collisionFree=False agents={planned.Count} bins={tbEnd - tbStart + 1} " +
                $"pairChecks={pairChecks} conflicts={conflictCount} threshold={minDistThreshold:0.###} " +
                $"insideBoth={conflictsInsideBothHorizons} outsideEither={conflictsOutsideEitherHorizon} " +
                $"outsideBoth={conflictsOutsideBothHorizons} outsideAOnly={conflictsOutsideAHorizon} outsideBOnly={conflictsOutsideBHorizon} " +
                $"first=t{firstConflictTime - scenarioStartTime:0.###}s a={firstA} b={firstB} dist={firstDistance:0.###} reason={firstConflictReason}",
                warning: true,
                forceConsole: true);
        }

        private List<AgentTrajectory> BuildReleaseOrderedAgents()
        {
            List<AgentTrajectory> all = _trajectoryByAgent.Values
                .Where(record => record != null && record.pathWorld != null && record.pathWorld.Count >= 2)
                .OrderBy(record => record.globalIndex)
                .ToList();
            if (all.Count == 0)
            {
                return all;
            }

            int safeBatchSize = Mathf.Max(1, batchSize);

            List<List<AgentTrajectory>> batches;
            if (!enableBatchRelease)
            {
                batches = new List<List<AgentTrajectory>>
                {
                    all.OrderBy(record => record.globalIndex).ToList()
                };
            }
            else
            {
                StartGoalAssignments assignments = StartGoalAssignments.Instance;
                List<string> groupOrder = ResolveGroupOrder(assignments, all);
                bool useGroupedBatches = assignments != null && assignments.UsesGroupedBuild && groupOrder.Count > 1;
                batches = useGroupedBatches
                    ? BuildGroupedReleaseBatches(all, safeBatchSize, groupOrder, oddThenEvenOrdering)
                    : BuildNonGroupedReleaseBatches(all, safeBatchSize, oddThenEvenOrdering);
            }

            if (debugVerboseLogs && debugLogBatchComposition)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][BATCH_BUILD] useBatchRelease={enableBatchRelease} oddThenEven={oddThenEvenOrdering} " +
                    $"batchSize={safeBatchSize} batches={batches.Count}");
            }
            return FlattenBatchesAssignReleaseTimes(batches);
        }

        private static List<string> ResolveGroupOrder(StartGoalAssignments assignments, List<AgentTrajectory> all)
        {
            HashSet<string> presentGroups = new HashSet<string>(
                all.Select(record => NormalizeGroupName(record.groupName)),
                StringComparer.Ordinal);
            List<string> order = new List<string>();

            if (assignments != null)
            {
                IReadOnlyList<string> startsOrder = assignments.GroupOrderInStartsHierarchy;
                if (startsOrder != null)
                {
                    for (int i = 0; i < startsOrder.Count; i++)
                    {
                        string group = NormalizeGroupName(startsOrder[i]);
                        if (presentGroups.Contains(group) && !order.Contains(group))
                        {
                            order.Add(group);
                        }
                    }
                }
            }

            for (int i = 0; i < all.Count; i++)
            {
                string group = NormalizeGroupName(all[i].groupName);
                if (presentGroups.Contains(group) && !order.Contains(group))
                {
                    order.Add(group);
                }
            }

            return order;
        }

        private static List<List<AgentTrajectory>> BuildGroupedReleaseBatches(
            List<AgentTrajectory> all,
            int safeBatchSize,
            List<string> groupOrder,
            bool oddThenEvenOrdering)
        {
            List<List<AgentTrajectory>> membersPerGroup = new List<List<AgentTrajectory>>();

            for (int g = 0; g < groupOrder.Count; g++)
            {
                string groupName = NormalizeGroupName(groupOrder[g]);
                List<AgentTrajectory> membersOrdered = all
                    .Where(record => string.Equals(NormalizeGroupName(record.groupName), groupName, StringComparison.Ordinal))
                    .OrderBy(record => record.indexWithinGroup)
                    .ThenBy(record => record.globalIndex)
                    .ToList();
                List<AgentTrajectory> members = oddThenEvenOrdering
                    ? BuildOddThenEvenOrder(membersOrdered)
                    : membersOrdered;
                if (members.Count == 0)
                {
                    continue;
                }

                membersPerGroup.Add(members);
            }

            if (membersPerGroup.Count <= 1)
            {
                return BuildNonGroupedReleaseBatches(all, safeBatchSize, oddThenEvenOrdering);
            }

            int groupCount = membersPerGroup.Count;
            int[] idx = new int[groupCount];
            int remaining = all.Count;
            List<List<AgentTrajectory>> batches = new List<List<AgentTrajectory>>();

            while (remaining > 0)
            {
                List<AgentTrajectory> batchList = new List<AgentTrajectory>(safeBatchSize);
                int baseTake = safeBatchSize / groupCount;
                int remainder = safeBatchSize % groupCount;

                for (int g = 0; g < groupCount && batchList.Count < safeBatchSize; g++)
                {
                    int take = baseTake + (g < remainder ? 1 : 0);
                    List<AgentTrajectory> members = membersPerGroup[g];
                    while (take > 0 && idx[g] < members.Count && batchList.Count < safeBatchSize)
                    {
                        batchList.Add(members[idx[g]]);
                        idx[g]++;
                        remaining--;
                        take--;
                    }
                }

                while (batchList.Count < safeBatchSize && remaining > 0)
                {
                    bool addedInPass = false;
                    for (int g = 0; g < groupCount && batchList.Count < safeBatchSize; g++)
                    {
                        List<AgentTrajectory> members = membersPerGroup[g];
                        if (idx[g] >= members.Count)
                        {
                            continue;
                        }

                        batchList.Add(members[idx[g]]);
                        idx[g]++;
                        remaining--;
                        addedInPass = true;
                    }

                    if (!addedInPass)
                    {
                        break;
                    }
                }

                if (batchList.Count == 0)
                {
                    break;
                }

                batches.Add(batchList);
            }

            return batches;
        }

        private static List<List<AgentTrajectory>> BuildNonGroupedReleaseBatches(
            List<AgentTrajectory> all,
            int safeBatchSize,
            bool oddThenEvenOrdering)
        {
            List<AgentTrajectory> orderedByGlobal = all.OrderBy(record => record.globalIndex).ToList();
            List<AgentTrajectory> ordered = oddThenEvenOrdering
                ? BuildOddThenEvenOrder(orderedByGlobal)
                : orderedByGlobal;
            List<List<AgentTrajectory>> batches = new List<List<AgentTrajectory>>();
            for (int start = 0; start < ordered.Count; start += safeBatchSize)
            {
                int count = Mathf.Min(safeBatchSize, ordered.Count - start);
                List<AgentTrajectory> batch = ordered.GetRange(start, count);
                batches.Add(batch);
            }

            return batches;
        }

        private static List<AgentTrajectory> BuildOddThenEvenOrder(List<AgentTrajectory> source)
        {
            if (source == null || source.Count <= 2)
            {
                return source ?? new List<AgentTrajectory>();
            }

            List<AgentTrajectory> ordered = new List<AgentTrajectory>(source.Count);
            for (int i = 0; i < source.Count; i += 2)
            {
                ordered.Add(source[i]);
            }

            for (int i = 1; i < source.Count; i += 2)
            {
                ordered.Add(source[i]);
            }

            return ordered;
        }

        private List<AgentTrajectory> FlattenBatchesAssignReleaseTimes(List<List<AgentTrajectory>> batches)
        {
            List<AgentTrajectory> ordered = new List<AgentTrajectory>();
            float interval = Mathf.Max(0f, batchIntervalSec);
            float microStagger = enableBatchRelease && enableWithinBatchMicroStagger
                ? Mathf.Max(0f, withinBatchStaggerSec)
                : 0f;
            if (batches == null)
            {
                return ordered;
            }

            for (int k = 0; k < batches.Count; k++)
            {
                List<AgentTrajectory> batch = batches[k];
                if (batch == null || batch.Count == 0)
                {
                    continue;
                }

                float batchBaseRelease = scenarioStartTime + k * interval;
                for (int pos = 0; pos < batch.Count; pos++)
                {
                    AgentTrajectory record = batch[pos];
                    record.batchIndex = k;
                    record.releaseTime = batchBaseRelease + microStagger * pos;
                    ordered.Add(record);
                }

                if (debugVerboseLogs && debugLogBatchComposition)
                {
                    string members = string.Join(
                        ", ",
                        batch.Select((record, pos) =>
                            $"id={record.agentId}|g={record.groupName}|gi={record.globalIndex}|ig={record.indexWithinGroup}|r={record.releaseTime - scenarioStartTime:0.###}s|slot={pos}"));
                    DebugOutput($"[SpaceTimeReservation][BATCH_{k}] {members}");
                }
            }

            return ordered;
        }

        private static string NormalizeGroupName(string groupName)
        {
            return string.IsNullOrEmpty(groupName) ? "Ungrouped" : groupName;
        }

        private float[] BuildNominalTimestamps(AgentTrajectory record)
        {
            int n = record.pathWorld.Count;
            float[] t = new float[n];
            if (n == 0)
            {
                return t;
            }

            t[0] = record.releaseTime;
            for (int i = 0; i < n - 1; i++)
            {
                float dist = Vector3.Distance(record.pathWorld[i], record.pathWorld[i + 1]);
                float segmentSpeed = ResolveSegmentSpeed(record, i);
                float dt = dist / Mathf.Max(0.1f, segmentSpeed);
                dt = Mathf.Max(0.02f, dt);
                t[i + 1] = t[i] + dt;
            }

            return t;
        }

        private float ResolveSegmentSpeed(AgentTrajectory record, int segmentIndex)
        {
            if (record.speedProfile != null && record.speedProfile.Length > segmentIndex + 1)
            {
                float v0 = Mathf.Max(0.1f, record.speedProfile[segmentIndex]);
                float v1 = Mathf.Max(0.1f, record.speedProfile[segmentIndex + 1]);
                return Mathf.Max(0.1f, 0.5f * (v0 + v1));
            }

            return Mathf.Max(0.1f, record.nominalSpeed > 0f ? record.nominalSpeed : defaultNominalSpeed);
        }

        private ScheduleSolveStats ResolveConflictsWithWaitInsertion(AgentTrajectory record)
        {
            ScheduleSolveStats stats = new ScheduleSolveStats();
            if (record.timestamps == null || record.timestamps.Length < 2)
            {
                return stats;
            }

            float binSize = Mathf.Max(0.05f, dtRes);
            float waitQuantum = binSize;
            float waitBudget = Mathf.Max(0f, maxTotalWaitSec);
            float totalWait = 0f;

            while (true)
            {
                bool shifted = false;
                float start = record.timestamps[0];
                float end = Mathf.Min(start + Mathf.Max(binSize, horizonSec), record.timestamps[record.timestamps.Length - 1]);
                int tbStart = Mathf.FloorToInt(start / binSize);
                int tbEnd = Mathf.FloorToInt(end / binSize);

                int segmentCursor = 0;
                for (int tb = tbStart; tb <= tbEnd; tb++)
                {
                    stats.checkedBins++;
                    float timeSample = tb * binSize;
                    Vector3 pos = SamplePositionAtTime(record.pathWorld, record.timestamps, timeSample, ref segmentCursor);
                    Vector2 posXZ = new Vector2(pos.x, pos.z);
                    if (!TryGetConflict(tb, posXZ, out Disc blocker, out float distance, out float minDistance))
                    {
                        continue;
                    }

                    stats.conflictHits++;
                    int conflictIndex = FindSegmentIndexForTime(record.timestamps, timeSample);
                    DelayTimestamps(record.timestamps, conflictIndex, waitQuantum);
                    totalWait += waitQuantum;
                    stats.waitInsertions++;
                    stats.totalWaitSec = totalWait;
                    shifted = true;

                    if (debugVerboseLogs && debugLogConflictEvents)
                    {
                        DebugOutput(
                            $"[SpaceTimeReservation][CONFLICT] agent={record.agentId} tb={tb} t={timeSample:0.###}s seg={conflictIndex} " +
                            $"pos=({posXZ.x:0.###},{posXZ.y:0.###}) blocker={blocker.agentId} dist={distance:0.###} minDist={minDistance:0.###} " +
                            $"waitAdded={waitQuantum:0.###} totalWait={totalWait:0.###}/{waitBudget:0.###}");
                    }

                    if (totalWait > waitBudget)
                    {
                        stats.fallback = true;
                        return stats;
                    }

                    break;
                }

                if (!shifted)
                {
                    return stats;
                }
            }
        }

        private bool TryGetConflict(int timeBin, Vector2 posXZ, out Disc blocker, out float distance, out float minDistance)
        {
            blocker = default;
            distance = 0f;
            minDistance = 0f;
            if (!_occupied.TryGetValue(timeBin, out List<Disc> discs) || discs == null || discs.Count == 0)
            {
                return false;
            }

            float ownRadius = Mathf.Max(0.1f, reserveRadius);
            for (int i = 0; i < discs.Count; i++)
            {
                Disc disc = discs[i];
                float minDist = ownRadius + Mathf.Max(0.1f, disc.radius);
                Vector2 delta = posXZ - disc.posXZ;
                float sqDist = delta.sqrMagnitude;
                if (sqDist < minDist * minDist)
                {
                    blocker = disc;
                    distance = Mathf.Sqrt(sqDist);
                    minDistance = minDist;
                    return true;
                }
            }

            return false;
        }

        private int CommitReservations(AgentTrajectory record)
        {
            if (record.timestamps == null || record.timestamps.Length < 2)
            {
                return 0;
            }

            float binSize = Mathf.Max(0.05f, dtRes);
            float start = record.timestamps[0];
            float end = Mathf.Min(start + Mathf.Max(binSize, horizonSec), record.timestamps[record.timestamps.Length - 1]);
            int tbStart = Mathf.FloorToInt(start / binSize);
            int tbEnd = Mathf.FloorToInt(end / binSize);

            int segmentCursor = 0;
            float radius = Mathf.Max(0.1f, reserveRadius);
            int binsCommitted = 0;
            for (int tb = tbStart; tb <= tbEnd; tb++)
            {
                float timeSample = tb * binSize;
                Vector3 pos = SamplePositionAtTime(record.pathWorld, record.timestamps, timeSample, ref segmentCursor);
                Disc disc = new Disc(new Vector2(pos.x, pos.z), radius, record.agentId);
                if (!_occupied.TryGetValue(tb, out List<Disc> list))
                {
                    list = new List<Disc>();
                    _occupied.Add(tb, list);
                }

                list.Add(disc);
                binsCommitted++;
            }

            if (debugVerboseLogs && debugLogReservationCommits)
            {
                DebugOutput(
                    $"[SpaceTimeReservation][COMMIT] agent={record.agentId} bins={binsCommitted} tb=[{tbStart},{tbEnd}] " +
                    $"window=[{start:0.###},{end:0.###}]");
            }

            return binsCommitted;
        }

        private static float[] CopySpeedProfile(float[] source, int count)
        {
            if (source == null || source.Length == 0 || count <= 0)
            {
                return null;
            }

            float[] copy = new float[count];
            int copyCount = Mathf.Min(source.Length, count);
            Array.Copy(source, copy, copyCount);
            float fillValue = Mathf.Max(1f, copyCount > 0 ? source[copyCount - 1] : 1f);
            for (int i = copyCount; i < count; i++)
            {
                copy[i] = fillValue;
            }

            return copy;
        }

        private static Vector3 SamplePositionAtTime(
            List<Vector3> path,
            float[] timestamps,
            float timeSample,
            ref int segmentIndex)
        {
            if (path == null || path.Count == 0 || timestamps == null || timestamps.Length == 0)
            {
                return Vector3.zero;
            }

            int pointCount = Mathf.Min(path.Count, timestamps.Length);
            if (pointCount == 1)
            {
                return path[0];
            }

            int maxSegment = pointCount - 2;
            segmentIndex = Mathf.Clamp(segmentIndex, 0, maxSegment);
            while (segmentIndex < maxSegment && timeSample >= timestamps[segmentIndex + 1])
            {
                segmentIndex++;
            }

            while (segmentIndex > 0 && timeSample < timestamps[segmentIndex])
            {
                segmentIndex--;
            }

            float t0 = timestamps[segmentIndex];
            float t1 = timestamps[segmentIndex + 1];
            Vector3 p0 = path[segmentIndex];
            Vector3 p1 = path[segmentIndex + 1];
            if (t1 <= t0 + 1e-5f)
            {
                return p1;
            }

            float alpha = Mathf.Clamp01((timeSample - t0) / (t1 - t0));
            return Vector3.Lerp(p0, p1, alpha);
        }

        private static int FindSegmentIndexForTime(float[] timestamps, float timeSample)
        {
            if (timestamps == null || timestamps.Length < 2)
            {
                return 0;
            }

            int maxSegment = timestamps.Length - 2;
            if (timeSample <= timestamps[0])
            {
                return 0;
            }

            if (timeSample >= timestamps[timestamps.Length - 1])
            {
                return maxSegment;
            }

            int lo = 0;
            int hi = maxSegment;
            while (lo <= hi)
            {
                int mid = lo + ((hi - lo) >> 1);
                float t0 = timestamps[mid];
                float t1 = timestamps[mid + 1];
                if (timeSample < t0)
                {
                    hi = mid - 1;
                }
                else if (timeSample >= t1)
                {
                    lo = mid + 1;
                }
                else
                {
                    return mid;
                }
            }

            return Mathf.Clamp(lo, 0, maxSegment);
        }

        private static void DelayTimestamps(float[] timestamps, int startIndex, float delay)
        {
            if (timestamps == null || timestamps.Length == 0 || delay <= 0f)
            {
                return;
            }

            int from = Mathf.Clamp(startIndex, 0, timestamps.Length - 1);
            for (int i = from; i < timestamps.Length; i++)
            {
                timestamps[i] += delay;
            }
        }

        private void MaybeLogGateDecision(AIP1TrafficCar agent, float now, GateDecision decision)
        {
            if (!debugVerboseLogs || !debugLogGateDecisions || agent == null)
            {
                return;
            }

            int agentId = agent.GetInstanceID();
            if (!ShouldLogAgent(agentId, now))
            {
                return;
            }

            Vector3 pos = agent.transform.position;
            string release = float.IsNaN(decision.releaseTime) ? "n/a" : $"{decision.releaseTime - now:0.###}s";
            string schedule = float.IsNaN(decision.scheduleTime) ? "n/a" : $"{decision.scheduleTime - now:0.###}s";
            string cap = float.IsPositiveInfinity(decision.cap) ? "inf" : $"{decision.cap:0.###}";
            DebugOutput(
                $"[SpaceTimeReservation][GATE] agent={agentId} t={now:0.###}s reason={decision.reason} cap={cap} " +
                $"pathIdx={decision.clampedPathIndex} dtToRelease={release} dtToSchedule={schedule} " +
                $"pos=({pos.x:0.###},{pos.z:0.###})");
        }

        private bool ShouldLogCoordinator(float now)
        {
            float interval = Mathf.Max(0.02f, debugPrintIntervalSec);
            if (now + 1e-5f < _nextCoordinatorDebugTime)
            {
                return false;
            }

            _nextCoordinatorDebugTime = now + interval;
            return true;
        }

        private bool ShouldLogAgent(int agentId, float now)
        {
            float interval = Mathf.Max(0.02f, debugPrintIntervalSec);
            if (_nextGateDebugTimeByAgent.TryGetValue(agentId, out float nextTime) && now + 1e-5f < nextTime)
            {
                return false;
            }

            _nextGateDebugTimeByAgent[agentId] = now + interval;
            return true;
        }

        public void LogExternalDebug(string message, bool warning = false, bool forceConsole = false)
        {
            if (string.IsNullOrEmpty(message))
            {
                return;
            }

            DebugOutput(message, warning, forceConsole);
        }

        private void DebugOutput(string message, bool warning = false, bool forceConsole = false)
        {
            if (string.IsNullOrEmpty(message))
            {
                return;
            }

            EnsureDebugFileReady();
            string line = $"{DateTime.Now:HH:mm:ss.fff} {message}";
            if (debugWriteToFile && !string.IsNullOrEmpty(_debugLogFilePath))
            {
                try
                {
                    File.AppendAllText(_debugLogFilePath, line + Environment.NewLine);
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[SpaceTimeReservation][DEBUG_FILE_ERROR] {ex.GetType().Name}: {ex.Message}");
                }
            }

            if (forceConsole || debugEchoToConsole)
            {
                if (warning)
                {
                    Debug.LogWarning(message);
                }
                else
                {
                    Debug.Log(message);
                }
            }
        }

        private void EnsureDebugFileReady()
        {
            string desiredName = string.IsNullOrWhiteSpace(debugFileName)
                ? "SpaceTimeReservationDebug.txt"
                : debugFileName.Trim();
            bool fileChanged = !string.Equals(_debugFileNameResolved, desiredName, StringComparison.Ordinal);
            if (_debugFileInitialized && !fileChanged && !_pendingClearDebugFile)
            {
                return;
            }

            _debugFileNameResolved = desiredName;
            _debugLogFilePath = System.IO.Path.Combine(Application.persistentDataPath, desiredName);
            debugLogFilePathRuntime = _debugLogFilePath;
            bool clearFile = !_debugFileInitialized
                ? debugClearFileOnAwake || _pendingClearDebugFile
                : _pendingClearDebugFile;

            if (debugWriteToFile && clearFile)
            {
                try
                {
                    File.WriteAllText(_debugLogFilePath, string.Empty);
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[SpaceTimeReservation][DEBUG_FILE_RESET_ERROR] {ex.GetType().Name}: {ex.Message}");
                }
            }

            _pendingClearDebugFile = false;
            _debugFileInitialized = true;
            if (debugWriteToFile)
            {
                try
                {
                    File.AppendAllText(
                        _debugLogFilePath,
                        $"{DateTime.Now:HH:mm:ss.fff} [SpaceTimeReservation][DEBUG_FILE] path={_debugLogFilePath}{Environment.NewLine}");
                }
                catch
                {
                    // Best effort; avoid recursive logging from here.
                }
            }

            if (debugVerboseLogs || debugEchoToConsole)
            {
                Debug.Log($"[SpaceTimeReservation][DEBUG_FILE] path={_debugLogFilePath}");
            }
        }
    }
}
