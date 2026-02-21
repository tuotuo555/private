using System;
using System.Collections.Generic;
using System.Text;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Control;
using Scripts.Game;
using Scripts.Path;
using Scripts.Planning;
using Scripts.Traffic;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : Agent
{
    [Header("Phase 1 - Grid A*")]
    public float gridCellSize = 2f;
    public bool allowDiagonal = true;
    public int maxAStarIterations = 200000;
    public LayerMask obstacleMask;
    public float inflateR = 1.2f;
    public float probeScale = 0.7f;
    public float probeFwd = 1.2f;
    public float probeSide = 0.8f;
    public float checkY = 0.3f;

    [Header("Category Obstacle Inflation")]
    public bool enableCategoryObstacleInflation = false;
    [TextArea(1, 3)] public string obstacleInflationCategories = string.Empty;
    public float categoryInflationRadius = 1f;

    [Header("Phase 2 - Pure Pursuit")]
    public bool enablePurePursuit = true;
    public float lookaheadBase = 6f;
    public float lookaheadPerSpeed = 0.5f;
    public float wheelBase = 2.6f;
    public float maxSteerDeg = 25f;
    public float goalStopDistance = 2.2f;

    [Header("Speed Profile")]
    public float maxSpeed = 12f;
    public float maxAccel = 4f;
    public float maxDecel = 6f;
    public float maxLateralAccel = 5f;
    public float curveSafetyFactor = 0.7f;

    [Header("Speed Controller")]
    public float speedKp = 0.8f;
    public float throttleRate = 1.5f;
    public float brakeRate = 3f;

    [Header("Test (Random Active Cars)")]
    public bool testLimitActiveCars = false;
    public int testActiveCarCount = 8;
    public int testActiveCarsSeed = 2026;
    public bool testFreezeInactiveCars = true;

    [Header("Lifecycle")]
    public bool releaseOnGoalReached = true;

    [Header("Phase 4 - Space-Time Reservation")]
    public bool enableReservations = true;
    [Tooltip("Reservation time-bin size.")]
    public float reservationDtRes = 0.3f;
    [Tooltip("Disc radius used by reservation occupancy checks.")]
    public float reservationRadius = 2.2f;
    [Tooltip("Rolling horizon for reservation planning.")]
    public float reservationHorizonSec = 25f;
    [Tooltip("Maximum total WAIT insertion budget per agent.")]
    public float reservationMaxTotalWaitSec = 12f;
    [Tooltip("How early is considered too early for schedule gating.")]
    public float reservationGateTolerance = 0.35f;
    [Tooltip("Fallback speed used when no speed profile exists.")]
    public float reservationDefaultNominalSpeed = 8f;

    [Header("Phase 4 - Batch Release Ordering")]
    public bool reservationBatchRelease = true;
    [Min(1f)] public int reservationBatchSize = 4;
    [Min(0f)] public float reservationBatchIntervalSec = 15f;
    [Tooltip("Use odd-then-even order (1,3,5... then 2,4,6...).")]
    public bool reservationOddThenEvenOrdering = true;
    public bool reservationEnableMicroStagger = true;
    [Min(0f)] public float reservationMicroStaggerSec = 0.2f;
    [Min(0f)] public float reservationForceBuildAfterSec = 1f;
    public bool reservationLogReleaseSchedule = true;

    [Header("Phase 4 - Schedule Supervisor")]
    [Tooltip("If actual progress is ahead of allowed progress by more than this amount, clamp to crawl speed.")]
    [Min(0f)] public float reservationAheadCrawlMeters = 0.05f;
    [Tooltip("If actual progress is ahead of allowed progress by more than this amount, force stop.")]
    [Min(0f)] public float reservationAheadStopMeters = 0.5f;
    [Tooltip("Crawl speed cap used when the car is only slightly ahead of schedule.")]
    [Min(0f)] public float reservationAheadCrawlSpeed = 0.8f;
    [Tooltip("Conservative time guard for allowed-progress sampling (positive values reduce allowed progress).")]
    [Min(0f)] public float reservationProgressTimeGuardSec = 0.15f;

    [Header("Phase 4 - Debug")]
    [Tooltip("Enable detailed runtime logs for reservation gating/troubleshooting.")]
    public bool reservationDebugVerbose = false;
    [Tooltip("Debug print interval to avoid per-frame spam.")]
    [Min(0.02f)] public float reservationDebugIntervalSec = 0.2f;
    [Tooltip("Enable high-volume per-car gate/state logs (can be very large).")]
    public bool reservationDebugGateLogs = false;
    [Tooltip("Log planned-vs-real speed diagnostics for reservation troubleshooting.")]
    public bool reservationDebugSpeedMatchLogs = true;
    [Tooltip("Log planned path + planned timestamp axis once per agent (after schedule is available).")]
    public bool reservationDebugPlanTraceLogs = true;
    [Tooltip("Log real runtime path history + time axis and planned-vs-real drift.")]
    public bool reservationDebugRealTraceLogs = true;
    [Tooltip("Run one startup summary check that validates planned trajectories for pairwise overlaps.")]
    public bool reservationDebugValidatePlannedCollisionFree = true;
    [Tooltip("How many recent real samples to include in REAL_TRACE logs.")]
    [Min(4f)] public int reservationDebugRealTraceHistoryCount = 20;
    [Tooltip("Absolute speed error threshold for MISMATCH classification.")]
    [Min(0f)] public float reservationDebugSpeedMismatchAbs = 1.5f;
    [Tooltip("Relative speed error threshold for MISMATCH classification.")]
    [Min(0f)] public float reservationDebugSpeedMismatchRatio = 0.35f;
    [Tooltip("Log collision details with this car.")]
    public bool reservationDebugLogCollisions = true;
    [Tooltip("Write debug logs into a txt file.")]
    public bool reservationDebugWriteToFile = true;
    [Tooltip("Also mirror verbose debug logs to Unity Console.")]
    public bool reservationDebugEchoConsole = false;
    [Tooltip("Clear debug log file on each play start.")]
    public bool reservationDebugClearFileOnStart = true;
    [Tooltip("Debug log file name saved under Application.persistentDataPath.")]
    public string reservationDebugFileName = "SpaceTimeReservationDebug.txt";

    [Header("Gizmos")]
    public bool drawRawPath = true;
    public bool drawSmoothedPath = true;
    public bool drawLookaheadPoint = true;
    public Color rawPathColor = new Color(1f, 0.7f, 0.2f, 1f);
    public Color smoothedPathColor = Color.cyan;
    public Color lookaheadColor = Color.yellow;

    private CarController _car;
    private Rigidbody _rb;
    private StartGoalAssignments.Assignment _assignment;
    private bool _hasAssignment;
    private Vector3 _goalPosition;
    private readonly PurePursuitFollower _purePursuit = new PurePursuitFollower();

    private readonly GridAStarPlanner _gridPlanner = new GridAStarPlanner();
    private readonly List<Vector3> _rawPath = new List<Vector3>();
    private readonly List<Vector3> _smoothedPath = new List<Vector3>();
    private readonly List<Bounds> _inflatedObstacleBounds = new List<Bounds>();
    private readonly List<Bounds> _plannerObstacleBounds = new List<Bounds>();
    private float[] _speedProfile;

    private int _closestPathIndex;
    private Vector3 _lookaheadPoint;
    private bool _hasLookaheadPoint;
    private Vector3 _trackingGoalPosition;
    private float _throttle;
    private float _brake;
    private bool _goalReached;
    private bool _released;
    private bool _pathInvalid;
    private bool _isFrozenByActiveCarLimit;
    private float _nextReservationDebugLogAt;
    private int _autoObstacleMask;
    private int _priorityIndex;
    private bool _reservationPlanTraceLogged;
    private readonly List<RealTraceSample> _reservationRealTraceHistory = new List<RealTraceSample>();

    private struct RealTraceSample
    {
        public readonly float t;
        public readonly Vector3 pos;
        public readonly float speed;
        public readonly int pathIndex;

        public RealTraceSample(float time, Vector3 position, float speedMetersPerSec, int index)
        {
            t = time;
            pos = position;
            speed = speedMetersPerSec;
            pathIndex = index;
        }
    }

    private static bool s_randomSelectionReady;
    private static int s_randomSelectionSeed;
    private static int s_randomSelectionCount;
    private static readonly HashSet<int> RandomSelectedAgentIds = new HashSet<int>();
    public Rigidbody RigidbodyRef => _rb;
    public int PriorityIndex => _priorityIndex;
    public bool IsFinished => _goalReached || _released || _isFrozenByActiveCarLimit || !enabled;
    public int AssignmentIndex => _assignment.globalIndex;
    public string AssignmentGroup => _assignment.groupName;

    public override void Initialize()
    {
        _car = GetComponent<CarController>();
        _rb = GetComponent<Rigidbody>();
        AgentRegistry.Register(this);

        _assignment = StartGoalAssignments.Instance.GetAssignmentForAgent(this);
        _hasAssignment = _assignment.start != null && _assignment.goal != null;
        if (!_hasAssignment)
        {
            return;
        }
        _priorityIndex = _assignment.globalIndex;
        ApplyReservationConfigIfLeader();

        transform.position = _assignment.start.position;
        transform.rotation = _assignment.start.rotation;
        _goalPosition = _assignment.goal.position;
        _trackingGoalPosition = _goalPosition;

        if (_rb != null)
        {
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        _closestPathIndex = 0;
        _goalReached = false;
        _released = false;
        _throttle = 0f;
        _brake = 0f;
        _hasLookaheadPoint = false;
        _pathInvalid = false;
        _isFrozenByActiveCarLimit = false;
        _nextReservationDebugLogAt = 0f;
        _reservationPlanTraceLogged = false;
        _reservationRealTraceHistory.Clear();

        if (testLimitActiveCars && !IsInRandomActiveCarSubset())
        {
            FreezeInactiveCarFromTestLimit();
            return;
        }

        BuildPath();
    }

    public override void Step()
    {
        if (_released || _isFrozenByActiveCarLimit)
        {
            return;
        }

        if (_car == null || !_hasAssignment)
        {
            return;
        }

        if (_smoothedPath == null || _smoothedPath.Count < 2)
        {
            _car.Move(0f, 0f, 1f, 0f);
            return;
        }

        if (_pathInvalid || PathHasInvalidPoint(_smoothedPath))
        {
            if (!_pathInvalid)
            {
                _pathInvalid = true;
                Debug.LogError($"[AIP1TrafficCar:{GetInstanceID()}] Path contains NaN/Infinity. Disabling driving.");
            }

            _car.Move(0f, 0f, 1f, 0f);
            return;
        }

        if (_goalReached)
        {
            _car.Move(0f, 0f, 1f, 0f);
            return;
        }

        float speed = _rb != null ? _rb.linearVelocity.magnitude : 0f;
        float distToGoal = Vector3.Distance(transform.position, _trackingGoalPosition);

        if (distToGoal <= Mathf.Max(0.5f, goalStopDistance))
        {
            HandleGoalReached();
            return;
        }

        _closestPathIndex = Mathf.Clamp(_closestPathIndex, 0, _smoothedPath.Count - 1);

        float steerRef = 0f;
        if (enablePurePursuit)
        {
            steerRef = _purePursuit.ComputeSteer(
                _smoothedPath,
                transform.position,
                transform.forward,
                speed,
                _closestPathIndex,
                Mathf.Max(1f, lookaheadBase),
                Mathf.Max(0f, lookaheadPerSpeed),
                Mathf.Max(0.5f, wheelBase),
                Mathf.Max(1f, maxSteerDeg),
                out _closestPathIndex,
                out _lookaheadPoint);
            _hasLookaheadPoint = true;
        }
        else
        {
            _hasLookaheadPoint = false;
        }

        steerRef = SanitizeFinite(steerRef, 0f);
        float desiredSpeedRef = Mathf.Max(0f, maxSpeed);
        if (_speedProfile != null && _speedProfile.Length > 0)
        {
            desiredSpeedRef = SampleTargetSpeedFromProfile(
                transform.position,
                _closestPathIndex,
                out _,
                out _);
        }

        desiredSpeedRef = Mathf.Max(0f, SanitizeFinite(desiredSpeedRef, 0f));
        float targetSpeedRef = desiredSpeedRef;
        SpaceTimeReservationCoordinator.GateDecision gateDecision = default;
        if (enableReservations)
        {
            float now = Time.timeSinceLevelLoad;
            SpaceTimeReservationCoordinator coordinator = SpaceTimeReservationCoordinator.Instance;
            bool hasCoordinator = coordinator != null;
            gateDecision = coordinator.GetGateDecision(
                this,
                _closestPathIndex,
                transform.position,
                now);
            float cap = gateDecision.cap;
            targetSpeedRef = Mathf.Min(desiredSpeedRef, cap);

            bool wantGateLog = reservationDebugGateLogs;
            bool wantSpeedMatchLog = reservationDebugSpeedMatchLogs;
            bool wantPlanTraceLog = reservationDebugPlanTraceLogs;
            bool wantRealTraceLog = reservationDebugRealTraceLogs;
            if (reservationDebugVerbose &&
                (wantGateLog || wantSpeedMatchLog || wantPlanTraceLog || wantRealTraceLog) &&
                now >= _nextReservationDebugLogAt)
            {
                Vector3 p = transform.position;
                string capText = float.IsPositiveInfinity(cap) ? "inf" : $"{cap:0.###}";
                string dtRelease = float.IsNaN(gateDecision.releaseTime) ? "n/a" : $"{gateDecision.releaseTime - now:0.###}s";
                string dtSchedule = float.IsNaN(gateDecision.scheduleTime) ? "n/a" : $"{gateDecision.scheduleTime - now:0.###}s";
                string progressActualText = float.IsNaN(gateDecision.actualProgressMeters)
                    ? "n/a"
                    : $"{gateDecision.actualProgressMeters:0.###}m";
                string progressAllowedText = float.IsNaN(gateDecision.allowedProgressMeters)
                    ? "n/a"
                    : $"{gateDecision.allowedProgressMeters:0.###}m";
                string progressErrorText = float.IsNaN(gateDecision.progressErrorMeters)
                    ? "n/a"
                    : $"{gateDecision.progressErrorMeters:0.###}m";
                float aheadMeters = float.NaN;
                if (!float.IsNaN(gateDecision.actualProgressMeters) &&
                    !float.IsNaN(gateDecision.allowedProgressMeters))
                {
                    aheadMeters = gateDecision.actualProgressMeters - gateDecision.allowedProgressMeters;
                }
                string aheadText = float.IsNaN(aheadMeters) ? "n/a" : $"{aheadMeters:0.###}m";

                if (wantPlanTraceLog && !_reservationPlanTraceLogged)
                {
                    TryLogPlannedTraceOnce(coordinator, hasCoordinator, now);
                }

                if (wantGateLog)
                {
                    string line =
                        $"[AIP1TrafficCar:{GetInstanceID()}][DBG] t={now:0.###}s reason={gateDecision.reason} " +
                        $"cap={capText} desired={desiredSpeedRef:0.###} target={targetSpeedRef:0.###} speed={speed:0.###} " +
                        $"sActual={progressActualText} sAllowed={progressAllowedText} sErr={progressErrorText} ahead={aheadText} " +
                        $"pathIdx={_closestPathIndex} dtRelease={dtRelease} dtSchedule={dtSchedule} " +
                        $"thr={_throttle:0.###} brk={_brake:0.###} pos=({p.x:0.###},{p.z:0.###})";
                    if (hasCoordinator)
                    {
                        coordinator.LogExternalDebug(line);
                    }
                    else
                    {
                        Debug.Log(line);
                    }
                }

                if (wantSpeedMatchLog)
                {
                    float errToProfile = speed - desiredSpeedRef;
                    float errToTarget = speed - targetSpeedRef;
                    float relErrToTarget = Mathf.Abs(errToTarget) / Mathf.Max(0.1f, targetSpeedRef);
                    float absThreshold = Mathf.Max(0.05f, reservationDebugSpeedMismatchAbs);
                    float relThreshold = Mathf.Max(0f, reservationDebugSpeedMismatchRatio);
                    bool waitingTarget = targetSpeedRef <= 0.01f;
                    bool mismatch = waitingTarget
                        ? speed > absThreshold
                        : Mathf.Abs(errToTarget) > absThreshold && relErrToTarget > relThreshold;
                    string state = mismatch ? "MISMATCH" : "MATCH";

                    string speedLine =
                        $"[AIP1TrafficCar:{GetInstanceID()}][SPEED_MATCH] t={now:0.###}s state={state} gate={gateDecision.reason} " +
                        $"actual={speed:0.###} profile={desiredSpeedRef:0.###} target={targetSpeedRef:0.###} cap={capText} " +
                        $"sActual={progressActualText} sAllowed={progressAllowedText} sErr={progressErrorText} ahead={aheadText} " +
                        $"errProfile={errToProfile:0.###} errTarget={errToTarget:0.###} relErrTarget={relErrToTarget:0.###} " +
                        $"thrAbs={absThreshold:0.###} thrRel={relThreshold:0.###} pathIdx={_closestPathIndex} " +
                        $"dtRelease={dtRelease} dtSchedule={dtSchedule} pos=({p.x:0.###},{p.z:0.###})";
                    if (hasCoordinator)
                    {
                        coordinator.LogExternalDebug(speedLine, warning: mismatch);
                    }
                    else if (mismatch)
                    {
                        Debug.LogWarning(speedLine);
                    }
                    else
                    {
                        Debug.Log(speedLine);
                    }
                }

                if (wantRealTraceLog)
                {
                    PushRealTraceSample(now, p, speed, _closestPathIndex);
                    string historyAxis = BuildRealTraceAxisString(
                        hasCoordinator && coordinator != null ? coordinator.scenarioStartTime : 0f);

                    if (hasCoordinator && coordinator != null &&
                        coordinator.TrySamplePlannedStateAtTime(
                            this,
                            now,
                            out Vector3 plannedPos,
                            out float plannedSpeedAtNow,
                            out int plannedSeg,
                            out float segStartTime,
                            out float segEndTime))
                    {
                        float driftXZ = Vector2.Distance(
                            new Vector2(p.x, p.z),
                            new Vector2(plannedPos.x, plannedPos.z));
                        float speedDelta = speed - plannedSpeedAtNow;
                        string segWindow = float.IsNaN(segStartTime) || float.IsNaN(segEndTime)
                            ? "n/a"
                            : $"[{segStartTime - coordinator.scenarioStartTime:0.###},{segEndTime - coordinator.scenarioStartTime:0.###}]s";
                        string traceLine =
                            $"[AIP1TrafficCar:{GetInstanceID()}][REAL_TRACE] t={now:0.###}s gate={gateDecision.reason} " +
                            $"realPos=({p.x:0.###},{p.z:0.###}) plannedPos=({plannedPos.x:0.###},{plannedPos.z:0.###}) " +
                            $"driftXZ={driftXZ:0.###} speed={speed:0.###} plannedSpeed={plannedSpeedAtNow:0.###} speedDelta={speedDelta:0.###} " +
                            $"pathIdx={_closestPathIndex} plannedSeg={plannedSeg} plannedSegWindow={segWindow} " +
                            $"timeAxis={historyAxis}";
                        bool driftBad = driftXZ > Mathf.Max(2f, reservationRadius);
                        bool speedBad = Mathf.Abs(speedDelta) > Mathf.Max(0.5f, reservationDebugSpeedMismatchAbs);
                        coordinator.LogExternalDebug(traceLine, warning: driftBad || speedBad);
                    }
                    else if (hasCoordinator && coordinator != null)
                    {
                        string traceLine =
                            $"[AIP1TrafficCar:{GetInstanceID()}][REAL_TRACE] t={now:0.###}s gate={gateDecision.reason} " +
                            $"realPos=({p.x:0.###},{p.z:0.###}) speed={speed:0.###} pathIdx={_closestPathIndex} " +
                            $"plannedPos=n/a timeAxis={historyAxis}";
                        coordinator.LogExternalDebug(traceLine);
                    }
                    else
                    {
                        Debug.Log(
                            $"[AIP1TrafficCar:{GetInstanceID()}][REAL_TRACE] t={now:0.###}s gate={gateDecision.reason} " +
                            $"realPos=({p.x:0.###},{p.z:0.###}) speed={speed:0.###} pathIdx={_closestPathIndex} " +
                            $"plannedPos=n/a timeAxis={historyAxis}");
                    }
                }

                _nextReservationDebugLogAt = now + Mathf.Max(0.02f, reservationDebugIntervalSec);
            }
        }

        targetSpeedRef = Mathf.Max(0f, SanitizeFinite(targetSpeedRef, 0f));
        float steerFinal = Mathf.Clamp(SanitizeFinite(steerRef, 0f), -1f, 1f);
        if (targetSpeedRef <= 0.01f)
        {
            _throttle = 0f;
            _brake = 1f;
            _car.Move(steerFinal, 0f, 1f, 1f);
            return;
        }

        ApplySpeedControl(targetSpeedRef, speed);
        _throttle = Mathf.Clamp01(SanitizeFinite(_throttle, 0f));
        _brake = Mathf.Clamp01(SanitizeFinite(_brake, 0f));
        Debug.Assert(_brake >= 0f, $"[AIP1TrafficCar:{GetInstanceID()}] Brake must be non-negative.");
        _car.Move(steerFinal, _throttle, _brake, 0f);
    }

    private void HandleGoalReached()
    {
        if (_goalReached)
        {
            return;
        }

        _goalReached = true;
        _throttle = 0f;
        _brake = 0f;
        _car.Move(0f, 0f, 0f, 0f);

        if (!releaseOnGoalReached)
        {
            return;
        }

        ReleaseAfterGoalReached();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!reservationDebugLogCollisions)
        {
            return;
        }

        float now = Time.timeSinceLevelLoad;
        Collider other = collision != null ? collision.collider : null;
        string otherName = other != null ? other.name : "null";
        string otherRoot = other != null && other.transform != null && other.transform.root != null
            ? other.transform.root.name
            : "null";
        float relSpeed = collision != null ? collision.relativeVelocity.magnitude : 0f;
        Vector3 contact = transform.position;
        if (collision != null && collision.contactCount > 0)
        {
            contact = collision.GetContact(0).point;
        }

        float speed = _rb != null ? _rb.linearVelocity.magnitude : 0f;
        string line =
            $"[AIP1TrafficCar:{GetInstanceID()}][COLLISION] t={now:0.###}s other={otherName} root={otherRoot} " +
            $"relSpeed={relSpeed:0.###} selfSpeed={speed:0.###} pathIdx={_closestPathIndex} " +
            $"contact=({contact.x:0.###},{contact.y:0.###},{contact.z:0.###})";
        if (SpaceTimeReservationCoordinator.HasInstance)
        {
            SpaceTimeReservationCoordinator.Instance.LogExternalDebug(line, warning: true, forceConsole: true);
        }
        else
        {
            Debug.LogWarning(line);
        }
    }

    private void ReleaseAfterGoalReached()
    {
        if (_released)
        {
            return;
        }

        _released = true;
        AgentRegistry.Unregister(this);

        if (_car != null)
        {
            _car.Move(0f, 0f, 0f, 0f);
        }

        _hasLookaheadPoint = false;
        _rawPath.Clear();
        _smoothedPath.Clear();
        _speedProfile = null;
        _inflatedObstacleBounds.Clear();
        _plannerObstacleBounds.Clear();
        enabled = false;
    }

    private void ApplySpeedControl(float targetSpeed, float currentSpeed)
    {
        float speedError = targetSpeed - currentSpeed;
        float accelCmd = Mathf.Clamp(speedKp * speedError, -1f, 1f);

        float throttleCmd = Mathf.Clamp01(accelCmd);
        float brakeCmd = Mathf.Clamp01(-accelCmd);

        _throttle = Mathf.MoveTowards(_throttle, throttleCmd, Mathf.Max(0f, throttleRate) * Time.fixedDeltaTime);
        _brake = Mathf.MoveTowards(_brake, brakeCmd, Mathf.Max(0f, brakeRate) * Time.fixedDeltaTime);

        if (_brake > 0.05f)
        {
            _throttle = 0f;
        }
    }

    private void TryLogPlannedTraceOnce(
        SpaceTimeReservationCoordinator coordinator,
        bool hasCoordinator,
        float now)
    {
        if (_reservationPlanTraceLogged)
        {
            return;
        }

        if (!hasCoordinator || coordinator == null)
        {
            return;
        }

        if (!coordinator.TryGetPlannedTrajectorySnapshot(
                this,
                out Vector3[] planPath,
                out float[] planTimes,
                out float releaseTime,
                out int batchIndex))
        {
            return;
        }

        string plannedAxis = BuildPlannedAxisString(planPath, planTimes, coordinator.scenarioStartTime);
        string plannedSpeedAxis = BuildPlannedSpeedAxisString(planPath, planTimes, coordinator.scenarioStartTime);
        string line =
            $"[AIP1TrafficCar:{GetInstanceID()}][PLAN_TRACE] t={now:0.###}s release={releaseTime - coordinator.scenarioStartTime:0.###}s " +
            $"batch={batchIndex} plannedPoints={Mathf.Min(planPath.Length, planTimes.Length)} " +
            $"planAxis={plannedAxis} speedAxis={plannedSpeedAxis}";
        coordinator.LogExternalDebug(line);
        _reservationPlanTraceLogged = true;
    }

    private void PushRealTraceSample(float now, Vector3 position, float speed, int pathIndex)
    {
        int maxHistory = Mathf.Max(4, reservationDebugRealTraceHistoryCount);
        _reservationRealTraceHistory.Add(new RealTraceSample(now, position, speed, pathIndex));
        int overflow = _reservationRealTraceHistory.Count - maxHistory;
        if (overflow > 0)
        {
            _reservationRealTraceHistory.RemoveRange(0, overflow);
        }
    }

    private string BuildRealTraceAxisString(float scenarioStart)
    {
        if (_reservationRealTraceHistory.Count == 0)
        {
            return "[]";
        }

        StringBuilder sb = new StringBuilder(_reservationRealTraceHistory.Count * 40);
        sb.Append('[');
        for (int i = 0; i < _reservationRealTraceHistory.Count; i++)
        {
            if (i > 0)
            {
                sb.Append(" | ");
            }

            RealTraceSample sample = _reservationRealTraceHistory[i];
            sb.Append(i);
            sb.Append(":(");
            sb.Append(sample.pos.x.ToString("0.###"));
            sb.Append(",");
            sb.Append(sample.pos.z.ToString("0.###"));
            sb.Append(")@");
            sb.Append((sample.t - scenarioStart).ToString("0.###"));
            sb.Append("s,v=");
            sb.Append(sample.speed.ToString("0.###"));
            sb.Append(",idx=");
            sb.Append(sample.pathIndex);
        }

        sb.Append(']');
        return sb.ToString();
    }

    private static string BuildPlannedAxisString(
        IReadOnlyList<Vector3> planPath,
        IReadOnlyList<float> planTimes,
        float scenarioStart)
    {
        if (planPath == null || planTimes == null)
        {
            return "[]";
        }

        int count = Mathf.Min(planPath.Count, planTimes.Count);
        if (count <= 0)
        {
            return "[]";
        }

        StringBuilder sb = new StringBuilder(count * 36);
        sb.Append('[');
        for (int i = 0; i < count; i++)
        {
            if (i > 0)
            {
                sb.Append(" | ");
            }

            Vector3 p = planPath[i];
            sb.Append(i);
            sb.Append(":(");
            sb.Append(p.x.ToString("0.###"));
            sb.Append(",");
            sb.Append(p.z.ToString("0.###"));
            sb.Append(")@");
            sb.Append((planTimes[i] - scenarioStart).ToString("0.###"));
            sb.Append("s");
        }

        sb.Append(']');
        return sb.ToString();
    }

    private static string BuildPlannedSpeedAxisString(
        IReadOnlyList<Vector3> planPath,
        IReadOnlyList<float> planTimes,
        float scenarioStart)
    {
        if (planPath == null || planTimes == null)
        {
            return "[]";
        }

        int count = Mathf.Min(planPath.Count, planTimes.Count);
        if (count < 2)
        {
            return "[]";
        }

        StringBuilder sb = new StringBuilder((count - 1) * 40);
        sb.Append('[');
        for (int i = 0; i < count - 1; i++)
        {
            if (i > 0)
            {
                sb.Append(" | ");
            }

            float t0 = planTimes[i];
            float t1 = planTimes[i + 1];
            float dt = Mathf.Max(0.02f, t1 - t0);
            float dist = Vector3.Distance(planPath[i], planPath[i + 1]);
            float v = dist / dt;
            sb.Append(i);
            sb.Append(":v=");
            sb.Append(v.ToString("0.###"));
            sb.Append("@");
            sb.Append((t0 - scenarioStart).ToString("0.###"));
            sb.Append("->");
            sb.Append((t1 - scenarioStart).ToString("0.###"));
            sb.Append("s");
        }

        sb.Append(']');
        return sb.ToString();
    }

    private float SampleTargetSpeedFromProfile(
        Vector3 position,
        int hintClosestIndex,
        out int segmentIndex,
        out float segmentT)
    {
        segmentIndex = -1;
        segmentT = 0f;

        if (_speedProfile == null || _smoothedPath == null)
        {
            return Mathf.Max(0f, maxSpeed);
        }

        int nodeCount = Mathf.Min(_speedProfile.Length, _smoothedPath.Count);
        if (nodeCount <= 0)
        {
            return Mathf.Max(0f, maxSpeed);
        }

        if (nodeCount == 1)
        {
            segmentIndex = 0;
            return Mathf.Max(0f, _speedProfile[0]);
        }

        int searchStart = hintClosestIndex >= 0
            ? Mathf.Clamp(hintClosestIndex - 1, 0, nodeCount - 2)
            : 0;
        int searchEnd = nodeCount - 2;

        float bestSqDist = float.PositiveInfinity;
        int bestSeg = 0;
        float bestT = 0f;

        for (int i = searchStart; i <= searchEnd; i++)
        {
            Vector3 a = _smoothedPath[i];
            Vector3 b = _smoothedPath[i + 1];
            Vector3 ab = b - a;
            float abLenSq = ab.sqrMagnitude;
            float t = 0f;
            if (abLenSq > 1e-6f)
            {
                t = Mathf.Clamp01(Vector3.Dot(position - a, ab) / abLenSq);
            }

            Vector3 projected = a + ab * t;
            float sqDist = (position - projected).sqrMagnitude;
            if (sqDist < bestSqDist)
            {
                bestSqDist = sqDist;
                bestSeg = i;
                bestT = t;
            }
        }

        segmentIndex = bestSeg;
        segmentT = bestT;

        float v0 = _speedProfile[bestSeg];
        float v1 = _speedProfile[Mathf.Min(bestSeg + 1, nodeCount - 1)];
        float v0Sq = v0 * v0;
        float v1Sq = v1 * v1;
        float vSq = Mathf.Lerp(v0Sq, v1Sq, bestT);
        return Mathf.Sqrt(Mathf.Max(0f, vSq));
    }

    private void BuildPath()
    {
        _rawPath.Clear();
        _smoothedPath.Clear();
        _inflatedObstacleBounds.Clear();
        _plannerObstacleBounds.Clear();

        if (!_hasAssignment)
        {
            return;
        }

        Bounds planningBounds = ComputePlanningBounds(_assignment.start.position, _goalPosition);
        CollectInflatedObstacleBounds();
        IReadOnlyList<Bounds> plannerObstacles = enableCategoryObstacleInflation && _plannerObstacleBounds.Count > 0
            ? _plannerObstacleBounds
            : Array.Empty<Bounds>();

        GridAStarPlanner.Config cfg = new GridAStarPlanner.Config
        {
            CellSize = Mathf.Max(0.5f, gridCellSize),
            AllowDiagonal = allowDiagonal,
            MaxIterations = Mathf.Max(5000, maxAStarIterations),
            PoseFree = IsPlanningPoseFree
        };

        bool ok = _gridPlanner.Plan(
            _assignment.start.position,
            _goalPosition,
            planningBounds,
            plannerObstacles,
            cfg,
            _rawPath,
            out _);

        if (!ok || _rawPath.Count < 2)
        {
            _rawPath.Clear();
            _rawPath.Add(_assignment.start.position);
            _rawPath.Add(_goalPosition);
            _smoothedPath.Clear();
            _smoothedPath.AddRange(_rawPath);
            _trackingGoalPosition = _goalPosition;
            _speedProfile = SpeedProfiler.BuildSpeedProfile(
                _smoothedPath,
                maxSpeed,
                maxAccel,
                maxDecel,
                maxLateralAccel,
                curveSafetyFactor);
            RegisterTrajectoryForReservations();
            return;
        }

        List<Vector3> smoothed = PathSmoother.SmoothPolyline(_rawPath, _inflatedObstacleBounds);
        _smoothedPath.Clear();
        _smoothedPath.AddRange(smoothed);
        _trackingGoalPosition = _smoothedPath.Count > 0 ? _smoothedPath[_smoothedPath.Count - 1] : _goalPosition;
        _speedProfile = SpeedProfiler.BuildSpeedProfile(
            _smoothedPath,
            maxSpeed,
            maxAccel,
            maxDecel,
            maxLateralAccel,
            curveSafetyFactor);
        RegisterTrajectoryForReservations();
    }

    private void RegisterTrajectoryForReservations()
    {
        if (!_hasAssignment || _smoothedPath == null || _smoothedPath.Count < 2)
        {
            return;
        }

        _reservationPlanTraceLogged = false;
        SpaceTimeReservationCoordinator.Instance.RegisterTrajectory(
            this,
            _smoothedPath,
            _speedProfile,
            Mathf.Max(1f, maxSpeed),
            _assignment.globalIndex,
            _assignment.groupName,
            _assignment.indexWithinGroup);
    }

    private void ApplyReservationConfigIfLeader()
    {
        if (!_hasAssignment)
        {
            return;
        }

        // Apply a single global config from the first assignment to avoid multi-agent races.
        if (_assignment.globalIndex != 0)
        {
            return;
        }

        SpaceTimeReservationCoordinator coordinator = SpaceTimeReservationCoordinator.Instance;
        if (coordinator == null)
        {
            return;
        }

        coordinator.enableSpacePlanningReservation = enableReservations;
        coordinator.oddThenEvenOrdering = reservationOddThenEvenOrdering;
        coordinator.dtRes = Mathf.Max(0.05f, reservationDtRes);
        coordinator.reserveRadius = Mathf.Max(0.1f, reservationRadius);
        coordinator.horizonSec = Mathf.Max(1f, reservationHorizonSec);
        coordinator.maxTotalWaitSec = Mathf.Max(0f, reservationMaxTotalWaitSec);
        coordinator.gateTolerance = Mathf.Max(0f, reservationGateTolerance);
        float aheadCrawl = Mathf.Max(0f, reservationAheadCrawlMeters);
        coordinator.gateAheadCrawlMeters = aheadCrawl;
        coordinator.gateAheadStopMeters = Mathf.Max(aheadCrawl, reservationAheadStopMeters);
        coordinator.gateAheadCrawlSpeed = Mathf.Max(0f, reservationAheadCrawlSpeed);
        coordinator.gateProgressTimeGuardSec = Mathf.Max(0f, reservationProgressTimeGuardSec);
        coordinator.defaultNominalSpeed = Mathf.Max(1f, reservationDefaultNominalSpeed);
        coordinator.enableBatchRelease = reservationBatchRelease;
        coordinator.batchSize = Mathf.Max(1, reservationBatchSize);
        coordinator.batchIntervalSec = Mathf.Max(0f, reservationBatchIntervalSec);
        coordinator.enableWithinBatchMicroStagger = reservationEnableMicroStagger;
        coordinator.withinBatchStaggerSec = Mathf.Max(0f, reservationMicroStaggerSec);
        coordinator.forceBuildAfterSec = Mathf.Max(0f, reservationForceBuildAfterSec);
        coordinator.logReleaseSchedule = reservationLogReleaseSchedule;
        coordinator.debugVerboseLogs = reservationDebugVerbose;
        coordinator.debugPrintIntervalSec = Mathf.Max(0.02f, reservationDebugIntervalSec);
        coordinator.debugLogGateDecisions = reservationDebugVerbose && reservationDebugGateLogs;
        coordinator.debugLogScheduleBuild = reservationDebugVerbose;
        coordinator.debugLogConflictEvents = reservationDebugVerbose;
        coordinator.debugLogBatchComposition = reservationDebugVerbose;
        coordinator.debugLogReservationCommits = reservationDebugVerbose;
        coordinator.debugValidatePlannedCollisionFree = reservationDebugValidatePlannedCollisionFree;
        coordinator.debugWriteToFile = reservationDebugWriteToFile;
        coordinator.debugEchoToConsole = reservationDebugEchoConsole;
        coordinator.debugClearFileOnAwake = reservationDebugClearFileOnStart;
        coordinator.debugFileName = string.IsNullOrWhiteSpace(reservationDebugFileName)
            ? "SpaceTimeReservationDebug.txt"
            : reservationDebugFileName.Trim();
        coordinator.ReinitializeDebugLogFile(reservationDebugClearFileOnStart);
    }

    private Bounds ComputePlanningBounds(Vector3 start, Vector3 goal)
    {
        Bounds bounds = new Bounds(start, Vector3.one);
        bounds.Encapsulate(goal);
        bool hasMapBounds = false;

        if (MapManager != null)
        {
            Collider[] colliders = MapManager.transform.GetComponentsInChildren<Collider>(true);
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider col = colliders[i];
                if (col == null || !col.enabled)
                {
                    continue;
                }

                if (!hasMapBounds)
                {
                    bounds = col.bounds;
                    hasMapBounds = true;
                }
                else
                {
                    bounds.Encapsulate(col.bounds);
                }
            }

            if (!hasMapBounds)
            {
                Renderer[] renderers = MapManager.transform.GetComponentsInChildren<Renderer>(true);
                for (int i = 0; i < renderers.Length; i++)
                {
                    Renderer r = renderers[i];
                    if (r == null || !r.enabled)
                    {
                        continue;
                    }

                    if (!hasMapBounds)
                    {
                        bounds = r.bounds;
                        hasMapBounds = true;
                    }
                    else
                    {
                        bounds.Encapsulate(r.bounds);
                    }
                }
            }
        }

        if (!hasMapBounds)
        {
            Vector3 min = Vector3.Min(start, goal);
            Vector3 max = Vector3.Max(start, goal);
            bounds.SetMinMax(min, max);
        }

        bounds.Encapsulate(start);
        bounds.Encapsulate(goal);
        float pad = Mathf.Max(1f, gridCellSize);
        bounds.Expand(new Vector3(pad * 2f, 2f, pad * 2f));
        return bounds;
    }

    private void CollectInflatedObstacleBounds()
    {
        _autoObstacleMask = 0;

        if (MapManager == null)
        {
            return;
        }

        List<GameObject> obstacles = MapManager.GetObstacleObjects();
        HashSet<string> categoriesToInflate = ParseCategoryList(obstacleInflationCategories);
        float inflation = Mathf.Max(0f, categoryInflationRadius);
        bool hasCategoryFilter = categoriesToInflate.Count > 0;
        for (int i = 0; i < obstacles.Count; i++)
        {
            GameObject obj = obstacles[i];
            if (obj == null)
            {
                continue;
            }

            bool includeForPlanner = hasCategoryFilter && MatchesAnyConfiguredCategory(obj.transform, categoriesToInflate);
            bool inflateThis = includeForPlanner && inflation > 0f;

            _autoObstacleMask |= 1 << obj.layer;
            Collider[] colliders = obj.GetComponentsInChildren<Collider>(true);
            for (int c = 0; c < colliders.Length; c++)
            {
                if (colliders[c] != null)
                {
                    _autoObstacleMask |= 1 << colliders[c].gameObject.layer;
                }
            }

            Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
            for (int r = 0; r < renderers.Length; r++)
            {
                Bounds b = renderers[r].bounds;
                Bounds smoothingBounds = b;
                if (inflateThis)
                {
                    smoothingBounds.Expand(new Vector3(inflation * 2f, 4f, inflation * 2f));
                }

                _inflatedObstacleBounds.Add(smoothingBounds);

                if (includeForPlanner)
                {
                    _plannerObstacleBounds.Add(smoothingBounds);
                }
            }
        }
    }

    private static HashSet<string> ParseCategoryList(string categoriesCsv)
    {
        HashSet<string> result = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        if (string.IsNullOrWhiteSpace(categoriesCsv))
        {
            return result;
        }

        string[] tokens = categoriesCsv.Split(
            new[] { ',', ';', '\n', '\r', '\t' },
            StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < tokens.Length; i++)
        {
            string name = tokens[i].Trim();
            if (!string.IsNullOrEmpty(name))
            {
                result.Add(name);
            }
        }

        return result;
    }

    private bool MatchesAnyConfiguredCategory(Transform node, HashSet<string> categories)
    {
        if (node == null || MapManager == null || categories == null || categories.Count == 0)
        {
            return false;
        }

        Transform mapRoot = MapManager.transform;
        Transform cursor = node;
        while (cursor != null)
        {
            if (categories.Contains(cursor.name))
            {
                return true;
            }

            if (cursor == mapRoot)
            {
                break;
            }

            cursor = cursor.parent;
        }

        return false;
    }

    private bool IsPlanningPoseFree(Vector3 worldPos, float theta)
    {
        if (MapManager == null)
        {
            return true;
        }

        Vector3 pLocal = MapManager.transform.InverseTransformPoint(worldPos);
        return PoseFree(pLocal, theta);
    }

    private bool PoseFree(Vector3 pLocal, float theta)
    {
        if (MapManager == null)
        {
            return true;
        }

        float r = Mathf.Max(0.05f, inflateR);
        float pr = Mathf.Max(0.05f, r * Mathf.Clamp(probeScale, 0.3f, 1.2f));

        Vector3 cW = MapManager.transform.TransformPoint(pLocal);
        cW.y += checkY;

        int mask = ResolveObstacleMask();
        if (mask == 0)
        {
            return true;
        }

        if (Physics.CheckSphere(cW, r, mask, QueryTriggerInteraction.Ignore))
        {
            return false;
        }

        float s = Mathf.Sin(theta);
        float c = Mathf.Cos(theta);
        Vector3 fwdL = new Vector3(s, 0f, c);
        Vector3 rightL = new Vector3(c, 0f, -s);

        Vector3 fwdW = MapManager.transform.TransformDirection(fwdL);
        Vector3 rightW = MapManager.transform.TransformDirection(rightL);

        if (Physics.CheckSphere(cW + fwdW * probeFwd, pr, mask, QueryTriggerInteraction.Ignore)) return false;
        if (Physics.CheckSphere(cW - fwdW * probeFwd, pr, mask, QueryTriggerInteraction.Ignore)) return false;
        if (Physics.CheckSphere(cW + rightW * probeSide, pr, mask, QueryTriggerInteraction.Ignore)) return false;
        if (Physics.CheckSphere(cW - rightW * probeSide, pr, mask, QueryTriggerInteraction.Ignore)) return false;

        return true;
    }

    private int ResolveObstacleMask()
    {
        return obstacleMask.value != 0 ? obstacleMask.value : _autoObstacleMask;
    }

    private static float SanitizeFinite(float value, float fallback)
    {
        return float.IsNaN(value) || float.IsInfinity(value) ? fallback : value;
    }

    private static bool PathHasInvalidPoint(IReadOnlyList<Vector3> path)
    {
        if (path == null)
        {
            return true;
        }

        for (int i = 0; i < path.Count; i++)
        {
            Vector3 p = path[i];
            if (float.IsNaN(p.x) || float.IsInfinity(p.x) ||
                float.IsNaN(p.y) || float.IsInfinity(p.y) ||
                float.IsNaN(p.z) || float.IsInfinity(p.z))
            {
                return true;
            }
        }

        return false;
    }

    private bool IsInRandomActiveCarSubset()
    {
        int requested = Mathf.Max(0, testActiveCarCount);
        if (requested <= 0)
        {
            return false;
        }

        PrepareRandomActiveCarSubset(requested, testActiveCarsSeed);
        return RandomSelectedAgentIds.Contains(GetInstanceID());
    }

    private void PrepareRandomActiveCarSubset(int requestedCount, int seed)
    {
        if (s_randomSelectionReady && s_randomSelectionSeed == seed && s_randomSelectionCount == requestedCount)
        {
            return;
        }

        RandomSelectedAgentIds.Clear();
        AIP1TrafficCar[] allCars = FindObjectsByType<AIP1TrafficCar>(FindObjectsSortMode.None);
        int total = allCars != null ? allCars.Length : 0;
        int count = Mathf.Clamp(requestedCount, 0, total);
        if (total <= 0 || count <= 0)
        {
            s_randomSelectionReady = true;
            s_randomSelectionSeed = seed;
            s_randomSelectionCount = requestedCount;
            return;
        }

        int[] ids = new int[total];
        for (int i = 0; i < total; i++)
        {
            ids[i] = allCars[i] != null ? allCars[i].GetInstanceID() : 0;
        }

        System.Random rng = new System.Random(seed);
        for (int i = total - 1; i > 0; i--)
        {
            int j = rng.Next(i + 1);
            int tmp = ids[i];
            ids[i] = ids[j];
            ids[j] = tmp;
        }

        for (int i = 0; i < count; i++)
        {
            RandomSelectedAgentIds.Add(ids[i]);
        }

        s_randomSelectionReady = true;
        s_randomSelectionSeed = seed;
        s_randomSelectionCount = requestedCount;
    }

    private void FreezeInactiveCarFromTestLimit()
    {
        _isFrozenByActiveCarLimit = true;
        _goalReached = true;
        _throttle = 0f;
        _brake = 1f;
        _hasLookaheadPoint = false;
        _speedProfile = null;

        if (_rb != null)
        {
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
            if (testFreezeInactiveCars)
            {
                _rb.isKinematic = true;
                _rb.detectCollisions = false;
            }
        }

        if (_car != null)
        {
            _car.Move(0f, 0f, 1f, 0f);
            if (testFreezeInactiveCars)
            {
                _car.enabled = false;
            }
        }

        AgentRegistry.Unregister(this);
        enabled = false;
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || !_hasAssignment)
        {
            return;
        }

        if (drawRawPath && _rawPath.Count >= 2)
        {
            Gizmos.color = rawPathColor;
            for (int i = 0; i < _rawPath.Count - 1; i++)
            {
                Gizmos.DrawLine(_rawPath[i], _rawPath[i + 1]);
            }
        }

        if (drawSmoothedPath && _smoothedPath.Count >= 2)
        {
            Gizmos.color = smoothedPathColor;
            for (int i = 0; i < _smoothedPath.Count - 1; i++)
            {
                Gizmos.DrawLine(_smoothedPath[i], _smoothedPath[i + 1]);
            }
        }

        if (drawLookaheadPoint && _hasLookaheadPoint)
        {
            Gizmos.color = lookaheadColor;
            Gizmos.DrawSphere(_lookaheadPoint + Vector3.up * 0.2f, 0.7f);
            Gizmos.DrawLine(transform.position + Vector3.up * 0.3f, _lookaheadPoint + Vector3.up * 0.3f);
        }
    }

    private void OnEnable()
    {
        AgentRegistry.Register(this);
    }

    private void OnDisable()
    {
        AgentRegistry.Unregister(this);
    }

    private void OnDestroy()
    {
        AgentRegistry.Unregister(this);
    }
}
