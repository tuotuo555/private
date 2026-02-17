using System.Collections.Generic;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController), typeof(Rigidbody))]
public class AIP1TrafficCar : Agent, IAgentState
{
    [Header("Optional Direct Goal")]
    public Transform goalOverride;

    [Header("Goal Refresh")]
    public float goalRefreshPeriod = 0.5f;
    public float goalRefreshDistance = 0.5f;

    [Header("Static Obstacles")]
    public ObstacleQuery obstacleQuery;
    public float obstacleInflateRadius = 0.6f;
    public ObstacleQuery.FootprintProbe[] obstacleProbes;
    public bool autoCreateObstacleQueryIfMissing = true;
    public LayerMask fallbackObstacleMask = 0;
    public float fallbackObstacleCheckY = 0.1f;

    [Header("Planner")]
    public HybridAStarPlanner planner = new HybridAStarPlanner();

    [Header("Follower")]
    public PurePursuitFollower follower = new PurePursuitFollower();

    [Header("Speed Controller")]
    public SpeedController speedController = new SpeedController();

    [Header("Safety Filter")]
    public SafetyFilter safetyFilter = new SafetyFilter();

    [Header("Replanning")]
    public float replanCooldown = 0.75f;
    public float stuckSpeedThreshold = 0.5f;
    public float tStuck = 2.5f;
    public float blockedAheadDistance = 6f;
    public float blockedNeighborSpeed = 0.3f;
    public float blockedReplanDelay = 1.25f;
    public float collisionHoldTime = 0.35f;

    [Header("Debug")]
    public bool drawPath = true;
    public bool logPathStatus = true;

    public CarController car;
    private Rigidbody rb;

    private readonly List<Pose2D> pathPoses = new List<Pose2D>(1024);
    private readonly List<Vector3> pathWorld = new List<Vector3>(1024);
    private readonly List<IAgentState> neighbors = new List<IAgentState>(32);

    private float nextPlanAllowedTime;
    private float nextGoalRefreshTime;
    private float stuckTimer;
    private float blockedTimer;
    private float collisionHoldUntil;
    private bool pendingCollisionReplan;
    private bool hasPath;
    private bool isInitialized;
    private int lastStepFrame = -1;
    private bool loggedMissingObstacleQuery;

    private Vector3 currentGoalWorld;
    private bool hasVehicleSpecificGoal;

    public Transform AgentTransform => transform;
    public bool IsAgentActive => isActiveAndEnabled;

    public Vector2 PlanarPosition
    {
        get
        {
            Vector3 p = transform.position;
            return new Vector2(p.x, p.z);
        }
    }

    public Vector2 PlanarVelocity
    {
        get
        {
            if (rb == null)
            {
                return Vector2.zero;
            }

            Vector3 v = rb.linearVelocity;
            return new Vector2(v.x, v.z);
        }
    }

    public float Speed => PlanarVelocity.magnitude;

    public override void Initialize()
    {
        EnsureInitialized();
    }

    private void Start()
    {
        EnsureInitialized();
    }

    private void FixedUpdate()
    {
        EnsureInitialized();
        if (!isInitialized)
        {
            return;
        }

        Step();
    }

    private void EnsureInitialized()
    {
        if (isInitialized)
        {
            return;
        }

        if (!car) car = GetComponent<CarController>();
        rb = GetComponent<Rigidbody>();

        if (obstacleQuery == null)
        {
            obstacleQuery = FindFirstObjectByType<ObstacleQuery>();
        }

        if (obstacleQuery == null && autoCreateObstacleQueryIfMissing)
        {
            obstacleQuery = CreateRuntimeObstacleQuery();
            if (logPathStatus)
            {
                Debug.LogWarning(
                    $"[{name}] ObstacleQuery was missing. Created runtime fallback with mask={fallbackObstacleMask.value}. " +
                    "Set a proper ObstacleQuery in scene for static obstacle avoidance.",
                    this);
            }
        }

        if (car == null || rb == null)
        {
            if (logPathStatus)
            {
                Debug.LogWarning($"[{name}] Missing CarController or Rigidbody. AI disabled.", this);
            }

            return;
        }

        SyncConfig();
        follower.ResetState();
        safetyFilter.ResetState();

        AgentRegistry.Instance.Register(this);

        currentGoalWorld = ResolveGoalWorldPosition(out hasVehicleSpecificGoal);
        PlanPath(currentGoalWorld);
        nextGoalRefreshTime = Time.time + Mathf.Max(0.1f, goalRefreshPeriod);
        isInitialized = true;

        if (logPathStatus)
        {
            Debug.Log($"[{name}] AIP1TrafficCar initialized. obstacleQuery={(obstacleQuery != null ? obstacleQuery.name : "NULL")}", this);
        }
    }

    private void OnDestroy()
    {
        if (AgentRegistry.HasInstance)
        {
            AgentRegistry.Instance.Unregister(this);
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        pendingCollisionReplan = true;
        collisionHoldUntil = Time.time + Mathf.Max(0.05f, collisionHoldTime);
    }

    public override void Step()
    {
        if (lastStepFrame == Time.frameCount)
        {
            return;
        }

        lastStepFrame = Time.frameCount;

        if (car == null || rb == null)
        {
            return;
        }

        float dt = Mathf.Max(0.01f, Time.deltaTime);

        if (Time.time >= nextGoalRefreshTime)
        {
            nextGoalRefreshTime = Time.time + Mathf.Max(0.1f, goalRefreshPeriod);
            TryRefreshGoalAndReplan();
        }

        if (Time.time < collisionHoldUntil)
        {
            car.Move(0f, 0f, 1f, 0f);
            return;
        }

        if ((!hasPath || follower.PathCount < 2 || pendingCollisionReplan) && Time.time >= nextPlanAllowedTime)
        {
            PlanPath(currentGoalWorld);
            pendingCollisionReplan = false;
        }

        Pose2D pose = Pose2D.FromWorld(transform);
        float speed = Speed;
        PurePursuitFollower.ControlOutput follow = follower.UpdateControl(pose, speed);

        if (!follow.hasPath)
        {
            hasPath = false;
            car.Move(0f, 0f, 1f, 0f);
            return;
        }

        if (follow.reachedGoal)
        {
            car.Move(0f, 0f, 1f, 0f);
            stuckTimer = 0f;
            blockedTimer = 0f;
            return;
        }

        speedController.ComputeThrottleBrake(follow.targetSpeed, speed, out float throttleRef, out float brakeRef);
        float accelRef = throttleRef * Mathf.Max(0.1f, speedController.maxAccel)
            - brakeRef * Mathf.Max(0.1f, speedController.maxBrake);

        AgentRegistry.Instance.GetNeighbors(this, safetyFilter.neighborRadius, neighbors);

        SafetyFilter.FilterInput filterInput = new SafetyFilter.FilterInput
        {
            pose = pose,
            velocity = PlanarVelocity,
            speed = speed,
            steerRef = follow.steerCmd,
            accelRef = accelRef
        };

        SafetyFilter.FilterOutput safe = safetyFilter.Filter(
            filterInput,
            this,
            neighbors,
            obstacleQuery,
            obstacleInflateRadius,
            obstacleProbes,
            dt);

        car.Move(safe.steer, safe.throttle, safe.brake, 0f);

        UpdateReplanningHeuristics(dt);

        if (drawPath)
        {
            DrawPathDebug();
        }
    }

    private bool PlanPath(Vector3 goalWorld)
    {
        if (obstacleQuery == null)
        {
            if (logPathStatus && !loggedMissingObstacleQuery)
            {
                Debug.LogWarning($"[{name}] Path NOT found. ObstacleQuery is missing in scene.", this);
                loggedMissingObstacleQuery = true;
            }

            hasPath = false;
            return false;
        }

        loggedMissingObstacleQuery = false;

        SyncConfig();

        Pose2D start = Pose2D.FromWorld(transform);
        Pose2D goalPose = BuildGoalPose(goalWorld, start);

        bool success = planner.Plan(start, goalPose, pathPoses);
        if (!success || pathPoses.Count < 2)
        {
            if (logPathStatus)
            {
                Debug.LogWarning($"[{name}] Path NOT found. start={transform.position} goal={goalWorld}", this);
            }

            hasPath = false;
            nextPlanAllowedTime = Time.time + Mathf.Max(0.1f, replanCooldown);
            return false;
        }

        pathWorld.Clear();
        float y = transform.position.y;

        for (int i = 0; i < pathPoses.Count; i++)
        {
            pathWorld.Add(Pose2D.ToWorldXZ(pathPoses[i].pos, y));
        }

        pathWorld[0] = transform.position;
        pathWorld[pathWorld.Count - 1] = goalWorld;

        follower.SetPath(pathWorld);

        hasPath = true;
        pendingCollisionReplan = false;
        stuckTimer = 0f;
        blockedTimer = 0f;
        nextPlanAllowedTime = Time.time + Mathf.Max(0.1f, replanCooldown);

        if (logPathStatus)
        {
            Debug.Log($"[{name}] Path found with {pathPoses.Count} poses. goal={goalWorld}", this);
        }

        return true;
    }

    private void TryRefreshGoalAndReplan()
    {
        Vector3 newGoal = ResolveGoalWorldPosition(out bool fromVehicleGoal);

        if (!fromVehicleGoal && hasVehicleSpecificGoal && goalOverride == null)
        {
            return;
        }

        bool changed = Vector3.Distance(newGoal, currentGoalWorld) > Mathf.Max(0.05f, goalRefreshDistance);
        bool upgradedGoalSource = fromVehicleGoal && !hasVehicleSpecificGoal;

        if (!changed && !upgradedGoalSource)
        {
            return;
        }

        hasVehicleSpecificGoal = fromVehicleGoal;
        currentGoalWorld = newGoal;

        if (Time.time >= nextPlanAllowedTime)
        {
            PlanPath(currentGoalWorld);
        }
        else
        {
            pendingCollisionReplan = true;
        }
    }

    private void UpdateReplanningHeuristics(float dt)
    {
        if (Speed < Mathf.Max(0.05f, stuckSpeedThreshold))
        {
            stuckTimer += dt;
        }
        else
        {
            stuckTimer = 0f;
        }

        AgentRegistry.Instance.GetNeighbors(this, blockedAheadDistance, neighbors);
        if (IsBlockedByStoppedAgent(neighbors))
        {
            blockedTimer += dt;
        }
        else
        {
            blockedTimer = 0f;
        }

        bool shouldReplan = stuckTimer >= Mathf.Max(0.1f, tStuck)
            || blockedTimer >= Mathf.Max(0.1f, blockedReplanDelay);

        if (shouldReplan && Time.time >= nextPlanAllowedTime)
        {
            PlanPath(currentGoalWorld);
        }
    }

    private bool IsBlockedByStoppedAgent(List<IAgentState> nearAgents)
    {
        if (nearAgents == null || nearAgents.Count == 0)
        {
            return false;
        }

        float yaw = Pose2D.FromWorld(transform).yawRad;
        Vector2 myFwd = Math2D.Forward(yaw);
        Vector2 myPos = PlanarPosition;

        for (int i = 0; i < nearAgents.Count; i++)
        {
            IAgentState other = nearAgents[i];
            if (other == null || other == this || !other.IsAgentActive)
            {
                continue;
            }

            if (other.Speed > blockedNeighborSpeed)
            {
                continue;
            }

            Vector2 rel = other.PlanarPosition - myPos;
            float d = rel.magnitude;
            if (d < 0.001f || d > blockedAheadDistance)
            {
                continue;
            }

            Vector2 dir = rel / d;
            if (Vector2.Dot(myFwd, dir) > 0.6f)
            {
                return true;
            }
        }

        return false;
    }

    private Pose2D BuildGoalPose(Vector3 goalWorld, Pose2D startPose)
    {
        float yaw = startPose.yawRad;

        if (goalOverride != null)
        {
            Vector3 gf = goalOverride.forward;
            yaw = Mathf.Atan2(gf.x, gf.z);
        }
        else
        {
            Vector2 toGoal = new Vector2(goalWorld.x - startPose.pos.x, goalWorld.z - startPose.pos.y);
            if (toGoal.sqrMagnitude > 1e-6f)
            {
                yaw = Mathf.Atan2(toGoal.x, toGoal.y);
            }
        }

        return new Pose2D(new Vector2(goalWorld.x, goalWorld.z), yaw);
    }

    private Vector3 ResolveGoalWorldPosition(out bool fromVehicleGoal)
    {
        fromVehicleGoal = false;

        if (goalOverride != null)
        {
            fromVehicleGoal = true;
            return goalOverride.position;
        }

        GameManagerA2 gmA2 = FindFirstObjectByType<GameManagerA2>();
        if (gmA2 != null)
        {
            List<MultiVehicleGoal> myGoals = gmA2.GetGoals(gameObject);
            if (myGoals != null && myGoals.Count > 0)
            {
                GameObject bestTarget = null;
                float bestD2 = float.PositiveInfinity;

                for (int i = 0; i < myGoals.Count; i++)
                {
                    GameObject target = myGoals[i].GetTargetObject();
                    if (target == null) continue;

                    Vector3 d = target.transform.position - transform.position;
                    float d2 = d.sqrMagnitude;
                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        bestTarget = target;
                    }
                }

                if (bestTarget != null)
                {
                    fromVehicleGoal = true;
                    return bestTarget.transform.position;
                }
            }
        }

        return MapManager.GetGlobalGoalPosition();
    }

    private void SyncConfig()
    {
        planner.obstacleQuery = obstacleQuery;
        planner.inflateRadius = obstacleInflateRadius;
        planner.footprintProbes = obstacleProbes;

        follower.wheelBase = planner.wheelBase;
        follower.maxSteerDeg = planner.maxSteerDeg;

        safetyFilter.wheelBase = planner.wheelBase;
        safetyFilter.maxSteerDeg = planner.maxSteerDeg;
        safetyFilter.maxAccel = speedController.maxAccel;
        safetyFilter.maxBrake = speedController.maxBrake;
    }

    private void DrawPathDebug()
    {
        if (pathWorld.Count < 2)
        {
            return;
        }

        for (int i = 1; i < pathWorld.Count; i++)
        {
            Debug.DrawLine(pathWorld[i - 1] + Vector3.up * 0.2f, pathWorld[i] + Vector3.up * 0.2f, Color.cyan);
        }
    }

    private ObstacleQuery CreateRuntimeObstacleQuery()
    {
        GameObject go = new GameObject("RuntimeObstacleQuery");
        ObstacleQuery q = go.AddComponent<ObstacleQuery>();
        q.obstacleMask = fallbackObstacleMask;
        q.checkY = fallbackObstacleCheckY;
        return q;
    }
}
