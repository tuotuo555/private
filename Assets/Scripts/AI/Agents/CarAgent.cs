using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;

[RequireComponent(typeof(CarController), typeof(Rigidbody))]
public class CarAgent : MonoBehaviour, IAgentState
{
    [Header("Goal")]
    public Transform goal;
    public bool replanWhenGoalMoves = true;
    public float goalMoveThreshold = 0.5f;

    [Header("Static Obstacles")]
    public ObstacleQuery obstacleQuery;
    public float obstacleInflateRadius = 0.6f;
    public ObstacleQuery.FootprintProbe[] obstacleProbes;

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

    private readonly List<Pose2D> pathPoses = new List<Pose2D>(1024);
    private readonly List<Vector3> pathWorld = new List<Vector3>(1024);
    private readonly List<IAgentState> neighbors = new List<IAgentState>(32);

    private CarController car;
    private Rigidbody rb;

    private float nextPlanAllowedTime;
    private float stuckTimer;
    private float blockedTimer;
    private float collisionHoldUntil;
    private bool pendingCollisionReplan;
    private bool hasPath;
    private Vector3 lastGoalWorld;

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

    private void Awake()
    {
        car = GetComponent<CarController>();
        rb = GetComponent<Rigidbody>();

        if (obstacleQuery == null)
        {
            obstacleQuery = FindFirstObjectByType<ObstacleQuery>();
        }

        SyncConfig();
    }

    private void OnEnable()
    {
        AgentRegistry.Instance.Register(this);
    }

    private void Start()
    {
        if (goal != null)
        {
            lastGoalWorld = goal.position;
        }

        PlanPath();
    }

    private void OnDisable()
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

    private void FixedUpdate()
    {
        if (car == null || rb == null || goal == null)
        {
            return;
        }

        float dt = Mathf.Max(0.01f, Time.fixedDeltaTime);

        if (Time.time < collisionHoldUntil)
        {
            car.Move(0f, 0f, 1f, 0f);
            return;
        }

        bool needPlan = !hasPath || follower.PathCount < 2;

        if (!needPlan && replanWhenGoalMoves)
        {
            if (Vector3.Distance(goal.position, lastGoalWorld) > Mathf.Max(0.05f, goalMoveThreshold))
            {
                needPlan = true;
            }
        }

        if (pendingCollisionReplan)
        {
            needPlan = true;
        }

        if (needPlan && Time.time >= nextPlanAllowedTime)
        {
            PlanPath();
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
        float accelRef = throttleRef * Mathf.Max(0.1f, speedController.maxAccel) - brakeRef * Mathf.Max(0.1f, speedController.maxBrake);

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
    }

    public bool PlanPath()
    {
        if (goal == null || obstacleQuery == null)
        {
            hasPath = false;
            return false;
        }

        SyncConfig();

        Pose2D start = Pose2D.FromWorld(transform);
        float goalYaw = Mathf.Atan2(goal.forward.x, goal.forward.z);
        Pose2D goalPose = new Pose2D(new Vector2(goal.position.x, goal.position.z), goalYaw);

        bool success = planner.Plan(start, goalPose, pathPoses);
        if (!success || pathPoses.Count < 2)
        {
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
        pathWorld[pathWorld.Count - 1] = goal.position;

        follower.SetPath(pathWorld);

        hasPath = true;
        pendingCollisionReplan = false;
        stuckTimer = 0f;
        blockedTimer = 0f;
        nextPlanAllowedTime = Time.time + Mathf.Max(0.1f, replanCooldown);
        lastGoalWorld = goal.position;
        return true;
    }

    private void UpdateReplanningHeuristics(float dt)
    {
        if (goal == null)
        {
            stuckTimer = 0f;
            blockedTimer = 0f;
            return;
        }

        if (Speed < Mathf.Max(0.05f, stuckSpeedThreshold))
        {
            stuckTimer += dt;
        }
        else
        {
            stuckTimer = 0f;
        }

        AgentRegistry.Instance.GetNeighbors(this, blockedAheadDistance, neighbors);
        if (IsBlockedByStoppedCar(neighbors))
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
            PlanPath();
        }
    }

    private bool IsBlockedByStoppedCar(List<IAgentState> nearAgents)
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
}
