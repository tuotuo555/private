using UnityEngine;

public interface IAgentState
{
    Transform AgentTransform { get; }
    Vector2 PlanarPosition { get; }
    Vector2 PlanarVelocity { get; }
    float Speed { get; }
    bool IsAgentActive { get; }
}
