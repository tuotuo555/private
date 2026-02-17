using System.Collections.Generic;
using UnityEngine;

public class AgentRegistry : MonoBehaviour
{
    private static AgentRegistry instance;

    public static bool HasInstance => instance != null;

    public static AgentRegistry Instance
    {
        get
        {
            if (instance == null)
            {
                instance = FindFirstObjectByType<AgentRegistry>();
                if (instance == null)
                {
                    GameObject go = new GameObject("AgentRegistry");
                    instance = go.AddComponent<AgentRegistry>();
                }
            }

            return instance;
        }
    }

    private readonly List<IAgentState> agents = new List<IAgentState>(64);

    public IReadOnlyList<IAgentState> Agents => agents;

    private void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
    }

    public void Register(IAgentState agent)
    {
        if (IsDead(agent) || agents.Contains(agent))
        {
            return;
        }

        agents.Add(agent);
    }

    public void Unregister(IAgentState agent)
    {
        if (agent == null)
        {
            return;
        }

        agents.Remove(agent);
    }

    public void GetNeighbors(IAgentState self, float radius, List<IAgentState> result)
    {
        result.Clear();
        if (IsDead(self))
        {
            return;
        }

        float r = Mathf.Max(0.01f, radius);
        float r2 = r * r;
        Vector3 selfPos = self.AgentTransform.position;

        for (int i = 0; i < agents.Count; i++)
        {
            IAgentState other = agents[i];
            if (IsDead(other) || other == self || !other.IsAgentActive)
            {
                continue;
            }

            if ((other.AgentTransform.position - selfPos).sqrMagnitude <= r2)
            {
                result.Add(other);
            }
        }
    }

    private static bool IsDead(IAgentState agent)
    {
        if (agent == null)
        {
            return true;
        }

        Object o = agent as Object;
        return o == null || agent.AgentTransform == null;
    }
}
