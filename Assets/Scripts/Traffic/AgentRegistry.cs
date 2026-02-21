using System.Collections.Generic;
using Scripts.Game;

namespace Scripts.Traffic
{
    public static class AgentRegistry
    {
        private static readonly List<Agent> ActiveAgentsInternal = new List<Agent>();
        private static readonly List<AIP1TrafficCar> AllAgentsInternal = new List<AIP1TrafficCar>();

        public static IReadOnlyList<Agent> ActiveAgents => ActiveAgentsInternal;
        public static IReadOnlyList<AIP1TrafficCar> AllAgents => AllAgentsInternal;

        public static void Register(Agent agent)
        {
            if (agent == null || ActiveAgentsInternal.Contains(agent))
            {
                return;
            }

            ActiveAgentsInternal.Add(agent);
            if (agent is AIP1TrafficCar car && !AllAgentsInternal.Contains(car))
            {
                AllAgentsInternal.Add(car);
            }
        }

        public static void Unregister(Agent agent)
        {
            if (agent == null)
            {
                return;
            }

            ActiveAgentsInternal.Remove(agent);
            if (agent is AIP1TrafficCar car)
            {
                AllAgentsInternal.Remove(car);
            }
        }
    }
}
