using System;
using System.Collections.Generic;
using Scripts.Game;
using UnityEngine;

namespace FormationGame
{
    public class MultiVehicleGoal : Goal
    {
        private readonly Goal trackedGoal;
        private readonly List<GameObject> teamVehicles;

        public MultiVehicleGoal(Goal underlying, List<GameObject> objects)
        {
            trackedGoal = underlying;
            teamVehicles = new List<GameObject>(objects);
        }

        public bool CheckAchieved(GameObject objectToCheck)
        {
            if (teamVehicles.Contains(objectToCheck))
            {
                return trackedGoal.CheckAchieved(objectToCheck);
            }

            return false;
        }

        public bool IsAchieved()
        {
            return trackedGoal.IsAchieved();
        }

        public void RestartTimer()
        {
            trackedGoal.RestartTimer();
        }

        public float CurrentTime()
        {
            return trackedGoal.CurrentTime();
        }

        internal void AssignVehicle(GameObject gameObject)
        {
            teamVehicles.Add(gameObject);
        }

        internal void ClearVehicles()
        {
            teamVehicles.Clear();
        }

        public GameObject GetTargetObject()
        {
            return trackedGoal.GetTargetObject();
        }

        public bool ContainsVehicle(GameObject vehicle)
        {
            return teamVehicles.Contains(vehicle);
        }
    }
}