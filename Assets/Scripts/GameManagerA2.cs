using System;
using System.Collections.Generic;
using System.Linq;
using FormationGame;
using NUnit.Framework;
using Scripts.Game;
using Scripts.Utils;
using UnityEngine;

public class GameManagerA2 : AbstractGameManager
{
    public float maxDistance = 5;
    private Dictionary<string, List<MultiVehicleGoal>> goalsByGroup;
    private Dictionary<string, List<GameObject>> vehiclesByGroup;
    private static string UNGROUPED_KEY = "Free";

    public override List<Goal> CreateGoals(List<GameObject> vehicles)
    {
        var targets = mapManager.transform.FindAllChildrenWithTag("Target");
        var starts = mapManager.transform.FindAllChildrenWithTag("Start");

        vehiclesByGroup = vehicles
            .Select((vehicle, index) => new { Vehicle = vehicle, Start = starts[index] })
            .GroupBy(pair => pair.Start.transform.parent.name)
            .ToDictionary(
                group => CreateKey(group.Key),
                group => group.Select(pair => pair.Vehicle).ToList()
            );

        goalsByGroup = targets.GroupBy(target => target.transform.parent.name)
            .ToDictionary(
                group => CreateKey(group.Key),
                group => group.Select(target => CreateLoSGoal(target)).ToList()
            ).ToDictionary(
                pair => CreateKey(pair.Key),
                pair => pair.Value.Select(goal => new MultiVehicleGoal(goal, vehiclesByGroup.GetValueOrDefault(pair.Key, new List<GameObject>()))).ToList()
            );

        if (goalsByGroup.ContainsKey(UNGROUPED_KEY) && vehiclesByGroup.ContainsKey(UNGROUPED_KEY) && goalsByGroup[UNGROUPED_KEY].Count == vehiclesByGroup[UNGROUPED_KEY].Count)
        {
            for (int i = 0; i < goalsByGroup[UNGROUPED_KEY].Count; i++)
            {
                var vehiclesByUngrouped = vehiclesByGroup[UNGROUPED_KEY];
                var goalsByUngrouped = goalsByGroup[UNGROUPED_KEY];
                goalsByUngrouped[i].ClearVehicles();
                goalsByUngrouped[i].AssignVehicle(vehiclesByUngrouped[i]);
            }
        }

        goalsByGroup.ToList()
        .FindAll(pair => pair.Key != UNGROUPED_KEY)
        .ForEach(pair => pair.Value.ForEach(goal => goal.GetTargetObject().GetComponent<GoalColorIndicator>().SetByIndex(vehiclesByGroup.Keys.ToList().IndexOf(pair.Key))));

        vehiclesByGroup.ToList()
        .FindAll(pair => pair.Key != UNGROUPED_KEY)
          .ForEach(pair => pair.Value.ForEach(vehicle => vehicle.GetComponent<ColorIndicator>().SetByIndex(vehiclesByGroup.Keys.ToList().IndexOf(pair.Key))));

        return goalsByGroup.Values
            .SelectMany(group => group.ToList())
            .Select(multi => (Goal)multi)
            .ToList();
    }

    public string CreateKey(string key)
    {
        if (key == "Targets") return UNGROUPED_KEY;
        if (key == "Starts") return UNGROUPED_KEY;
        return key;
    }

    private LineOfSightGoal CreateLoSGoal(GameObject target)
    {
        var goal = new LineOfSightGoal(target, maxDistance);

        var goalColorIndicator = target.GetComponent<GoalColorIndicator>();
        if (goalColorIndicator != null) goalColorIndicator.SetGoal(goal);

        return goal;
    }
    
    public override bool IsDone()
    {
        foreach (var vehicle in vehicleList)
        {
            foreach (var goal in goals)
            {
                goal.CheckAchieved(vehicle);
            }
        }

        return goals.ToList().TrueForAll(goal => goal.IsAchieved());
    }

    public List<MultiVehicleGoal> GetGoals(GameObject vehicle)
    {
        return goals
        .FindAll(goal => ((MultiVehicleGoal)goal).ContainsVehicle(vehicle))
        .Select(goal => (MultiVehicleGoal)goal).ToList();
    }

    public string GetGroup(GameObject vehicle)
    {
        return vehiclesByGroup.FirstOrDefault(x => x.Value.Contains(vehicle)).Key;
    }

    public List<GameObject> GetGroupVehicles(GameObject vehicle)
    {
        var team = GetGroup(vehicle);
        if (team == UNGROUPED_KEY) return new List<GameObject>() { vehicle };
        return vehiclesByGroup[team];
    }
}