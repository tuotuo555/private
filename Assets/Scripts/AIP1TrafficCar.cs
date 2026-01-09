using System.Collections.Generic;
using System.Linq;
using FormationGame;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : Agent
{
    public CarController car; // the car controller we want to use. Assigned in prefab
    
    private GameObject[] _mOtherCars;
    private List<MultiVehicleGoal> _mCurrentGoals;

    public bool drawTargets;
    public bool drawAllCars;
    public bool drawTeamCars;

    public float steering;
    public float acceleration;
    public List<GameObject> targetObjects;
    public List<GameObject> teamVehicles;

    public override void Initialize()
    {
        var gameManagerA2 = FindFirstObjectByType<GameManagerA2>();
        
        _mCurrentGoals = gameManagerA2.GetGoals(gameObject); // This car's goals. Can be multiple per vehicle!
        teamVehicles = gameManagerA2.GetGroupVehicles(gameObject); //Other vehicles in a Group with this vehicle
        _mOtherCars = GameObject.FindGameObjectsWithTag("Player"); //All vehicles
        
        // Note that this array will have "holes" when objects are destroyed
        // But for initial planning they should work
        // If you dont like the "holes", you can re-fetch this during fixed update.

        // Where to go?
       
        // Equivalent ways to find all the targets in the scene
        targetObjects = _mCurrentGoals.Select(goal => goal.GetTargetObject()).ToList();

        // You can also fetch other types of objects using tags, assuming the objects you are looking for have tags assigned :).

        // Feel free to refer to any examples from previous assignments.
    }


    public override void Step()
    {
        // Execute your path and collision checking here
        // ...

        // Feel free to refer to any examples from previous assignments.

        //Example of cars moving into the centre of the field.
        Vector3 avg_pos = _mOtherCars.Aggregate(Vector3.zero, (sum, car) => sum + car.transform.position) / _mOtherCars.Length;


        Vector3 goal_pos = targetObjects[0].transform.position;
        
        (steering, acceleration) = ControlsTowardsPoint(avg_pos);
        (steering, acceleration) = ControlsTowardsPoint(goal_pos);

        car.Move(steering, acceleration, acceleration, 0f);
    }

    private (float steering, float acceleration) ControlsTowardsPoint(Vector3 avg_pos)
    {
        Vector3 direction = (avg_pos - transform.position).normalized;

        bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
        bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

        float steering = 0f;
        float acceleration = 0;

        if (is_to_the_right && is_to_the_front)
        {
            steering = 1f;
            acceleration = 1f;
        }
        else if (is_to_the_right && !is_to_the_front)
        {
            steering = -1f;
            acceleration = -1f;
        }
        else if (!is_to_the_right && is_to_the_front)
        {
            steering = -1f;
            acceleration = 1f;
        }
        else if (!is_to_the_right && !is_to_the_front)
        {
            steering = 1f;
            acceleration = -1f;
        }

        float alpha = Mathf.Asin(Vector3.Dot(direction, transform.right));
        if (is_to_the_front && Mathf.Abs(alpha) < 1f)
        {
            steering = alpha;
        }

        return (steering, acceleration);
    }


    private void Update()
    {
        if (drawTargets)
        {
            foreach (var item in targetObjects)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.red);
            }
        }

        if (drawTeamCars)
        {
            foreach (var item in teamVehicles)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.blue);
            }
        }

        if (drawAllCars)
        {
            foreach (var item in _mOtherCars)
            {
                Debug.DrawLine(transform.position, item.transform.position, Color.yellow);
            }
        }
        //Debug.DrawLine(Vector3.zero, new Vector3(1, 0, 0), Color.red);
    }
}