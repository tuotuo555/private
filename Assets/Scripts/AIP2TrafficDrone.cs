using System.Threading;
using Scripts.Game;
using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : Agent
{
    public DroneController mDrone;
    
    public override void Initialize()
    {
        // See AIP1TrafficCar for how to get info from the world.
    }


    public override void Step()
    {
        mDrone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
    }
}