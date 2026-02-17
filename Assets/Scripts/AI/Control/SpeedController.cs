using UnityEngine;

[System.Serializable]
public class SpeedController
{
    [Header("Throttle/Brake Gains")]
    public float throttleGain = 0.35f;
    public float brakeGain = 0.45f;

    [Header("Longitudinal Limits")]
    public float maxAccel = 3f;
    public float maxBrake = 6f;

    public void ComputeThrottleBrake(float targetSpeed, float currentSpeed, out float throttle, out float brake)
    {
        float speedErr = targetSpeed - currentSpeed;

        if (speedErr > 0f)
        {
            throttle = Mathf.Clamp01(throttleGain * speedErr);
            brake = 0f;
            return;
        }

        throttle = 0f;
        brake = Mathf.Clamp01(brakeGain * -speedErr);
    }

    public float ComputeAccelReference(float targetSpeed, float currentSpeed)
    {
        float speedErr = targetSpeed - currentSpeed;
        if (speedErr >= 0f)
        {
            return Mathf.Clamp(speedErr * throttleGain * Mathf.Max(0.1f, maxAccel), 0f, Mathf.Max(0.1f, maxAccel));
        }

        return -Mathf.Clamp(-speedErr * brakeGain * Mathf.Max(0.1f, maxBrake), 0f, Mathf.Max(0.1f, maxBrake));
    }
}
