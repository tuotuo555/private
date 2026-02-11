using UnityEngine;

public class ObstacleAI : MonoBehaviour
{
    public bool isKinematic = true;
    public float speed = 1.3f;
    public float force = 1f;
    public Vector3 direction;
    public float characterRadius = 1f;
    private Rigidbody myRB;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        myRB = GetComponent<Rigidbody>();
        direction = new Vector3(0.7f, 0f, 0.7f);
    }

    // Update is called once per frame
    void Update()
    {
        if (myRB == null)
        {
            Start();
        }
        if (ObstacleInFront())
            {
                SetNewDirection();
            }
        
        
        transform.LookAt(transform.position + direction);
        if (isKinematic)
        {
            transform.position += direction * Time.deltaTime * speed;
        }
        else myRB.AddForce(force * direction);
    }

    bool ObstacleInFront()
    {
        RaycastHit hit;

        Vector3 p1 = transform.position + direction;
        float distanceToObstacle = 0;

        // Cast a sphere wrapping character controller some meters forward
        // to see if it is about to hit anything.
        if (Physics.SphereCast(p1, characterRadius, direction, out hit, 20))
        {
            distanceToObstacle = hit.distance;
            //Debug.Log("Distance before turning: " + (distanceToObstacle - 2f));
            Debug.DrawLine(transform.position, transform.position + direction * distanceToObstacle, Color.red, 0.1f);
            if (distanceToObstacle < 2f)
            {
                return true;
            }
        }
        return false;
    }

    void SetNewDirection()
    {
        
        direction = Vector3.Cross(direction, Vector3.up);
        if (Random.Range(0f, 1f) > 0.5f)
        {
            direction = -1f * direction;
        }
        
    }
}
