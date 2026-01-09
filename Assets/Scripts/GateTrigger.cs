using UnityEngine;

namespace FormationGame
{
    public class GateTrigger : MonoBehaviour
    {
        public bool hasBeenPassed;

        void OnTriggerEnter(Collider other)
        {
            if (!hasBeenPassed)
            {
                var componentInParent = transform.parent.GetComponentInParent<Renderer>();
                componentInParent.materials[2].color = Color.yellow;
            }

            hasBeenPassed = true;
        }
    }
}