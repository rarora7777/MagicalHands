//////////////////////////////////////////////////////////////////////////////////////
/// ShelfObject is a MonoBehaviour that should be applied to "creator" objects placed
/// in the MagicalHands object shelf. The main task is the creation and destruction of
/// Animatable objects.
//////////////////////////////////////////////////////////////////////////////////////

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace GestureAnim
{
    [RequireComponent(typeof(Collider), typeof(Rigidbody))]
    public class ShelfObject : MonoBehaviour
    {
        private DateTime lastObjectDestroyTime = DateTime.MinValue;
        public GameObject AnimatablePrefab;
        public GameObject ParticlePrefab;

        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public Animatable InstantiateAnimatable()
        {
            GameObject obj = (GameObject)Instantiate(AnimatablePrefab);

            if (obj == null)
            {
                Debug.LogWarning("Cannot find the Prefab for " + name);
                return null;
            }
            else if(obj.GetComponent<Animatable>() == null)
            {
                Debug.LogWarning("No Animatable Component attached on the prefab.");
                return null;
            }

            return obj.GetComponent<Animatable>();
        }

        private void OnTriggerEnter(Collider other)
        {
            if ((DateTime.Now - lastObjectDestroyTime).TotalSeconds < Globals.SHELF_GESTURE_WAITING_PERIOD)
                return;

            var animScript = other.GetComponent<Animatable>();
            if (animScript != null && animScript.Creator != null && 
                animScript.Creator.GetInstanceID() == GetInstanceID())
            {
                animScript.Deselected();
                GestureManager.ProcessObjectDeletion(animScript);
                Destroy(animScript.gameObject);
                lastObjectDestroyTime = DateTime.Now;
                return;
            }

            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip)
            {
                GestureManager.TryInstantiatingAnimatable(hand, this);
            }
        }

        private void OnTriggerStay(Collider other)
        {
            if ((DateTime.Now - lastObjectDestroyTime).TotalSeconds < Globals.SHELF_GESTURE_WAITING_PERIOD)
                return;

            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip)
            {
                GestureManager.TryInstantiatingAnimatable(hand, this);
            }
        }
    }
}