using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace GestureAnim
{
    [RequireComponent(typeof(Rigidbody), typeof(Collider))]
    public class EmissionCone : MonoBehaviour
    {
        public ParticleEmitter emitter { get; private set; }
        private Animatable animScript;

        private int activeIntersections = 0;

        private void Awake()
        {
            emitter = GetComponentInParent<ParticleEmitter>();
            animScript = GetComponentInParent<Animatable>();

            if (emitter == null)
                throw new ArgumentNullException("Emission cone without emitter parent!");
        }
        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public void ResetIntersectionCounter()
        {
            activeIntersections = 0;
            emitter.EnableEmissionConeGestures = false;
        }

        private void OnTriggerEnter(Collider other)
        {
            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);
            if (hand == OvrAvatar.HandType.Right && joint == PoseManager.HandJoint.IndexTip)
            {
                activeIntersections++;

                if (activeIntersections == 1)
                {
                    emitter.EnableEmissionConeGestures = true;
                    //Debug.Log("Enabling interactions with emission cone!");
                }
            }
            PoseManager.OnHandObjectCollisionStart(animScript, other.gameObject);
        }

        private void OnTriggerExit(Collider other)
        {
            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);
            if (hand == OvrAvatar.HandType.Right && joint == PoseManager.HandJoint.IndexTip)
            {
                activeIntersections = Mathf.Max(activeIntersections - 1, 0);
                if (activeIntersections == 0)
                {
                    emitter.EnableEmissionConeGestures = false;
                    //Debug.Log("Disabling interactions with emission cone!");
                }
            }
            PoseManager.OnHandObjectCollisionEnd(animScript, other.gameObject);
        }

        private void OnTriggerStay(Collider other)
        {
            PoseManager.OnHandObjectCollisionStay(animScript, other.gameObject);
        }
    }
}
