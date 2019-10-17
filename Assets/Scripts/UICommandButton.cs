using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    [RequireComponent(typeof(Collider), typeof(Rigidbody))]
    public class UICommandButton : MonoBehaviour
    {
        public Vector3 PushDirection;
        public float PushMagnitude;
        public bool DisallowMultiple = false;

        private bool isOn = false;
        private Vector3 defaultPosition;
        private bool pressAnimationOngoing = false;
        private DateTime lastActionTime;
        private DateTime lastPressBeginTime;

        private Transform visibleObj;

        private Action callback;

        void Start()
        {
            defaultPosition = transform.localPosition;
            PushDirection.Normalize();

            var timeManager = FindObjectOfType<TimeManager>();

            visibleObj = GetComponentInChildren<MeshRenderer>().transform;

            // We only have a few of these right now, so string search is _okay_
            if (name.Contains("PrevFrame"))
                callback = timeManager.GoToPreviousFrame;
            else if (name.Contains("NextFrame"))
                callback = timeManager.GoToNextFrame;
            else if (name.Contains("FirstFrame"))
                callback = timeManager.GoToFirstFrame;
            else if (name.Contains("LastFrame"))
                callback = timeManager.GoToLastFrame;
            else if (name.Contains("Undo"))
                callback = TimeManager.UndoLastOperation;
        }

        // Update is called once per frame
        void Update()
        {
            if (pressAnimationOngoing)
            {
                float animParam = (float)(DateTime.Now - lastPressBeginTime).TotalSeconds / (Globals.UI_BUTTON_ANIMATION_TIME/2);

                if (animParam >= 1)
                {
                    visibleObj.localPosition = defaultPosition + (isOn ? PushMagnitude * PushDirection : Vector3.zero);
                    pressAnimationOngoing = false;

                    // If the button is only supposed to be executed once when pressed, 
                    // then it should "bounce back" into position
                    if (DisallowMultiple && isOn)
                    {
                        isOn = false;
                        pressAnimationOngoing = true;
                        lastPressBeginTime = DateTime.Now;
                    }
                    return;
                }

                if (isOn)
                {
                    visibleObj.localPosition = defaultPosition + animParam * PushMagnitude * PushDirection;
                }
                else
                {
                    visibleObj.localPosition = defaultPosition + (1.0f - animParam) * PushMagnitude * PushDirection;
                }
            }
        }

        public void SwitchState()
        {
            pressAnimationOngoing = true;
        }

        private void OnTriggerEnter(Collider other)
        {
            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip &&
                (DateTime.Now - lastPressBeginTime).TotalSeconds >= Globals.UI_BUTTON_WAIT_TIME)
            {
                isOn = true;
                SwitchState();
                callback?.Invoke();
                lastPressBeginTime = lastActionTime = DateTime.Now;
            }
        }

        private void OnTriggerStay(Collider other)
        {
            if (DisallowMultiple)
                return;

            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip &&
                (DateTime.Now - lastActionTime).TotalSeconds >= 0.1f)
            {
                callback?.Invoke();
                lastActionTime = DateTime.Now;
            }
        }

        private void OnTriggerExit(Collider other)
        {
            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip)
            {
                isOn = false;
                SwitchState();
            }
        }
    }
}