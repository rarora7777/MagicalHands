//////////////////////////////////////////////////////////////////////////////////////
/// UIButton is a highly-specialized MonoBehaviour used to provide timeline-control
/// functionality to the buttons in the Spatial UI. By default, this UI is places on a
/// UI panel/box hovering over the user's left hand. Please look at the scene in the 
/// Unity Editor.
//////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    [RequireComponent(typeof(Collider), typeof(Rigidbody))]
    public class UIButton : MonoBehaviour
    {
        public TMPro.TextMeshPro Title;
        public TMPro.TextMeshPro Description;

        public string OnTitle;
        public string OffTitle;

        [TextArea(2, 3)]
        public string OnDescription;
        [TextArea(2, 3)]
        public string OffDescription;

        public Material OnMaterial;
        public Material OffMaterial;

        public bool InitiallyEnabled;

        public Vector3 PushDirection;
        public float PushMagnitude;

        private bool isOn;
        private Vector3 defaultPosition;
        private bool pressAnimationOngoing = false;
        private DateTime stateSwitchTime;

        private Action callback;

        void Start()
        {
            isOn = InitiallyEnabled;
            defaultPosition = transform.localPosition;
            PushDirection.Normalize();
            SetUI();

            // We only have a few of these right now, so string search is _okay_
            if (name.Contains("Play"))
            {
                callback = TimeManager.SwitchAnimationPlayState;
                TimeManager.OnAnimationPauseInScript += SetOffByScript;
                TimeManager.OnAnimationPlayInScript += SetOnByScript;
            }
            else if (name.Contains("Record"))
                callback = TimeManager.SwitchRecordingState;
        }

        // Update is called once per frame
        void Update()
        {
            if (pressAnimationOngoing)
            {
                float animParam = (float)(DateTime.Now - stateSwitchTime).TotalSeconds / Globals.UI_BUTTON_ANIMATION_TIME;

                if (animParam >= 1)
                {
                    pressAnimationOngoing = false;
                    transform.localPosition  = defaultPosition + (isOn ? PushMagnitude * PushDirection : Vector3.zero);
                    return;
                }

                if (isOn)
                {
                    transform.localPosition = defaultPosition + animParam * PushMagnitude * PushDirection;
                }
                else
                {
                    transform.localPosition = defaultPosition + (1.0f - animParam) * PushMagnitude * PushDirection;
                }
            }
        }

        private void SetUI()
        {
            stateSwitchTime = DateTime.Now;
            if (isOn)
            {
                Title.SetText(OnTitle);
                Description.SetText(OnDescription);
                GetComponent<MeshRenderer>().material = OnMaterial;
            }
            else
            {
                Title.SetText(OffTitle);
                Description.SetText(OffDescription);
                GetComponent<MeshRenderer>().material = OffMaterial;
            }
        }

        public void SwitchState()
        {
            isOn = !isOn;
            pressAnimationOngoing = true;
            SetUI();
        }

        private void OnTriggerEnter(Collider other)
        {
            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);

            if (joint == PoseManager.HandJoint.IndexTip && 
                (DateTime.Now - stateSwitchTime).TotalSeconds >= Globals.UI_BUTTON_WAIT_TIME)
            {
                SwitchState();
                callback?.Invoke();
            }
        }

        private void SetOffByScript()
        {
            isOn = false;
            SetUI();
            transform.localPosition = defaultPosition;
        }

        private void SetOnByScript()
        {
            isOn = true;
            SetUI();
            transform.localPosition = defaultPosition + PushMagnitude * PushDirection;
        }
    }
}