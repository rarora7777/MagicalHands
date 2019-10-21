//////////////////////////////////////////////////////////////////////////////////////
/// InputManager is a pseudo-static class. That is, while it is not a syntactically
/// static class, it should be treated as a semantically static class.
/// InputManager interfaces with the Oculus VR API and provides low-level input
/// data to PoseManager.
//////////////////////////////////////////////////////////////////////////////////////

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using GestureAnim;

namespace GestureAnim
{
    public class InputManager : MonoBehaviour
    {
        static int timePassed = int.MaxValue;
        static bool leftTouchActive = false;
        static bool rightTouchActive = false;

        public static PoseManager poseManager;
        private static int currentSkyboxIdx = 0;
        public Material[] Skybox;
        public GameObject GroundPlane;
        public Color[] GroundPlaneColour;

        public Camera Cam2;

        // Start is called before the first frame update
        void Start()
        {
            RenderSettings.skybox = Skybox[0];
        }

        // Update is called once per frame
        void Update()
        {
            OVRInput.Update();

            leftTouchActive = OVRInput.IsControllerConnected(OVRInput.Controller.LTouch);
            rightTouchActive = OVRInput.IsControllerConnected(OVRInput.Controller.RTouch);

            PoseManager.HandJoint joint = PoseManager.HandJoint.Max;

            if (Input.GetKeyUp(KeyCode.Alpha0))
                joint = PoseManager.HandJoint.ThumbTip;
            else if(Input.GetKeyUp(KeyCode.Alpha1))
                joint = PoseManager.HandJoint.IndexTip;
            else if (Input.GetKeyUp(KeyCode.Alpha2))
                joint = PoseManager.HandJoint.MiddleTip;
            else if (Input.GetKeyUp(KeyCode.Alpha3))
                joint = PoseManager.HandJoint.RingTip;
            else if (Input.GetKeyUp(KeyCode.Alpha4))
                joint = PoseManager.HandJoint.PinkyTip;
            else if (Input.GetKeyUp(KeyCode.Alpha5))
            {
                var thumb = poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Thumb);
                var finger = poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Fingers);
                var palm = poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Palm);

                var wrist = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.Wrist);

                var indexBase = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexBase);
                var indexTip = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip);

                var indexFingerVec = wrist.InverseTransformVector(indexTip.position - indexBase.position).normalized;

                Debug.Log(indexFingerVec.ToString("F4"));

            }
            else if(Input.GetKeyUp(KeyCode.G))
            {
                Debug.Log(GestureManager.RecognitionState);
            }

            if (joint < PoseManager.HandJoint.Max)
                LogJointState(joint, OvrAvatar.HandType.Right);


            // Ctrl-S to save current scene
            if (Input.GetKeyUp(KeyCode.S)
#if UNITY_EDITOR
                )
#else
                &&
                (Input.GetKey(KeyCode.RightControl) || Input.GetKey(KeyCode.LeftControl)))
#endif
                FileIOManager.TrySaveCurrentScene();
            // Ctrl-Z to undo
            else if (Input.GetKeyUp(KeyCode.Z)
#if UNITY_EDITOR
                )
#else
                &&
                (Input.GetKey(KeyCode.RightControl) || Input.GetKey(KeyCode.LeftControl)))
#endif
            {
                TimeManager.UndoLastOperation();
            }
            //Ctrl-E to change Skybox
            else if (Input.GetKeyUp(KeyCode.E)
#if UNITY_EDITOR
                )
#else
                &&
                (Input.GetKey(KeyCode.RightControl) || Input.GetKey(KeyCode.LeftControl)))
#endif
            {
                currentSkyboxIdx = (currentSkyboxIdx + 1) % Skybox.Length;
                RenderSettings.skybox = Skybox[currentSkyboxIdx];
                GroundPlane.GetComponent<MeshRenderer>().material.color = GroundPlaneColour[currentSkyboxIdx];
            }
            //Ctrl-H to hide Ground Plane
            else if (Input.GetKeyUp(KeyCode.H)
#if UNITY_EDITOR
                )
#else
                &&
                (Input.GetKey(KeyCode.RightControl) || Input.GetKey(KeyCode.LeftControl)))
#endif
            {
                GroundPlane.GetComponent<MeshRenderer>().enabled = !GroundPlane.GetComponent<MeshRenderer>().enabled;
            }

            if (Input.GetKeyUp(KeyCode.A))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Rotate(Vector3.down, Space.World);
                else
                    Cam2.transform.Rotate(5 * Vector3.down, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.D))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Rotate(Vector3.up, Space.World);
                else
                    Cam2.transform.Rotate(5*Vector3.up, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.W))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Rotate(Vector3.left, Space.World);
                else
                    Cam2.transform.Rotate(5 * Vector3.left, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.S) && !(Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Rotate(Vector3.right, Space.World);
                else
                    Cam2.transform.Rotate(5 * Vector3.right, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.LeftArrow))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f*Vector3.left, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.left, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.RightArrow))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f*Vector3.right, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.right, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.UpArrow))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f * Vector3.forward, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.forward, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.DownArrow))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f * Vector3.back, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.back, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.Q))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f * Vector3.up, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.up, Space.World);
            }
            else if (Input.GetKeyUp(KeyCode.E) && !(Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)))
            {
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                    Cam2.transform.Translate(0.01f * Vector3.down, Space.World);
                else
                    Cam2.transform.Translate(0.05f * Vector3.down, Space.World);
            }
        }


        static void LogJointState(PoseManager.HandJoint joint, OvrAvatar.HandType hand = OvrAvatar.HandType.Right)
        {
            var handActive = 
                (hand == OvrAvatar.HandType.Right) ?
                    (poseManager.HandRight != null) :
                    (poseManager.HandLeft != null);

            if (handActive)
            {
                Debug.Log(hand.ToString() + " " + joint.ToString() + " " + PoseManager.GetFingertipLocalPosition(hand, joint) * 100);
            }
        }

        static void FixedUpdate()
        {
            timePassed = (int)Mathf.Max(0, timePassed + 1);
        }

        // Check if controller is active
        public static bool IsControllerActive(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return leftTouchActive;
            else
                return rightTouchActive;
        }

        // Functions to find if buttons are pressed
        public static bool IsLowerButtonPressed(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
        }

        public static bool IsUpperButtonPressed(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch);
        }

        public static bool IsThumbStickPressed(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbstick, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbstick, OVRInput.Controller.RTouch);
        }

        // Checks if any controls on the upper surface are pressed
        public static bool IsTopSurfacePressed(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Button.Any & ~OVRInput.Button.PrimaryIndexTrigger & ~OVRInput.Button.PrimaryHandTrigger, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Button.Any & ~OVRInput.Button.PrimaryIndexTrigger & ~OVRInput.Button.PrimaryHandTrigger, OVRInput.Controller.RTouch);
        }

        // Functions to find if buttons, triggers, etc. are touched
        public static bool IsLowerButtonTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.One, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.One, OVRInput.Controller.RTouch);
        }

        public static bool IsUpperButtonTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.Two, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.Two, OVRInput.Controller.RTouch);
        }

        public static bool IsThumbStickTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbstick, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbstick, OVRInput.Controller.RTouch);
        }

        public static bool IsIndexTriggerTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        }

        public static bool IsThumbRestTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbRest, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.PrimaryThumbRest, OVRInput.Controller.RTouch);
        }

        // Checks if any controls on the top (horizontal) surface are touched
        public static bool IsTopSurfaceTouched(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Touch.Any & ~OVRInput.Touch.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Touch.Any & ~OVRInput.Touch.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        }

        // Get trigger values
        public static float GetIndexTriggerValue(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        }

        public static float GetMiddleTriggerValue(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);
        }

        // Get thumbstick position
        public static Vector2 GetThumbStickPosition(OvrAvatar.HandType hand)
        {
            if (hand == OvrAvatar.HandType.Left)
                return OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
            else
                return OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        }
    }

}