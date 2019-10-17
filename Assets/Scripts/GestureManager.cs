using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Oculus.Avatar;
using System;

namespace GestureAnim
{
    public class GestureManager : MonoBehaviour
    {
        // Start is called before the first frame update
        public static PoseManager poseManager;
        public static TimeManager TimeManager;
        public GameObject rotationAxisUI;
        static List<Animatable> rightInteracting = new List<Animatable>();
        static List<Animatable> leftInteracting = new List<Animatable>();

        public static Animatable ObjectBeingInstantiated { get; private set; }
        private static ShelfObject ObjectInstantiator = null;
        public static OvrAvatar.HandType HandInstantiatingObject { get; private set; }

        static bool rightPinchOn = false, leftPinchOn = false;
        static bool rightGrabOn = false, leftGrabOn = false;
        static bool rightPointOn = false, leftPointOn = false;
        static bool rightAlmostPointOn = false, leftAlmostPointOn = false;

        static DateTime rightPinchOnTime = DateTime.MaxValue, leftPinchOnTime = DateTime.MaxValue;
        static DateTime rightGrabOnTime = DateTime.MaxValue, leftGrabOnTime = DateTime.MaxValue;
        static DateTime rightPointOnTime = DateTime.MaxValue, leftPointOnTime = DateTime.MaxValue;
        static DateTime rightAlmostPointOnTime = DateTime.MaxValue, leftAlmostPointOnTime = DateTime.MaxValue;
        static DateTime rightPinchOffTime = DateTime.MaxValue, leftPinchOffTime = DateTime.MaxValue;
        static DateTime rightGrabOffTime = DateTime.MaxValue, leftGrabOffTime = DateTime.MaxValue;
        static DateTime rightPointOffTime = DateTime.MaxValue, leftPointOffTime = DateTime.MaxValue;
        static DateTime rightAlmostPointOffTime = DateTime.MaxValue, leftAlmostPointOffTime = DateTime.MaxValue;
        static DateTime tapGestureStartTime = DateTime.MinValue;

        static ParticleEmitter emitterReceivingConeGestures = null;

        //List<Vector3> rightHandPositions;
        //List<Vector3> leftHandPositions;
        //List<float> times;

        public static GestureRecognizerState RecognitionState { get; private set; } = GestureRecognizerState.Default;

        void Start()
        {
            PoseManager.OnLeftHandStartPinch += LeftHandPinchStartHandler;
            PoseManager.OnRightHandStartPinch += RightHandPinchStartHandler;

            PoseManager.OnLeftHandStopPinch += LeftHandPinchStopHandler;
            PoseManager.OnRightHandStopPinch += RightHandPinchStopHandler;

            PoseManager.OnLeftHandStartPoint += LeftHandPointStartHandler;
            PoseManager.OnRightHandStartPoint += RightHandPointStartHandler;

            PoseManager.OnLeftHandStopPoint += LeftHandPointStopHandler;
            PoseManager.OnRightHandStopPoint += RightHandPointStopHandler;

            PoseManager.OnLeftHandStartAlmostPoint += LeftHandAlmostPointStartHandler;
            PoseManager.OnRightHandStartAlmostPoint += RightHandAlmostPointStartHandler;

            PoseManager.OnLeftHandStopAlmostPoint += LeftHandAlmostPointStopHandler;
            PoseManager.OnRightHandStopAlmostPoint += RightHandAlmostPointStopHandler;

            PoseManager.OnLeftHandStartGrab += LeftHandGrabStartHandler;
            PoseManager.OnRightHandStartGrab += RightHandGrabStartHandler;

            PoseManager.OnLeftHandStartGrab += LeftHandGrabStopHandler;
            PoseManager.OnRightHandStopGrab += RightHandGrabStopHandler;

            PoseManager.OnIntersectionStart += IntersectionStartHandler;
            PoseManager.OnIntersectionEnd += IntersectionEndHandler;

            poseManager = FindObjectOfType<PoseManager>();

            rotationAxisUI.GetComponent<Renderer>().enabled = false;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyUp(KeyCode.P))
            {
                TimeManager.SwitchAnimationPlayState();
            }

            if ( Input.GetKeyUp(KeyCode.R))
            {
                TimeManager.SwitchRecordingState();
            }

            switch (RecognitionState)
            {
                case GestureRecognizerState.Default:
                    if (rightInteracting.Count > 0)
                    {
                        SetRecognitionState(GestureRecognizerState.Recognizing);
                    }
                    else if (leftInteracting.Count > 0)
                    {
                        SetRecognitionState(GestureRecognizerState.Recognizing);
                    }
                    else if (TimeManager.SelectedObject != null)
                    {
                        SetRecognitionState(GestureRecognizerState.Recognizing);
                    }
                    break;
                case GestureRecognizerState.Recognizing:
                    if (rightInteracting.Count == 0 && leftInteracting.Count == 0 && TimeManager.SelectedObject == null)
                    {
                        SetRecognitionState(GestureRecognizerState.Default);
                    }
                    // Wait for a bit before we infer which gesture to execute
                    // when the right hand is in pinched position
                    else if (rightInteracting.Count > 0 && 
                        rightPinchOn && 
                        (DateTime.Now - rightPinchOnTime).TotalSeconds > Globals.GESTURE_WAIT_TIME)
                    {
                        
                        // if left hand is also pinched onto an object, then execute scaling gesture
                        if (leftInteracting.Count > 0 && leftPinchOn)
                        {
                            SetRecognitionState(GestureRecognizerState.Scaling);
                            ProcessScalingGesture();
                        }
                        // otherwise, translate
                        else
                        {
                            SetRecognitionState(GestureRecognizerState.Translating);
                            ProcessTranslationGesture();
                        }
                    }
                    //show rotating axis
                    else if (rightInteracting.Count > 0 &&
                        rightPointOn && 
                        (DateTime.Now - rightPointOnTime).TotalSeconds > Globals.GESTURE_WAIT_TIME)
                    {
                        SetRecognitionState(GestureRecognizerState.ReadyToRotate);
                        TimeManager.KeepOldRotationAxis = false;
                        ProcessRotationAxisGesture();
                    }
                    // Can try to recognize an emission cone gesture if 
                    // 1. an emitter is selected, 
                    // 2. both hands have the same pose, and
                    // 3. some constraints on their positions w.r.t emitter and each other??
                    else if (TimeManager.SelectedObject != null &&
                        TimeManager.SelectedObject.GetComponent<ParticleEmitter>() != null &&
                        PoseManager.GetHandShape(OvrAvatar.HandType.Right) == PoseManager.GetHandShape(OvrAvatar.HandType.Left) &&
                        PoseManager.GetHandShape(OvrAvatar.HandType.Right).HasFlag(PoseManager.HandShape.Fist))
                    {
                        var selected = TimeManager.SelectedObject;
                        var leftPos = poseManager.GetHandTransform(OvrAvatar.HandType.Left, Globals.EMISSION_GESTURE_JOINT).position;
                        var rightPos = poseManager.GetHandTransform(OvrAvatar.HandType.Right, Globals.EMISSION_GESTURE_JOINT).position;
                        leftPos = selected.transform.InverseTransformPoint(leftPos);
                        rightPos = selected.transform.InverseTransformPoint(rightPos);
                        var leftPosProj = new Vector2(leftPos.x, leftPos.y);
                        var rightPosProj = new Vector2(rightPos.x, rightPos.y);

                        if (Vector2.Dot(-leftPosProj.normalized, rightPosProj.normalized) > 0.6f &&
                            leftPos.sqrMagnitude / rightPos.sqrMagnitude <= 2.0f &&
                            rightPos.sqrMagnitude / leftPos.sqrMagnitude <= 2.0f &&
                            Mathf.Max(Mathf.Abs(leftPos.z), Mathf.Abs(rightPos.z)) <= 0.2f / selected.transform.lossyScale.z)
                        {
                            SetRecognitionState(GestureRecognizerState.DrawingEmissionCone);
                            ProcessEmissionConeGestureStart();
                        }
                    }
                    else if (rightInteracting.Count == 0 && rightAlmostPointOn)
                    {
                        tapGestureStartTime = DateTime.Now;
                        SetRecognitionState(GestureRecognizerState.ReadyToSelect);
                    }
                    break;
                case GestureRecognizerState.Scaling:
                    if (rightInteracting.Count == 0)
                    {
                        TimeManager.StopTransforming(RecognitionState);
                        SetRecognitionState(GestureRecognizerState.Default);
                    }
                    else if (leftInteracting.Count == 0)
                    {
                        TimeManager.StopTransforming(RecognitionState);
                        SetRecognitionState(GestureRecognizerState.Recognizing);
                    }
                    break;
                case GestureRecognizerState.Translating:
                    if (rightInteracting.Count == 0)
                    {
                        TimeManager.StopTransforming(RecognitionState);
                        SetRecognitionState(GestureRecognizerState.Default);
                    }
                    break;
                case GestureRecognizerState.Rotating:
                    // Interaction ended: Go back to default state
                    if (rightInteracting.Count == 0)
                    {
                        TimeManager.StopTransforming(RecognitionState);
                        SetRecognitionState(GestureRecognizerState.Default);
                        ProcessRotationEnd();
                    }
                    // Current rotation ended, but user might be taking a break
                    // Keep showing the axis of rotation
                    else if (!(leftPinchOn || leftPointOn))
                    {
                        TimeManager.StopTransforming(RecognitionState);
                        SetRecognitionState(GestureRecognizerState.ReadyToRotate);
                        TimeManager.KeepOldRotationAxis = true;
                    }
                    break;
                case GestureRecognizerState.ReadyToRotate:
                    // Interaction ended: Go back to default state
                    if (rightInteracting.Count == 0)
                    {
                        SetRecognitionState(GestureRecognizerState.Default);
                        ProcessRotationCancellation();
                    }
                    // Starte computing the rotation deltas
                    else if (leftPointOn || leftPinchOn)
                    {
                        SetRecognitionState(GestureRecognizerState.Rotating);
                        ProcessRotationGesture();
                    }
                    // Try to recognize "Tap" gesture
                    else if (!rightPointOn && rightAlmostPointOn)
                    {
                        tapGestureStartTime = DateTime.Now;
                        SetRecognitionState(GestureRecognizerState.ReadyToSelect);
                        ProcessRotationCancellation();
                    }
                    // Otherwise, just recompute the selected object and the rotation axis
                    else
                    {
                        ProcessRotationAxisGesture();
                    }
                    break;
                case GestureRecognizerState.ReadyToSelect:
                    // Pointing right finger again
                    if (rightPointOn/* && rightInteracting.Count > 0*/)
                    {
                        // If the amount of time spent "almost-pointing" was small, execute selection command
                        if ((DateTime.Now - tapGestureStartTime).TotalSeconds < Globals.TAP_GESTURE_MAX_TIME)
                        {
                            Debug.Log("Selecting...");
                            ProcessTapGesture();
                            tapGestureStartTime = DateTime.MinValue;
                        }
                        // Else, ignore and try to go back to "Ready-to-rotate" state
                        else
                        {
                            if (rightInteracting.Count > 0)
                            {
                                SetRecognitionState(GestureRecognizerState.ReadyToRotate);
                                TimeManager.KeepOldRotationAxis = false;
                                ProcessRotationAxisGesture();
                            }
                            else
                                SetRecognitionState(GestureRecognizerState.Default);
                        }
                    }
                    //// If no objects are in the interaction range, ignore the whole interaction
                    //else if ((DateTime.Now - rightPointOffTime).TotalSeconds > Globals.TAP_GESTURE_MAX_TIME)
                    //{
                    //    SetRecognitionState(GestureRecognizerState.Default);
                    //}
                    //else
                    //{
                    //    Debug.Log("Point: " + rightPointOn + "Interacting: " + rightInteracting.Count +
                    //        "Time since off: " + (DateTime.Now - rightPointOffTime).TotalSeconds);
                    //}
                    break;
                case GestureRecognizerState.AfterSelectGesture:
                    // Ignore all other hand pose changes until an object is selected
                    if (rightInteracting.Count == 0)
                        SetRecognitionState(GestureRecognizerState.Default);
                    break;
                case GestureRecognizerState.ReadyForConeGesture:
                    // hand exited cone
                    if (emitterReceivingConeGestures.EnableEmissionConeGestures == false)
                    {
                        SetRecognitionState(GestureRecognizerState.Default);
                    }
                    // If right hand is in an active pose, start recording
                    else if (rightPointOn || rightPinchOn || rightGrabOn)
                    {
                        SetRecognitionState(GestureRecognizerState.RecognizingConeGesture);
                        emitterReceivingConeGestures.ConeGestureStarted();
                    }
                    break;
                case GestureRecognizerState.RecognizingConeGesture:
                    if (!(rightPointOn || rightPinchOn || rightGrabOn))
                    {
                        SetRecognitionState(GestureRecognizerState.ReadyForConeGesture);
                        emitterReceivingConeGestures.ConeGestureFinished();
                    }
                    break;
                case GestureRecognizerState.Instantiating:
                    if (
                        ( HandInstantiatingObject == OvrAvatar.HandType.Right &&
                        (rightAlmostPointOn || rightPointOn || rightGrabOn || rightPinchOn) ) ||
                        ( HandInstantiatingObject == OvrAvatar.HandType.Left && 
                        (leftAlmostPointOn || leftPointOn || leftGrabOn || leftPinchOn) )
                    )
                    {
                        var fingerPos = poseManager.GetHandTransform(HandInstantiatingObject, PoseManager.HandJoint.IndexTip).position;
                        ObjectBeingInstantiated.transform.position = fingerPos;
                    }
                    else
                    {
                        // If the object is within the bounds of an emitter
                        // then set it to be the particle mesh
                        var colliders = Physics.OverlapSphere(poseManager.GetHandTransform(HandInstantiatingObject, PoseManager.HandJoint.IndexTip).position, 0.01f);
                        ParticleEmitter targetEmitter = null;
                        foreach(var collider in colliders)
                            if (collider.gameObject.GetInstanceID() != ObjectBeingInstantiated.gameObject.GetInstanceID() &&
                                collider.GetComponent<ParticleEmitter>() != null)
                            {
                                targetEmitter = collider.GetComponent<ParticleEmitter>();
                                break;
                            }

                        if (targetEmitter != null && ObjectInstantiator.ParticlePrefab != null)
                        {
                            targetEmitter.TryChangeParticleShapeFromDroppedObject(ObjectInstantiator);
                            Destroy(ObjectBeingInstantiated.gameObject);
                            SetRecognitionState(GestureRecognizerState.Default);
                        }
                        // Otherwise, instatiate as an Animatable
                        else
                        {
                            ObjectBeingInstantiated.Init(ObjectInstantiator);
                            SetRecognitionState(GestureRecognizerState.Default);
                        }
                    }
                    break;
            }
        }

        static void SetRecognitionState(GestureRecognizerState state)
        {
            Debug.Log(RecognitionState + "-->" + state);
            RecognitionState = state;
        }

        private void LateUpdate()
        {
            
        }

        private static void TrySwitchToEmissionConeGestureMode(ref Animatable selected)
        {
            if (selected != null && selected.GetComponent<ParticleEmitter>() != null)
            {
                var emitterCenter = selected.SizeProxy.bounds.center;
                var emissionConeCenter = selected.GetComponent<ParticleEmitter>().RenderedConeMeshFilter.GetComponent<Renderer>().bounds.center;
                var indexFingerPos = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position;

                if ((indexFingerPos - emitterCenter).sqrMagnitude > (indexFingerPos - emissionConeCenter).sqrMagnitude)
                {
                    emitterReceivingConeGestures = selected.GetComponent<ParticleEmitter>();
                    // stop the gesture which would otherwise have started
                    selected = null;
                    SetRecognitionState(GestureRecognizerState.ReadyForConeGesture);
                }
            }
        }

        private static Animatable GetSmallest(List<Animatable> list, bool overrideGesturalSelection = false)
        {
            //if (overrideGesturalSelection && TimeManager.SelectedObject != null)
            //    return TimeManager.SelectedObject;
            //try
            //{
                float minSize = 100.0f;
                Animatable selected = null;
                foreach (var obj in list)
                {
                    var objSize = obj.SizeProxy.bounds.size.sqrMagnitude;
                    if (objSize < minSize)
                    {
                        selected = obj;
                        minSize = objSize;
                    }
                }

                TrySwitchToEmissionConeGestureMode(ref selected);

                return selected;
            //}
            //catch (Exception e)
            //{
            //    Debug.Log("hmmm");
            //    return null;
            //}
        }

        private static Animatable GetSmallest(List<Animatable> list1, List<Animatable> list2)
        {
            float minSize = 100.0f;
            Animatable selected = null;
            foreach (var obj in list1)
            {
                // inefficient, but this list should usually be small
                if (!list2.Contains(obj))
                    continue;
                var objSize = obj.SizeProxy.bounds.size.sqrMagnitude;
                if (objSize < minSize)
                {
                    selected = obj;
                    minSize = objSize;
                }
            }
            return selected;
        }

        static void ProcessTapGesture()
        {
            var selected = GetSmallest(rightInteracting, true);
            if (selected != null)
            {
                Debug.Log("Selecting!");
                TimeManager.SelectObject(selected);
            }
            else if (TimeManager.SelectedObject != null)
            {
                Debug.Log("Deselecting!");
                TimeManager.SelectedObject.Deselected();
                //SetRecognitionState(GestureRecognizerState.Default);
            }
            else
            {
                Debug.Log("Nothing to do here!");
            }
        }

        static void ProcessScalingGesture()
        {
            var selected = GetSmallest(rightInteracting, leftInteracting);

            if (selected != null)
            {
                TimeManager.StartScaling(selected);
            }
        }

        static void ProcessTranslationGesture()
        {
            var selected = GetSmallest(rightInteracting);

            if (selected != null)
            {
                Debug.Log("Translating " + selected.name);
                TimeManager.StartTranslation(selected);
            }
        }

        static void ProcessRotationGesture()
        {
            var index = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip);
            var pos = index.position;
            var dir = (
                Globals.POINTING_FINGER_COORDINATES.x * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Fingers) +
                Globals.POINTING_FINGER_COORDINATES.y * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Thumb) +
                Globals.POINTING_FINGER_COORDINATES.z * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Palm)
                ).normalized;

            var selected = GetSmallest(rightInteracting);
            if (selected != null)
            {
                Debug.Log("Rotation " + selected.name);
                TimeManager.StartRotation(selected);
            }
        }

        void ProcessRotationEnd()
        {
            Debug.Log("Rotation gesture ended!");
            rotationAxisUI.GetComponent<Renderer>().enabled = false;
        }

        void ProcessRotationAxisGesture()
        {
            var index = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip);
            var pos = index.position;
            var dir = (
                Globals.POINTING_FINGER_COORDINATES.x * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Fingers) +
                Globals.POINTING_FINGER_COORDINATES.y * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Thumb) +
                Globals.POINTING_FINGER_COORDINATES.z * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Palm) 
                ).normalized;

            var selected = GetSmallest(rightInteracting);
            if (selected != null)
            {
                //Debug.Log("Ready to rotate " + selected.name);
                var size = Mathf.Max(
                    1.25f*selected.SizeProxy.bounds.size.magnitude,
                    0.3f);

                rotationAxisUI.transform.position = selected.transform.position;
                rotationAxisUI.transform.localScale = new Vector3(0.01f, size, 0.01f);
                rotationAxisUI.transform.up = dir;
                //rotationAxisUI.transform.SetParent(index, true);
                rotationAxisUI.GetComponent<Renderer>().enabled = true;
            }
        }

        void ProcessRotationCancellation()
        {
            //Debug.Log("Rotation gesture cancelled!");
            rotationAxisUI.GetComponent<Renderer>().enabled = false;
        }

        static void ProcessEmissionConeGestureStart()
        {
            Debug.Log("Defining emission cone.");
            //emissionConeStartTime = DateTime.Now;
            //rightHandPositions = new List<Vector3>(250);
            //leftHandPositions = new List<Vector3>(250);
            //times = new List<float>(250);

            PoseManager.OnHandShapeChange += ProcessEmissionConeGestureEnd;
            TimeManager.SelectedObject.GetComponent<ParticleEmitter>().EmissionConeDrawingStarted();
        }

        static void ProcessEmissionConeGestureEnd()
        {
            //Debug.Log("Emission cone defined! #Pts: " + leftHandPositions.Count);
            PoseManager.OnHandShapeChange -= ProcessEmissionConeGestureEnd;
            SetRecognitionState(GestureRecognizerState.Default);

            //if (leftHandPositions.Count > 1)
            //{
            //    TimeManager.SelectedObject.GetComponent<ParticleEmitter>().NewEmissionCone(leftHandPositions, rightHandPositions, times);
            //}
            TimeManager.SelectedObject.GetComponent<ParticleEmitter>().EmissionConeDrawingFinished();
        }

        private static void LeftHandPinchStartHandler()
        {
            leftPinchOn = true;
            Debug.Log("Left Pinch ON");
            leftPinchOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Left, leftInteracting);
        }

        private static void RightHandPinchStartHandler()
        {
            rightPinchOn = true;
            Debug.Log("Right Pinch ON");
            rightPinchOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Right, rightInteracting);
        }

        private static void LeftHandPinchStopHandler()
        {
            leftPinchOn = false;
            Debug.Log("Left Pinch OFF");
            leftPinchOffTime = DateTime.Now;
            leftInteracting.Clear();
        }

        private static void RightHandPinchStopHandler()
        {
            rightPinchOn = false;
            Debug.Log("Right Pinch OFF");
            rightPinchOffTime = DateTime.Now;
            rightInteracting.Clear();
            //Debug.Log("Pinch OFF. All interactions cleared.");
        }

        private static void LeftHandPointStartHandler()
        {
            leftPointOn = true;
            leftPointOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Left, leftInteracting);
        }

        private static void RightHandPointStartHandler()
        {
            rightPointOn = true;
            rightPointOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Right, rightInteracting);

            //Debug.Log("Right Pointing. Time since Almost-point: " +
            //    (rightPointOnTime - rightAlmostPointOnTime).TotalSeconds +
            //    " Recognition state: " + RecognitionState +
            //    " Selection: " + (TimeManager.SelectedObject ==  null).ToString());
        }

        private static void LeftHandPointStopHandler()
        {
            leftPointOn = false;
            leftPointOffTime = DateTime.Now;
            leftInteracting.Clear();
        }

        private static void RightHandPointStopHandler()
        {
            rightPointOn = false;
            rightPointOffTime = DateTime.Now;
            rightInteracting.Clear();
            //Debug.Log("Point OFF. All interactions cleared!");
        }

        private static void LeftHandAlmostPointStartHandler()
        {
            leftAlmostPointOn = true;
            leftAlmostPointOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Left, leftInteracting);
        }

        private static void RightHandAlmostPointStartHandler()
        {
            rightAlmostPointOn = true;
            rightAlmostPointOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Right, rightInteracting);
        }

        private static void LeftHandAlmostPointStopHandler()
        {
            leftAlmostPointOn = false;
            leftAlmostPointOffTime = DateTime.Now;
            leftInteracting.Clear();
        }

        private static void RightHandAlmostPointStopHandler()
        {
            rightAlmostPointOn = false;
            rightAlmostPointOffTime = DateTime.Now;
            rightInteracting.Clear();
            //Debug.Log("Almost Point OFF. All interactions cleared!");
        }

        private static void LeftHandGrabStartHandler()
        {
            leftGrabOn = true;
            leftGrabOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Left, leftInteracting);
        }

        private static void RightHandGrabStartHandler()
        {
            rightGrabOn = true;
            rightGrabOnTime = DateTime.Now;
            ProcessInteractionPose(OvrAvatar.HandType.Right, rightInteracting);
        }

        private static void LeftHandGrabStopHandler()
        {
            leftGrabOn = false;
            leftGrabOffTime = DateTime.Now;
            leftInteracting.Clear();
        }

        private static void RightHandGrabStopHandler()
        {
            rightGrabOn = false;
            rightGrabOffTime = DateTime.Now;
            rightInteracting.Clear();
            //Debug.Log("Grab OFF. All interactions cleared!");
        }

        private static void ProcessInteractionPose(OvrAvatar.HandType hand, List<Animatable> interactingList)
        {
            foreach (var intersection in PoseManager.Intersections)
            {
                if (intersection.Item1 == hand  && intersection.Item2 == PoseManager.HandJoint.IndexTip)
                {
                    interactingList.Add(intersection.Item3);
                    //Debug.Log("[Left] pinching " + intersection.Item3.name);
                }
            }
        }

        private static void IntersectionStartHandler(Tuple<OvrAvatar.HandType, PoseManager.HandJoint, Animatable> intersection)
        {
            if (intersection.Item1 == OvrAvatar.HandType.Right &&
                intersection.Item2 == PoseManager.HandJoint.IndexTip &&
                (rightPinchOn || rightPointOn || rightGrabOn))
            {
                rightInteracting.Add(intersection.Item3);
                //Debug.Log("[Right] interacting with " + intersection.Item3.name);
            }
            else if (intersection.Item1 == OvrAvatar.HandType.Left &&
                intersection.Item2 == PoseManager.HandJoint.IndexTip &&
                (leftPinchOn || leftPointOn || leftGrabOn))
                leftInteracting.Add(intersection.Item3);
        }

        private static void IntersectionEndHandler(Tuple<OvrAvatar.HandType, PoseManager.HandJoint, Animatable> intersection)
        {
            // the third check is necessary since in general, multiple fingers will be intersecting with the object
            // and we only want to do this once
            if (intersection.Item1 == OvrAvatar.HandType.Right && (rightPinchOn || rightPointOn || rightGrabOn) &&
                intersection.Item2 == PoseManager.HandJoint.IndexTip && rightInteracting.Contains(intersection.Item3))
            {
                rightInteracting.Remove(intersection.Item3);
                //Debug.Log("[Right] no longer interacting with " + intersection.Item3.name);
            }
            else if (intersection.Item1 == OvrAvatar.HandType.Left && (leftPinchOn || leftPointOn || leftGrabOn) &&
                intersection.Item2 == PoseManager.HandJoint.IndexTip && leftInteracting.Contains(intersection.Item3))
            {
                leftInteracting.Remove(intersection.Item3);
                Debug.Log("[Left] no longer interacting with " + intersection.Item3.name);
            }
        }

        public static void TryInstantiatingAnimatable(OvrAvatar.HandType hand, ShelfObject shelfObj)
        {
            if (RecognitionState != GestureRecognizerState.Default && RecognitionState != GestureRecognizerState.Recognizing)
                return;

            if (hand == OvrAvatar.HandType.Right &&
                (rightAlmostPointOn || rightPointOn || rightGrabOn || rightPinchOn))
            {
                var anim = shelfObj.InstantiateAnimatable();
                if (anim != null)
                {
                    ObjectInstantiator = shelfObj;
                    ObjectBeingInstantiated = anim;
                    SetRecognitionState(GestureRecognizerState.Instantiating);
                }
            }
            else if (hand == OvrAvatar.HandType.Left &&
                (leftAlmostPointOn || leftPointOn || leftGrabOn || leftPinchOn))
            {
                var anim = shelfObj.InstantiateAnimatable();
                if (anim != null)
                {
                    ObjectInstantiator = shelfObj;
                    ObjectBeingInstantiated = anim;
                    SetRecognitionState(GestureRecognizerState.Instantiating);
                }
            }
        }

        public static void ProcessObjectDeletion(Animatable obj)
        {
            foreach (var child in obj.GetComponentsInChildren<Animatable>())
            {
                rightInteracting.Remove(child);
                leftInteracting.Remove(child);
            }

            TimeManager.StopTransforming(RecognitionState);
            SetRecognitionState(GestureRecognizerState.Default);
        }
    }
}
