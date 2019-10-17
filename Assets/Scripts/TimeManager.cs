using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    public class TimeManager : MonoBehaviour
    {
        static List<Animatable> objects = new List<Animatable>();
        public static int CurFrame { get; private set; } = 0;
        public static int NumFrame { get; private set; } = 1;
        private static int NumFrameBeforeGesture = 1;

        static GameObject initialObjectPose;   // initial transformation of the object being controlled

        static Stack<Tuple<Animatable, Operation, int>> undoStack = new Stack<Tuple<Animatable, Operation, int>>(20);

        static Stack<List<Tuple<int, Vector3>>> translationStack = new Stack<List<Tuple<int, Vector3>>>(20);
        static Stack<List<Tuple<int, Quaternion>>> rotationStack = new Stack<List<Tuple<int, Quaternion>>>(20);
        static Stack<List<Tuple<int, Vector3>>> scaleStack = new Stack<List<Tuple<int, Vector3>>>(20);
        static Stack<Tuple<List<Vector3>, List<float>, List<Tuple<float, ParticleEmitter.EmissionData>>>> emissionCurveStack = 
            new Stack<Tuple<List<Vector3>, List<float>, List<Tuple<float, ParticleEmitter.EmissionData>>>>(20);
        static Stack<Tuple<float, float>> emissionNoiseStack = new Stack<Tuple<float, float>>(20);
        static Stack<float> emissionSpiralStack = new Stack<float>(20);

        //static Func<Vector3, Vector3> dirW2P = x => {
        //    if (initialObjectPose.transform.parent == null)
        //        return x;
        //    else
        //        return initialObjectPose.transform.parent.InverseTransformDirection(x);
        //    };

        //static Func<Vector3, Vector3> ptW2P = x => {
        //    if (initialObjectPose.transform.parent == null)
        //        return x;
        //    else
        //        return initialObjectPose.transform.parent.InverseTransformPoint(x);
        //};

        //static Func<Vector3, Vector3> vecW2P = x => {
        //    if (initialObjectPose.transform.parent == null)
        //        return x;
        //    else
        //        return initialObjectPose.transform.parent.InverseTransformVector(x);
        //};

        static Vector3 initialHandPosition;    // initial hand position: useful for translation
        static float initialInterHandDistance; // initial distance b/w two hands (for scaling)
        static Vector3 rotationAxis;           // axis of rotation in the controlled object's initial local space
        static Vector3 initialHandDirection;   // normalized component of initialHandPosition orthogonal to rotationAxis (useful for rotation)

        static public PoseManager poseManager;

        static Animatable controlledObject = null;
        public static Animatable SelectedObject { get; set; } = null;

        public static bool IsRecording { get; set; } = false;
        public static bool IsAnimationPlaying { get; private set; } = false;
        static DateTime lastFrameTime = DateTime.MinValue;

        public static bool KeepOldRotationAxis { get; set; } = false;

        public delegate void AnimationPausedInScript();
        public static event AnimationPausedInScript OnAnimationPauseInScript;
        public delegate void AnimationPlayedInScript();
        public static event AnimationPlayedInScript OnAnimationPlayInScript;

        public Animatable[] PreRegisteredObjects;

        public TMPro.TextMeshPro FrameCounter;

        void Start()
        {
            poseManager = FindObjectOfType<PoseManager>();

            initialObjectPose = new GameObject();
            initialObjectPose.SetActive(false);

            foreach (var obj in PreRegisteredObjects)
                obj.Init(null);

            //var emitter = PreRegisteredObjects[0].GetComponentInChildren<ParticleEmitter>();
            //emitter.TryChangeParticleShapeFromDroppedObject(FindObjectOfType<ShelfObject>());
        }

        // Update is called once per frame
        void Update()
        {
            bool newFrame = false;

            if (IsAnimationPlaying && !IsRecording && (DateTime.Now - lastFrameTime).TotalSeconds < Globals.INV_FPS)
            {
                return;
            }
            else if (IsAnimationPlaying)
            {
                //Debug.Log((DateTime.Now - lastFrameTime).TotalSeconds.ToString("F3"));
                
                if (Mathf.FloorToInt((float)(DateTime.Now - lastFrameTime).TotalSeconds * Globals.FPS) > 0)
                {
                    // Use FPS to determine which frame to move to (NOTE: this might skip frames)
                    CurFrame = CurFrame + Mathf.FloorToInt((float)(DateTime.Now - lastFrameTime).TotalSeconds * Globals.FPS);

                    // Store ideal execution time for this frame
                    lastFrameTime = DateTime.Now.AddSeconds(-((DateTime.Now - lastFrameTime).TotalSeconds % Globals.INV_FPS));

                    newFrame = true;
                }

                //automatically add more frames if needed
                if (controlledObject != null && CurFrame >= NumFrame)
                {
                    Debug.Log("Adding additional frames: " + NumFrame + " -> " + (CurFrame + 1));
                    NumFrame = CurFrame + 1;
                }

                CurFrame = CurFrame % NumFrame;
                foreach (var obj in objects)
                {
                    obj.EvaluateTransform(CurFrame);
                }
            }

            if (controlledObject != null)
            {
                switch (GestureManager.RecognitionState)
                {
                    case GestureRecognizerState.Translating:
                        var curHandPosition = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position;
                        var addVector = (controlledObject.transform.parent == null) ?
                        (curHandPosition - initialHandPosition) :
                        controlledObject.transform.parent.InverseTransformVector(curHandPosition - initialHandPosition);
                        var position = initialObjectPose.transform.localPosition + addVector;

                        if (newFrame)
                            controlledObject.AddTranslationKey(position, CurFrame);
                        else
                            controlledObject.transform.localPosition = position;
                        break;

                    case GestureRecognizerState.Scaling:
                        var curInterHandDistance = (
                            poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position -
                            poseManager.GetHandTransform(OvrAvatar.HandType.Left, PoseManager.HandJoint.IndexTip).position
                            ).magnitude;
                        var scale = initialObjectPose.transform.localScale * (curInterHandDistance / (initialInterHandDistance + float.Epsilon));

                        if (newFrame)
                            controlledObject.AddScaleKey(scale, CurFrame);
                        else
                            controlledObject.transform.localScale = scale;
                        break;

                    case GestureRecognizerState.Rotating:
                        var rotation = ComputeCurrentRotation();
                        rotation = ((initialObjectPose.transform.parent == null) ?
                            Quaternion.identity :
                            Quaternion.Inverse(initialObjectPose.transform.parent.rotation)) * rotation;
                        if (newFrame)
                            controlledObject.AddRotationKey(rotation, CurFrame);
                        else
                            controlledObject.transform.localRotation = rotation;
                        break;
                }
                if (newFrame)
                {
                    controlledObject.EvaluateTransform(CurFrame);
                    //Debug.Log(controlledObject.transform.localScale.ToString("F2"));
                }
            }
            SetFrameCounterText();
        }

        static Quaternion ComputeCurrentRotation()
        {
            var curHandPosition = poseManager.GetHandTransform(OvrAvatar.HandType.Left, PoseManager.HandJoint.IndexTip).position;
            curHandPosition -= initialObjectPose.transform.position;
            var curHandDirection = (curHandPosition - rotationAxis * Vector3.Dot(curHandPosition, rotationAxis)).normalized;

            Quaternion delta = Quaternion.FromToRotation(initialHandDirection, curHandDirection);
            
            return delta * initialObjectPose.transform.rotation;
        }

        public static void RegisterObject(Animatable obj, bool createInitialKeys = true)
        {
            objects.Add(obj);
            var trans = obj.transform;
            if (createInitialKeys)
            {
                obj.StartFrame = CurFrame;
                obj.AddTranslationKey(trans.localPosition, 0);
                obj.AddRotationKey(trans.localRotation, 0);
                obj.AddScaleKey(trans.localScale, 0);
            }
        }

        public static void DeregisterObject(Animatable obj)
        {
            objects.Remove(obj);
        }

        public static void SelectObject(Animatable obj)
        {
            if (SelectedObject != null)
                SelectedObject.Deselected();

            obj.Selected();
        }

        private static void _StartTransforming(Animatable obj)
        {
            controlledObject = obj;
            initialObjectPose.transform.SetParent(obj.transform.parent, true);
            initialObjectPose.transform.SetPositionAndRotation(obj.transform.position, obj.transform.rotation);
            initialObjectPose.transform.localScale = obj.transform.localScale;
            NumFrameBeforeGesture = NumFrame;
            
            //Debug.Log("Starting with" + initialObjectPose.transform.localScale.ToString("F2"));

            if (IsRecording)
                PlayAnimationInScript();
            else
                PauseAnimationInScript();

            obj.Selected();
        }

        public static void StartTranslation(Animatable obj)
        {
            _StartTransforming(obj);

            initialHandPosition = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position;
            StackTranslation(controlledObject.TranslationKeys, controlledObject, 0);
        }

        public static void StartScaling(Animatable obj)
        {
            _StartTransforming(obj);

            initialInterHandDistance = (
                poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position -
                poseManager.GetHandTransform(OvrAvatar.HandType.Left, PoseManager.HandJoint.IndexTip).position
                ).magnitude;
            StackScale(controlledObject.ScaleKeys, controlledObject, 0);
        }

        public static void StartRotation(Animatable obj)
        {
            _StartTransforming(obj);

            if (!KeepOldRotationAxis)
                rotationAxis = (
                    Globals.POINTING_FINGER_COORDINATES.x * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Fingers) +
                    Globals.POINTING_FINGER_COORDINATES.y * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Thumb) +
                    Globals.POINTING_FINGER_COORDINATES.z * poseManager.GetHandFrameVector(OvrAvatar.HandType.Right, PoseManager.HandFrame.Palm)
                    ).normalized;

            var handPos = poseManager.GetHandTransform(OvrAvatar.HandType.Left, PoseManager.HandJoint.IndexTip).position;
            handPos -= initialObjectPose.transform.position;

            initialHandDirection = (handPos - rotationAxis*Vector3.Dot(handPos, rotationAxis)).normalized;
            StackRotation(controlledObject.RotationKeys, controlledObject, 0);
        }

        public static void StopTransforming(GestureRecognizerState state)
        {
            var opTpl = undoStack.Pop();
            undoStack.Push(Tuple.Create(opTpl.Item1, opTpl.Item2, NumFrame - NumFrameBeforeGesture));

            switch (state)
            {
                case GestureRecognizerState.Translating:
                    var curHandPosition = poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position;
                    var addVector = (controlledObject.transform.parent == null) ?
                        (curHandPosition - initialHandPosition) :
                        controlledObject.transform.parent.InverseTransformVector(curHandPosition - initialHandPosition);
                    controlledObject.AddTranslationKey(initialObjectPose.transform.localPosition + addVector, CurFrame);
                    break;

                case GestureRecognizerState.Scaling:
                    var curInterHandDistance = (
                        poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position -
                        poseManager.GetHandTransform(OvrAvatar.HandType.Left, PoseManager.HandJoint.IndexTip).position
                        ).magnitude;
                    controlledObject.AddScaleKey(initialObjectPose.transform.localScale * (curInterHandDistance / (initialInterHandDistance + float.Epsilon)), CurFrame);
                    break;

                case GestureRecognizerState.Rotating:
                    var rotation = ComputeCurrentRotation();
                    rotation = ((initialObjectPose.transform.parent == null) ?
                            Quaternion.identity :
                            Quaternion.Inverse(initialObjectPose.transform.parent.rotation)) * rotation;
                    controlledObject.AddRotationKey(rotation, CurFrame);
                    break;
            }
            controlledObject.EvaluateTransform(CurFrame);
            //Debug.Log(controlledObject.transform.localScale.ToString("F2"));
            controlledObject.Deselected();
            controlledObject = null;
            if (IsRecording)
                PauseAnimationInScript();

        }

        public static void PlayAnimation()
        {
            IsAnimationPlaying = true;
            lastFrameTime = DateTime.Now.AddSeconds(-Globals.INV_FPS);
            Debug.Log("Playing animation now. #frames: " + NumFrame);
        }

        public static void PauseAnimation()
        {
            IsAnimationPlaying = false;
            Debug.Log("Animation paused at frame #" + CurFrame);
        }

        public static void StartRecording()
        {
            IsRecording = true;
            Debug.Log("Recording gestural manipulations now!");
        }

        public static void StopRecording()
        {
            IsRecording = false;
            Debug.Log("Stopped recording gestural manipulations (only posing now).");
        }

        public static void SwitchAnimationPlayState()
        {
            if (IsAnimationPlaying)
                PauseAnimation();
            else
                PlayAnimation();
        }

        public static void SwitchRecordingState()
        {
            if (GestureManager.RecognitionState == GestureRecognizerState.Default || GestureManager.RecognitionState == GestureRecognizerState.Recognizing)
            {
                if (IsRecording)
                    StopRecording();
                else
                    StartRecording();
            }
        }

        public static void PauseAnimationInScript()
        {
            OnAnimationPauseInScript?.Invoke();
            PauseAnimation();
        }

        public static void PlayAnimationInScript()
        {
            OnAnimationPlayInScript?.Invoke();
            PlayAnimation();
        }

        public static void SetFrameCount(int n)
        {
            NumFrame = n;
        }

        public void GoToNextFrame()
        {
            CurFrame = (CurFrame + 1) % NumFrame;
            UIFrameChange();
        }

        public void GoToPreviousFrame()
        {
            CurFrame = (CurFrame==0) ? 
                NumFrame - 1 : 
                (CurFrame - 1) % NumFrame;
            UIFrameChange();
        }

        public void GoToFirstFrame()
        {
            CurFrame = 0;
            UIFrameChange();
        }

        public void GoToLastFrame()
        {
            CurFrame = NumFrame - 1;
            UIFrameChange();
        }

        private void UIFrameChange()
        {
            foreach (var obj in objects)
            {
                obj.EvaluateTransform(CurFrame);
            }
        }

        private void SetFrameCounterText()
        {
            FrameCounter.SetText((CurFrame+1).ToString("D4") + "\n――\n" + NumFrame.ToString("D4"));
        }

        public static void UndoLastOperation()
        {
            if (undoStack.Count == 0)
            {
                Debug.Log("Undo stack is empty!");
                return;
            }

            var opTuple = undoStack.Pop();
            var obj = opTuple.Item1;
            var op = opTuple.Item2;
            var numDeleteFrames = opTuple.Item3;

            Debug.Log("Undoing " + op.ToString() + " for " + obj?.name);

            switch(op)
            {
                case Operation.Translate:
                    obj?.SetTranslationKeys(translationStack.Pop());
                    break;
                case Operation.Rotate:
                    obj?.SetRotationKeys(rotationStack.Pop());
                    break;
                case Operation.Scale:
                    obj?.SetScaleKeys(scaleStack.Pop());
                    break;
                case Operation.SetEmissionCone:
                    obj?.TrySetEmissionCone(emissionCurveStack.Pop());
                    break;
                case Operation.SetEmissionNoise:
                    obj?.TrySetEmissionNoise(emissionNoiseStack.Pop());
                    break;
                case Operation.SetEmissionSpiral:
                    obj?.TrySetEmissionSpiral(emissionSpiralStack.Pop());
                    break;
                default:
                    Debug.LogWarning("Cannot undo " + op.ToString());
                    break;
            }

            NumFrame -= numDeleteFrames;
            CurFrame = CurFrame % NumFrame;
        }

        public static void StackTranslation(List<Tuple<int, Vector3>> t, Animatable obj, int numFramesAdded)
        {
            translationStack.Push(new List<Tuple<int, Vector3>>(t));
            undoStack.Push(Tuple.Create(obj, Operation.Translate, numFramesAdded));
        }

        public static void StackRotation(List<Tuple<int, Quaternion>> r, Animatable obj, int numFramesAdded)
        {
            rotationStack.Push(new List<Tuple<int, Quaternion>>(r));
            undoStack.Push(Tuple.Create(obj, Operation.Rotate, numFramesAdded));
        }

        public static void StackScale(List<Tuple<int, Vector3>> s, Animatable obj, int numFramesAdded)
        {
            scaleStack.Push(new List<Tuple<int, Vector3>>(s));
            undoStack.Push(Tuple.Create(obj, Operation.Scale, numFramesAdded));
        }

        public static void StackEmissionCone(
            Tuple<List<Vector3>, List<float>, List<Tuple<float, ParticleEmitter.EmissionData>>> e,
            Animatable obj)
        {
            emissionCurveStack.Push(Tuple.Create(
                new List<Vector3>(e.Item1),
                new List<float>(e.Item2),
                new List<Tuple<float, ParticleEmitter.EmissionData>>(e.Item3)
            ));
            undoStack.Push(Tuple.Create(obj, Operation.SetEmissionCone, 0));
        }

        public static void StackEmissionNoise(Tuple<float, float> n, Animatable obj)
        {
            emissionNoiseStack.Push(n);
            undoStack.Push(Tuple.Create(obj, Operation.SetEmissionNoise, 0));
        }

        public static void StackEmissionSpiral(float s, Animatable obj)
        {
            emissionSpiralStack.Push(s);
            undoStack.Push(Tuple.Create(obj, Operation.SetEmissionSpiral, 0));
        }
    }
}