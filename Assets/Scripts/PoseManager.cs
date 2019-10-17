using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using GestureAnim;
using System.Runtime.InteropServices;
using System;
using Oculus.Avatar;

namespace GestureAnim
{
    public class PoseManager : OvrAvatar
    {
        
        public enum FingerShape
        {
            Extended,   //Pointing
            Bent,
            Midway,
            Closed,

            Max
        }

        public new enum HandJoint
        {
            HandBase,
            ThumbBase,
            ThumbTip,
            IndexBase,
            IndexTip,
            MiddleBase,
            MiddleTip,
            RingBase,
            RingTip,
            PinkyBase,
            PinkyTip,
            Grip,
            Wrist,

            Max,
        }
        private static readonly HandJoint[] fingertips = { HandJoint.ThumbTip, HandJoint.IndexTip, HandJoint.MiddleTip, HandJoint.RingTip, HandJoint.PinkyTip };

        [Flags]
        public enum HandShape
        {
            // Basic states of fingers
            Unknown = 0,
            ThumbOpen = 1 << 0,
            ThumbMidway = 1 << 1,
            ThumbClosed = 1 << 2,
            IndexOpen = 1 << 3,
            IndexBent = 1 << 4,
            IndexMidway = 1 << 5,
            IndexClosed = 1 << 6,
            RestOpenOrBent = 1 << 7,
            RestMidway = 1 << 8,
            RestClosed = 1 << 9,

            // Compound flags based on the states of multiple fingers
            Fist = ThumbClosed | IndexClosed | RestClosed,
            Pointing = IndexOpen | RestClosed,
            AlmostPointing = IndexBent | RestClosed,
            ThumbsUp = ThumbOpen | IndexClosed | RestClosed,
            Extended = ThumbOpen | IndexOpen | RestOpenOrBent,
            Pinch = ThumbMidway | IndexMidway
        }

        static bool FlagTurnedOn(HandShape Old, HandShape New, HandShape flag) => 
            New.HasFlag(flag) && !Old.HasFlag(flag);

        static bool FlagTurnedOff(HandShape Old, HandShape New, HandShape flag) => 
            !New.HasFlag(flag) && Old.HasFlag(flag);


        // Canonical frame for a hand, independent of handedness
        // The three axes represent (when the hand is extended)
        // Finger base to tip
        // Thumb base to tip
        // Back of the hand to palm
        public enum HandFrame
        {
            Fingers,
            Thumb,
            Palm,

            Max
        }

        private static readonly Vector3[,] HAND_FRAME_VECTORS = new Vector3[(int)HandType.Max, (int)HandFrame.Max]
        {
            //HandType.Right
            //finger direction is the local -X axis, thumb is +Y, and palm is -Z
            {
                Vector3.left,
                Vector3.up,
                Vector3.back
            },
            //HandType.Left
            // The three directors are reversed for the left hand: (+X, -Y, +Z)
            {
                Vector3.right,
                Vector3.down,
                Vector3.forward
            }
        };

        
        private static string[,] HandJoints = new string[(int)HandType.Max, (int)HandJoint.Max]
        {
            //HandType.Right
            {
                "hands:r_hand_world",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_thumb1/hands:b_r_thumb2",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_thumb1/hands:b_r_thumb2/hands:b_r_thumb3/hands:b_r_thumb_ignore",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_index1",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_index1/hands:b_r_index2/hands:b_r_index3/hands:b_r_index_ignore",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_middle1",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_middle1/hands:b_r_middle2/hands:b_r_middle3/hands:b_r_middle_ignore",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_ring1",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_ring1/hands:b_r_ring2/hands:b_r_ring3/hands:b_r_ring_ignore",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_pinky0/hands:b_r_pinky1",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_pinky0/hands:b_r_pinky1/hands:b_r_pinky2/hands:b_r_pinky3/hands:b_r_pinky_ignore",
                "hands:r_hand_world/hands:b_r_hand/hands:b_r_grip",
                "hands:r_hand_world/hands:b_r_hand"
            },
            //HandType.Left
            {
                "hands:l_hand_world",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_thumb1/hands:b_l_thumb2",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_thumb1/hands:b_l_thumb2/hands:b_l_thumb3/hands:b_l_thumb_ignore",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_index1",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_index1/hands:b_l_index2/hands:b_l_index3/hands:b_l_index_ignore",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_middle1",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_middle1/hands:b_l_middle2/hands:b_l_middle3/hands:b_l_middle_ignore",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_ring1",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_ring1/hands:b_l_ring2/hands:b_l_ring3/hands:b_l_ring_ignore",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_pinky0/hands:b_l_pinky1",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_pinky0/hands:b_l_pinky1/hands:b_l_pinky2/hands:b_l_pinky3/hands:b_l_pinky_ignore",
                "hands:l_hand_world/hands:b_l_hand/hands:b_l_grip",
                "hands:l_hand_world/hands:b_l_hand"
            }
        };

        public struct PoseDataFrame
        {
            public Vector3[,] FingerPosition;
            public Vector3[] GripPosition;
            public FingerShape[,] FingerShape;
            public HandShape[] HandShape;
            public Vector3[] HandVelocity;
            public Vector3[] HandAcceleration;

            public PoseDataFrame(Vector3[,] ufp, Vector3[] ugp, FingerShape[,] ufs, HandShape[] uhs, Vector3[] vel, Vector3[] accel)
            {
                FingerPosition = ufp.Clone() as Vector3[,];
                GripPosition = ugp.Clone() as Vector3[];
                FingerShape = ufs.Clone() as FingerShape[,];
                HandShape = uhs.Clone() as HandShape[];
                HandVelocity = vel.Clone() as Vector3[];
                HandAcceleration = accel.Clone() as Vector3[];
            }
        }

        private static CircularBuffer<PoseDataFrame> cachedPoseData;

        //private Vector3[,] userFingertipPosition = new Vector3[(int)HandType.Max, fingertips.Length];
        //private Vector3[] userGripPosition = new Vector3[(int)HandType.Max];
        //private FingerShape[,] userFingerShape = new FingerShape[(int)HandType.Max, fingertips.Length];

        private static SphereCollider[,] fingertipCollider = new SphereCollider[(int)HandType.Max, fingertips.Length];

        private HandShape[] userHandShape = new HandShape[(int)HandType.Max];

        public delegate void LeftHandStartedGrab();
        public static event LeftHandStartedGrab OnLeftHandStartGrab;
        public delegate void LeftHandStoppedGrab();
        public static event LeftHandStoppedGrab OnLeftHandStopGrab;
        public delegate void LeftHandStartedPoint();
        public static event LeftHandStartedPoint OnLeftHandStartPoint;
        public delegate void LeftHandStoppedPoint();
        public static event LeftHandStoppedPoint OnLeftHandStopPoint;
        public delegate void LeftHandStartedPinch();
        public static event LeftHandStartedPinch OnLeftHandStartPinch;
        public delegate void LeftHandStoppedPinch();
        public static event LeftHandStoppedPinch OnLeftHandStopPinch;
        public delegate void LeftHandStartedAlmostPoint();
        public static event LeftHandStartedAlmostPoint OnLeftHandStartAlmostPoint;
        public delegate void LeftHandStoppedAlmostPoint();
        public static event LeftHandStoppedAlmostPoint OnLeftHandStopAlmostPoint;

        public delegate void RightHandStartedGrab();
        public static event RightHandStartedGrab OnRightHandStartGrab;
        public delegate void RightHandStoppedGrab();
        public static event RightHandStoppedGrab OnRightHandStopGrab;
        public delegate void RightHandStartedPoint();
        public static event RightHandStartedPoint OnRightHandStartPoint;
        public delegate void RightHandStoppedPoint();
        public static event RightHandStoppedPoint OnRightHandStopPoint;
        public delegate void RightHandStartedPinch();
        public static event RightHandStartedPinch OnRightHandStartPinch;
        public delegate void RightHandStoppedPinch();
        public static event RightHandStoppedPinch OnRightHandStopPinch;
        public delegate void RightHandStartedAlmostPoint();
        public static event RightHandStartedAlmostPoint OnRightHandStartAlmostPoint;
        public delegate void RightHandStoppedAlmostPoint();
        public static event RightHandStoppedAlmostPoint OnRightHandStopAlmostPoint;

        public delegate void HandShapeChanged();
        public static event HandShapeChanged OnHandShapeChange;

        public delegate void IntersectionStarted(Tuple<HandType, HandJoint, Animatable> tpl);
        public static event IntersectionStarted OnIntersectionStart;

        public delegate void IntersectionEnded(Tuple<HandType, HandJoint, Animatable> tpl);
        public static event IntersectionEnded OnIntersectionEnd;


        private static bool[] collidersNeedInitialization = new bool[(int)HandType.Max] {true, true};
        OvrAvatarHand[] _avatarHands = new OvrAvatarHand[(int)HandType.Max];

        public static List<Tuple<HandType, HandJoint, Animatable>> Intersections { get; private set; } = new List<Tuple<HandType, HandJoint, Animatable>>();

        DateTime lastHandPoseChangeTime = new DateTime();


        // Start is called before the first frame update
        new void Start()
        {
            base.Start();

            int s1 = (int)HandType.Max, s2 = fingertips.Length;
            Vector3[,] fingerPosition = new Vector3[s1, s2];
            Vector3[] gripPosition = new Vector3[s1];
            FingerShape[,] fingerShape = new FingerShape[s1, s2];
            HandShape[] handShape = new HandShape[s1];
            Vector3[] handVelocity = new Vector3[s1];
            Vector3[] handAcceleration = new Vector3[s1];

            for (int i=0; i<s1; ++i)
            {
                for(int j=0; j<s2; ++j)
                {
                    fingerPosition[i, j] = new Vector3();
                    fingerShape[i, j] = FingerShape.Max;
                }
                gripPosition[i] = new Vector3();
                handShape[i] = HandShape.Unknown;
                handVelocity[i] = new Vector3();
                handAcceleration[i] = new Vector3();
            }

            cachedPoseData = new CircularBuffer<PoseDataFrame>(Globals.FRAME_BUFFER_SIZE, 
                new PoseDataFrame(fingerPosition, gripPosition, fingerShape, handShape, handVelocity, handAcceleration));
            _avatarHands[0] = HandRight; _avatarHands[1] = HandLeft;
        }

        new void Update()
        {
            base.Update();

            List<HandType> hands = new List<HandType>();
            for (int i = 0; i < (int)HandType.Max; ++i)
                if (collidersNeedInitialization[i] && _avatarHands[i] != null)
                {
                    hands.Add((HandType)i);
                }
                else if (_avatarHands[i] == null)
                    collidersNeedInitialization[i] = true;
            InitializeColliders(hands);

            ProcessNewFrame();
        }

        private static void _OnHandShapeChange(PoseDataFrame oldData)
        {
            var curData = cachedPoseData.LatestFrame();
            var oldShape = oldData.HandShape;
            var newShape = curData.HandShape;

            OnHandShapeChange?.Invoke();

            int r = (int)HandType.Right, l = (int)HandType.Left;

            // Right hand OFF signals
            if (FlagTurnedOff(oldShape[r], newShape[r], HandShape.Pinch))
                OnRightHandStopPinch?.Invoke();
            else if (FlagTurnedOff(oldShape[r], newShape[r], HandShape.Fist))
                OnRightHandStopGrab?.Invoke();
            else if (FlagTurnedOff(oldShape[r], newShape[r], HandShape.Pointing))
                OnRightHandStopPoint?.Invoke();
            else if (FlagTurnedOff(oldShape[r], newShape[r], HandShape.AlmostPointing))
                OnRightHandStopAlmostPoint?.Invoke();

            //Right hand ON signals
            if (FlagTurnedOn(oldShape[r], newShape[r], HandShape.Pinch))
                OnRightHandStartPinch?.Invoke();
            else if (FlagTurnedOn(oldShape[r], newShape[r], HandShape.Fist))
                OnRightHandStartGrab?.Invoke();
            else if (FlagTurnedOn(oldShape[r], newShape[r], HandShape.Pointing))
                OnRightHandStartPoint?.Invoke();
            else if (FlagTurnedOn(oldShape[r], newShape[r], HandShape.AlmostPointing))
                OnRightHandStartAlmostPoint?.Invoke();


            // Left hand OFF signals
            if (FlagTurnedOff(oldShape[l], newShape[l], HandShape.Pinch))
                OnLeftHandStopPinch?.Invoke();
            else if (FlagTurnedOff(oldShape[l], newShape[l], HandShape.Fist))
                OnLeftHandStopGrab?.Invoke();
            else if (FlagTurnedOff(oldShape[l], newShape[l], HandShape.Pointing))
                OnLeftHandStopPoint?.Invoke();
            else if (FlagTurnedOff(oldShape[l], newShape[l], HandShape.AlmostPointing))
                OnLeftHandStopAlmostPoint?.Invoke();

            //Left hand ON signals
            if (FlagTurnedOn(oldShape[l], newShape[l], HandShape.Pinch))
                OnLeftHandStartPinch?.Invoke();
            else if (FlagTurnedOn(oldShape[l], newShape[l], HandShape.Fist))
                OnLeftHandStartGrab?.Invoke();
            else if (FlagTurnedOn(oldShape[l], newShape[l], HandShape.Pointing))
                OnLeftHandStartPoint?.Invoke();
            else if (FlagTurnedOn(oldShape[l], newShape[l], HandShape.AlmostPointing))
                OnLeftHandStartAlmostPoint?.Invoke();
        }

        private void InitializeColliders(List<HandType> hands)
        {
            // one for each finger
            foreach(var hand in hands)
            {
                int i = (int)hand;
                var transform = GetHandTransform((HandType)i, HandJoint.HandBase);
                if (transform == null)
                    continue;
                for (int j=0; j < fingertips.Length; ++j)
                {
                    transform = GetHandTransform((HandType)i, fingertips[j]);
                    var fingertipObject = transform.gameObject;
                    SphereCollider collider;
                    //Rigidbody rigidbody;

                    if (fingertipObject.GetComponent<SphereCollider>() == null)
                    {
                        collider = fingertipObject.AddComponent<SphereCollider>();
                        //rigidbody = fingertipObject.AddComponent<Rigidbody>();
                    }
                    else
                    {
                        collider = fingertipObject.GetComponent<SphereCollider>();
                        //rigidbody = fingertipObject.GetComponent<Rigidbody>();
                    }

                    collider.isTrigger = true;
                    //rigidbody.isKinematic = true;
                    collider.radius = 0.01f;    //1cm
                    collider.center = new Vector3();    //(0, 0, 0)

                    fingertipCollider[i, j] = collider;
                }
                collidersNeedInitialization[i] = false;
                Debug.Log(hand.ToString() + " colliders initialized!");
            }
        }

        private void ProcessNewFrame()
        {
            Vector3[,] fingerPosition;
            Vector3[] gripPosition;
            FingerShape[,] fingerShape;
            HandShape[] handShape;
            Vector3[] handVelocity;
            Vector3[] handAcceleration;

            _GetHandVelocityAndAceleration(out handVelocity, out handAcceleration);
            ComputeFingertipLocalPosition(out fingerPosition, out gripPosition);
            ComputeFingerShape(fingerPosition, out fingerShape);
            ComputeHandShape(fingerShape, out handShape);

            PoseDataFrame oldFrame = cachedPoseData.LatestFrame();

            cachedPoseData.Add(new PoseDataFrame(fingerPosition, gripPosition, fingerShape, handShape, handVelocity, handAcceleration));

            if (oldFrame.HandShape[0] != handShape[0] ||
                oldFrame.HandShape[1] != handShape[1])
            {
                lastHandPoseChangeTime = DateTime.Now;
                _OnHandShapeChange(oldFrame);
            }

        }

        public static Vector3 GetHandVelocity(HandType hand)
        {
            return cachedPoseData.LatestFrame().HandVelocity[(int)hand];
        }

        public static Vector3 GetHandAcceleration(HandType hand)
        {
            return cachedPoseData.LatestFrame().HandAcceleration[(int)hand];
        }

        private void _GetHandVelocityAndAceleration(out Vector3[] velocity, out Vector3[] acceleration)
        {
            velocity = new Vector3[2];
            acceleration = new Vector3[2];

            if (HandLeft)
            {
                velocity[(int)HandType.Left] = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.LTouch);
                acceleration[(int)HandType.Left] = OVRInput.GetLocalControllerAcceleration(OVRInput.Controller.LTouch);
            }
            else
            {
                velocity[(int)HandType.Left] = new Vector3();
                acceleration[(int)HandType.Left] = new Vector3();
            }

            if (HandRight)
            {
                velocity[(int)HandType.Right] = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.RTouch);
                acceleration[(int)HandType.Right] = OVRInput.GetLocalControllerAcceleration(OVRInput.Controller.RTouch);
            }
            else
            {
                velocity[(int)HandType.Right] = new Vector3();
                acceleration[(int)HandType.Right] = new Vector3();
            }
        }

        // Get the transform for a hand-joint
        public Transform GetHandTransform(HandType hand, HandJoint joint)
        {
            if (hand >= HandType.Max || joint >= HandJoint.Max)
            {
                return null;
            }

            var HandObject = hand == HandType.Left ? HandLeft : HandRight;

            if (HandObject != null)
            {
                var AvatarComponent = HandObject.GetComponent<OvrAvatarComponent>();
                if (AvatarComponent != null && AvatarComponent.RenderParts.Count > 0)
                {
                    var SkinnedMesh = AvatarComponent.RenderParts[0];
                    return SkinnedMesh.transform.Find(HandJoints[(int)hand, (int)joint]);
                }
            }

            return null;
        }

        // Get local position of a joint w.r.t the wrist of that hand
        private void ComputeFingertipLocalPosition(out Vector3[,] userFingerPosition, out Vector3[] userGripPosition)
        {
            userFingerPosition = new Vector3[(int)HandType.Max, fingertips.Length];
            userGripPosition = new Vector3[(int)HandType.Max];

            for (int i= 0; i<(int)HandType.Max; ++i)
            {
                var hand = (HandType)i;
                if (hand == HandType.Left && HandLeft == null)
                    continue;
                else if (hand == HandType.Right && HandRight == null)
                    continue;

                var baseTransform = GetHandTransform(hand, HandJoint.Wrist);
                if (baseTransform == null)
                    continue;

                var grip = GetHandTransform(hand, HandJoint.Grip).position;
                Vector3[] fingerPos = {
                    GetHandTransform(hand, HandJoint.ThumbTip).position,
                    GetHandTransform(hand, HandJoint.IndexTip).position,
                    GetHandTransform(hand, HandJoint.MiddleTip).position,
                    GetHandTransform(hand, HandJoint.RingTip).position,
                    GetHandTransform(hand, HandJoint.PinkyTip).position };

                //store fingertip Position
                for (int j = 0; j < fingerPos.Length; ++j)
                    userFingerPosition[(int)hand, j] = baseTransform.InverseTransformPoint(fingerPos[j]);

                //store position of the "grip" joint as well
                userGripPosition[(int)hand] = baseTransform.InverseTransformPoint(grip);
            }
        }

        private static int JointFingerMap(HandJoint joint)
        {
            int fingerIdx = -1;
            switch (joint)
            {
                case HandJoint.ThumbTip:
                    fingerIdx = 0;
                    break;
                case HandJoint.IndexTip:
                    fingerIdx = 1;
                    break;
                case HandJoint.MiddleTip:
                    fingerIdx = 2;
                    break;
                case HandJoint.RingTip:
                    fingerIdx = 3;
                    break;
                case HandJoint.PinkyTip:
                    fingerIdx = 4;
                    break;
            }
            return fingerIdx;
        }

        // Get the Thumb, Finger or Palm directors of a hand
        public Vector3 GetHandFrameVector(HandType hand, HandFrame vec)
        {
            var handTrans = GetHandTransform(hand, HandJoint.Wrist);
            return handTrans.TransformDirection(HAND_FRAME_VECTORS[(int)hand, (int)vec]);
        }

        // Only works when the input joint is a fingertip
        // returns FingerState.Max otherwise
        public static FingerShape GetFingerShape(HandType hand, HandJoint joint)
        {
            return _GetFingerShapeFromPosition(hand, joint, cachedPoseData.LatestFrame().FingerPosition);
        }

        // Only works when the input joint is a fingertip
        // returns FingerState.Max otherwise
        private static FingerShape _GetFingerShapeFromPosition(HandType hand, HandJoint joint, Vector3[,] userFingertipPosition)
        {
            var thumbVector = HAND_FRAME_VECTORS[(int)hand, (int)HandFrame.Thumb];
            var fingerVector = HAND_FRAME_VECTORS[(int)hand, (int)HandFrame.Fingers];
            var palmVector = HAND_FRAME_VECTORS[(int)hand, (int)HandFrame.Palm];

            var fingerIdx = JointFingerMap(joint);
            if (fingerIdx >= 0)
                joint = fingertips[fingerIdx];
            else
                joint = HandJoint.Max;
            if (joint == HandJoint.ThumbTip)
            {
                var pos = userFingertipPosition[(int)hand, 0];
                if (Vector3.Dot(pos, thumbVector) > 0.1)
                    return FingerShape.Extended;
                else if (Vector3.Dot(pos, thumbVector) > 0.02)
                    return FingerShape.Midway;
                else
                    return FingerShape.Closed;
            }
            else if (joint == HandJoint.IndexTip)
            {
                var pos = userFingertipPosition[(int)hand, fingerIdx];
                if (Vector3.Dot(pos, fingerVector) > 0.13 && Vector3.Dot(pos, palmVector) < 0.01)
                    return FingerShape.Extended;
                else if (Vector3.Dot(pos, fingerVector) > 0.112)
                    //else if (Vector3.Dot(pos, fingerVector) > 0.1 && Vector3.Dot(pos, palmVector) < 0.05)
                    return FingerShape.Bent;
                else if (Vector3.Dot(pos, palmVector) > 0.05)
                    return FingerShape.Midway;
                else
                    return FingerShape.Closed;
            }
            else if (joint != HandJoint.Max)
            {
                var pos = userFingertipPosition[(int)hand, fingerIdx];
                if (Vector3.Dot(pos, fingerVector) > 0.13 && Vector3.Dot(pos, palmVector) < 0.01)
                    return FingerShape.Extended;
                else if (Vector3.Dot(pos, fingerVector) > 0.1 && Vector3.Dot(pos, palmVector) < 0.05)
                    return FingerShape.Bent;
                else if (Vector3.Dot(pos, palmVector) > 0.05)
                    return FingerShape.Midway;
                else
                    return FingerShape.Closed;
            }
            else
                return FingerShape.Max;
        }

        private static void ComputeFingerShape(Vector3[,] userFingertipPosition, out FingerShape[,] userFingerShape)
        {
            userFingerShape = new FingerShape[(int)HandType.Max, fingertips.Length];

            for (int i=0; i<(int)HandType.Max; ++i)
            {
                for (int j = 0; j < fingertips.Length; ++j)
                    userFingerShape[i, j] = _GetFingerShapeFromPosition((HandType)i, fingertips[j], userFingertipPosition);
            }
        }

        public static Vector3 GetFingertipLocalPosition(HandType hand, HandJoint joint)
        {
            if ((int)hand >= (int)HandType.Max)
                return new Vector3();

            var fingerIdx = JointFingerMap(joint);

            if (fingerIdx == -1)
                return new Vector3();
            else
                return cachedPoseData.LatestFrame().FingerPosition[(int)hand, fingerIdx];
        }

        public static HandShape GetHandShape(HandType hand)
        {
            return cachedPoseData.LatestFrame().HandShape[(int)hand];
        }

        private static void ComputeHandShape(FingerShape[,] userFingerShape, out HandShape[] userHandShape)
        {
            userHandShape = new HandShape[(int)HandType.Max];

            for(int i=0; i<(int)HandType.Max; ++i)
            {
                var hand = (HandType)i;
                userHandShape[i] = HandShape.Unknown;

                switch(userFingerShape[i, JointFingerMap(HandJoint.ThumbTip)])
                {
                    case FingerShape.Extended:
                    case FingerShape.Bent:
                        userHandShape[i] |= HandShape.ThumbOpen;
                        break;
                    case FingerShape.Midway:
                        userHandShape[i] |= HandShape.ThumbMidway;
                        break;
                    case FingerShape.Closed:
                        userHandShape[i] |= HandShape.ThumbClosed;
                        break;
                }

                switch (userFingerShape[i, JointFingerMap(HandJoint.IndexTip)])
                {
                    case FingerShape.Extended:
                        userHandShape[i] |= HandShape.IndexOpen;
                        break;
                    case FingerShape.Bent:
                        userHandShape[i] |= HandShape.IndexBent;
                        break;
                    case FingerShape.Midway:
                        userHandShape[i] |= HandShape.IndexMidway;
                        break;
                    case FingerShape.Closed:
                        userHandShape[i] |= HandShape.IndexClosed;
                        break;
                }

                switch (userFingerShape[i, JointFingerMap(HandJoint.MiddleTip)])
                {
                    case FingerShape.Extended:
                    case FingerShape.Bent:
                        userHandShape[i] |= HandShape.RestOpenOrBent;
                        break;
                    case FingerShape.Midway:
                        userHandShape[i] |= HandShape.RestMidway;
                        break;
                    case FingerShape.Closed:
                        userHandShape[i] |= HandShape.RestClosed;
                        break;
                }
            }
        }


        public static void OnHandObjectCollisionStart(Animatable other, GameObject handPart)
        {
            HandType hand; HandJoint joint;
            GetJointFromGameobjectName(handPart.name, out hand, out joint);
            if (hand == HandType.Max || other == null)
                return;

            var tpl = new Tuple<HandType, HandJoint, Animatable>(hand, joint, other);
            if (!Intersections.Contains(tpl))
            {
                Intersections.Add(tpl);
                OnIntersectionStart?.Invoke(tpl);
            }
        }

        public static void OnHandObjectCollisionStay(Animatable other, GameObject handPart)
        {
            HandType hand; HandJoint joint;
            GetJointFromGameobjectName(handPart.name, out hand, out joint);
            if (hand == HandType.Max || other == null)
                return;
        }

        public static void OnHandObjectCollisionEnd(Animatable other, GameObject handPart)
        {
            HandType hand; HandJoint joint;
            GetJointFromGameobjectName(handPart.name, out hand, out joint);
            if (hand == HandType.Max)
                return;

            var tpl = new Tuple<HandType, HandJoint, Animatable>(hand, joint, other);
            if (Intersections.Remove(tpl) == false)
                Debug.LogWarning("Collision-end detected on an object without collision-start.");
            else
                OnIntersectionEnd?.Invoke(tpl);
        }

        public static void DeleteReferencesToObject(Animatable obj)
        {
            var deleteKeys = new List<int>(Intersections.Count);
            for (int i = Intersections.Count - 1; i >= 0; --i)
                if (Intersections[i].Item3.GetInstanceID() == obj.GetInstanceID())
                {
                    OnIntersectionEnd?.Invoke(Intersections[i]);
                    Intersections.RemoveAt(i);
                }
        }

        public static void GetJointFromGameobjectName(string name, out HandType hand, out HandJoint joint)
        {
            if (name.Contains("_r_"))
                hand = HandType.Right;
            else if (name.Contains("_l_"))
                hand = HandType.Left;
            else
                hand = HandType.Max;

            if (name.Contains("thumb"))
                joint = HandJoint.ThumbTip;
            else if (name.Contains("index"))
                joint = HandJoint.IndexTip;
            else if (name.Contains("middle"))
                joint = HandJoint.MiddleTip;
            else if (name.Contains("ring"))
                joint = HandJoint.RingTip;
            else if (name.Contains("pinky"))
                joint = HandJoint.PinkyTip;
            else
                joint = HandJoint.Max;
        }
    }
}
