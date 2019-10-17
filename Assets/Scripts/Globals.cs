using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;


namespace GestureAnim
{
    //buttons
    public enum Button { RightA = 0, RightB = 1, LeftX = 2, LeftY = 3, LeftStart = 7, LeftStick = 8, RightStick = 9};
    public enum Touch { RightA = 10, RightB = 11, LeftX = 12, LeftY = 13, LeftStick = 16, RightStick = 17, LeftThumbrest = 18, RightThumbrest = 19, LeftIndexTrigger = 14, RightIndexTrigger = 15};
    public enum NearTouch { LeftStick = 15, RightStick = 16, LeftIndex = 13, RightIndex = 14};
    public enum Trigger { LeftIndex = 9, RightIndex = 10, LeftMiddle = 11, RightMiddle = 12};
    public enum Axis { LeftHorizontal = 1, LeftVertical = 2, RightHorizontal = 4, RightVertical = 5};

    public enum Operation
    {
        Translate, Rotate, Scale,
        Instantiate, Delete, SetParticle, 
        SetEmissionCone, SetEmissionNoise, SetEmissionSpiral
    }

    public enum OculusTouchController { Left, Right };

    [Flags]
    public enum GestureRecognizerState {
        Default = 0,
        Recognizing = 1 << 0,
        Translating = 1 << 1,
        Rotating = 1 << 2,
        Scaling = 1 << 3,
        ReadyToRotate = 1 << 4,
        ReadyToSelect = 1 << 5,
        AfterSelectGesture = 1 << 6,
        DrawingEmissionCone = 1 << 7,
        ReadyForConeGesture = 1 << 8,
        RecognizingConeGesture = 1 << 9,
        Instantiating = 1 << 10,

        Transforming = Translating | Rotating | Scaling
    };

    
    public static class Globals
	{
        // number of hand pose frames to store for recognition tasks
        public const int FRAME_BUFFER_SIZE = 200;

        // FPS for the animation
        public const int FPS = 30;
        public const double INV_FPS = 1.0 / FPS;

        // Time (seconds) to wait before gesture is recognized.
        // Useful for gestures which can have multiple meanings (none yet)
        // Or for gestures where there the recognition hint for a bi-manual gesture cannot be differentiated
        // from that of another unimanual gesture (e.g. translation vs. scale)
        // Can also be useful for bimanual gestures which share the pose for one hand (e.g. rotation vs. scale)
        public const double GESTURE_WAIT_TIME = 0.3;
        public const float TAP_GESTURE_MAX_TIME = 1.5f;
        public const float SHELF_GESTURE_WAITING_PERIOD = 1.0f;

        // Bezier fitting parameters
        public const float RDP_ERROR = 0.001f;
        public const float BEZIER_FIT_MAX_ERROR = 0.01f;

        // # of time (and arc length) values stored on each cubic Bezier
        public const int BEZIER_PARAM_CACHE_NUM_SAMPLES = 100;

        public const int MAX_PARTICLES = 1000;
        public const float PARTICLE_DEFAULT_START_SPEED = 0.5f;     // start at 0.5 m/s
        public const float PARTICLE_EMITTER_DEFAULT_SIZE = 0.25f;
        public const int PARTICLE_DEFAULT_EMISSION_RATE = 100;
        public const float EMISSION_CURVE_RENDER_WIDTH = 0.01f;
        public const int EMISSION_CONE_RENDER_DETAIL = 10;          // #vertices per ring
        public const float CONE_GESTURE_CLASSIFICATION_SINGULAR_VALUE_RATIO = 2.0f;

        public const PoseManager.HandJoint EMISSION_GESTURE_JOINT = PoseManager.HandJoint.Grip;

        public static readonly Vector3 POINTING_FINGER_COORDINATES = new Vector3(0.9727f, 0.2306f, 0.0251f);

        public static GameObject CurrentSelection = null;
        public static readonly Material SELECTION_MATERIAL = Resources.Load<Material>("SelectedObject");
        public const float SELECTION_ANIMATION_DURATION = 0.5f;

        public const int OCULUS_TRACKING_UPDATE_FREQUENCY = 90;
        public const float OCULUS_TRACKING_UPDATE_TIME = 1.0f / OCULUS_TRACKING_UPDATE_FREQUENCY;

        public const float UI_BUTTON_ANIMATION_TIME = 0.2f;
        public const float UI_BUTTON_WAIT_TIME = 0.5f;
    }

    // Circular buffer implementation by Heslacher@codereview.stackexchange (adapted)
    // https://codereview.stackexchange.com/questions/133926/circular-buffer-implementation
    public class CircularBuffer<T>
    {
        private readonly Queue<T> _data;
        //private readonly ReaderWriterLockSlim _lock = new ReaderWriterLockSlim();
        private readonly int _size;
        private readonly T _default;

        public CircularBuffer(int size, T defaultMember)
        {
            if (size < 1)
            {
                throw new ArgumentException($"{nameof(size)} cannot be negative or zero");
            }
            _data = new Queue<T>();
            _size = size;
            _default = defaultMember;
        }

        public IEnumerable<T> Data()
        {
            return _data.ToArray();
        }

        public T LatestFrame()
        {
            return _data.DefaultIfEmpty(_default).LastOrDefault();
        }

        public void Add(T t)
        {
            //_lock.EnterWriteLock();
            try
            {
                if (_data.Count == _size)
                {
                    //T value;
                    //_data.TryDequeue(out value);
                    _data.Dequeue();
                }

                _data.Enqueue(t);
            }
            finally
            {
                //_lock.ExitWriteLock();
            }
        }
    }

    public class ParticleCustomData
    {
        public Vector3[] Xn_1 { get; set; } = new Vector3[0];   // position at the (n-1)th timestep
        public Vector3[] Xn { get; set; } = new Vector3[0];   // position at the (n)th timestep

        public bool InUse { get; set; } = true;             // is this object currently in use?

        public int NumTimestep = 0;                          // #timesteps since the particle's birth

        public DateTime LastTime = DateTime.Now;

        public float Delta;

        public float Theta0;

        public Vector3 Nn_1;

        public float[] PositionNoiseSeed = new float[2];
        public float[] RotationNoiseSeed = new float[2];

        public float ParamPhase = 0.0f;

        public ParticleCustomData(int numForceFields)
        {
            Xn_1 = new Vector3[numForceFields + 1];
            Xn = new Vector3[numForceFields + 1];

            for (int i=0; i<= numForceFields; ++i)
            {
                Xn_1[i] = Vector3.zero;
                Xn[i] = Vector3.zero;
            }

        }
    }

    public class TransformStruct
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Scale;

        public TransformStruct(Vector3 p, Quaternion r, Vector3 s)
        {
            Position = p;
            Rotation = r;
            Scale = s;
        }
    }
}