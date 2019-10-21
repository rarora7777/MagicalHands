//////////////////////////////////////////////////////////////////////////////////////
/// Animatable is the base class for objects that can be manipulated using gestures.
/// Attach this script to the GameObjects that you want to animate.
//////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    [RequireComponent(typeof(Rigidbody), typeof(Collider))]
    public class Animatable : MonoBehaviour
    {
        Rigidbody rigidbody;
        Collider collider;

        public PoseManager poseManager { get; private set; }
        public TimeManager timeManager { get; private set; }
        public GestureManager gestureManager { get; private set; }

        public List<Tuple<int, Vector3>> TranslationKeys { get; private set; }  = new List<Tuple<int, Vector3>>();
        public List<Tuple<int, Quaternion>> RotationKeys { get; private set; } = new List<Tuple<int, Quaternion>>();
        public List<Tuple<int, Vector3>> ScaleKeys { get; private set; } = new List<Tuple<int, Vector3>>();

        public int StartFrame { get; set; } = 0;

        bool isSelected = false;

        Dictionary<int, Material> storedMaterials;

        public delegate void ObjectSelected();
        public event ObjectSelected OnSelect;
        public delegate void ObjectDeselected();
        public event ObjectDeselected OnDeselect;
        public delegate void Initialized();
        public event Initialized OnInit;
        public delegate void EmissionConeSetFromUndoStack(List<Vector3> curve, List<float> timeCache, List<Tuple<float, ParticleEmitter.EmissionData>> emData);
        public event EmissionConeSetFromUndoStack OnEmissionConeFromUndoStack;
        public delegate void EmissionNoiseSetFromUndoStack(float amp, float freq);
        public event EmissionNoiseSetFromUndoStack OnEmissionNoiseFromUndoStack;
        public delegate void EmissionSpiralSetFromUndoStack(float freq);
        public event EmissionSpiralSetFromUndoStack OnEmissionSpiralFromUndoStack;


        public List<Renderer> SelectUI = new List<Renderer>(5);
        public List<Renderer> ApproachUI = new List<Renderer>(5);

        public Renderer SizeProxy;

        int CurJointIntersectionCount = 0;

        public ShelfObject Creator { get; private set; }

        private void Awake()
        {
            if (SizeProxy == null)
                SizeProxy = GetComponent<Renderer>();

            // Size proxy must either be pre-assigned or there should be a Renderer component
            if (SizeProxy == null)
                throw new ArgumentNullException(name + ": Size proxy must be set manually or a Renderer component must be available!");
        }

        // Start is called before the first frame update
        void Start()
        {
            rigidbody = GetComponent<Rigidbody>();
            collider = GetComponent<Collider>();
            var avatar = GameObject.FindWithTag("Avatar");
            poseManager = avatar.GetComponent<PoseManager>();
            
            Debug.Assert(poseManager != null, "Cannot find avatar!");

            var camera = GameObject.FindWithTag("MainCamera");
            timeManager = camera.GetComponent<TimeManager>();
            gestureManager = camera.GetComponent<GestureManager>();

            Debug.Assert(timeManager != null, "Cannot find camera!");

            storedMaterials = new Dictionary<int, Material>();
            foreach(var child in GetComponentsInChildren<MeshRenderer>())
            {
                storedMaterials.Add(child.gameObject.GetInstanceID(), child.material);
            }
        }

        public void Init(ShelfObject parent)
        {
            Creator = parent;
            OnInit?.Invoke();
            TimeManager.RegisterObject(this);

            foreach (var mf in GetComponentsInChildren<MeshFilter>())
                mf.mesh = mf.sharedMesh;

            foreach (var child in GetComponentsInChildren<Animatable>())
            {
                if (child.GetInstanceID() == GetInstanceID())
                    continue;

                child.Init(null);
            }
        }

        private void OnDestroy()
        {
            TimeManager.DeregisterObject(this);
            PoseManager.DeleteReferencesToObject(this);
        }

        // Update is called once per frame
        void Update()
        {
            if (isSelected)
            {
                float lerp = Mathf.PingPong(Time.time, Globals.SELECTION_ANIMATION_DURATION) / Globals.SELECTION_ANIMATION_DURATION;
                var colour = Globals.SELECTION_MATERIAL.color;
                Globals.SELECTION_MATERIAL.color = new Color(colour.r, colour.g, colour.b, lerp);
            }
        }

        public void Selected()
        {
            if (TimeManager.SelectedObject != null)
                TimeManager.SelectedObject.Deselected();

            TimeManager.SelectedObject = this;

            OnSelect?.Invoke();

            isSelected = true;

            foreach (var elem in SelectUI)
                elem.enabled = true;

            foreach (var child in GetComponentsInChildren<MeshRenderer>())
            {
                child.material = Globals.SELECTION_MATERIAL;
            }
        }

        public void Deselected()
        {
            TimeManager.SelectedObject = null;
            OnDeselect?.Invoke();
            isSelected = false;
            try
            {
                foreach (var elem in SelectUI)
                    elem.enabled = false;

                foreach (var child in GetComponentsInChildren<MeshRenderer>())
                {
                    child.material = storedMaterials[child.gameObject.GetInstanceID()];
                }

                if (CurJointIntersectionCount > 0)
                    foreach (var elem in ApproachUI)
                        elem.enabled = true;

                // Reset alpha for selection material
                var colour = Globals.SELECTION_MATERIAL.color;
                Globals.SELECTION_MATERIAL.color = new Color(colour.r, colour.g, colour.b, 1.0f);
            }
            catch (KeyNotFoundException e)
            {
                Debug.LogWarning("Child object's material not found!");
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            PoseManager.OnHandObjectCollisionStart(this, other.gameObject);

            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);
            if (joint != PoseManager.HandJoint.IndexTip)
                return;

            foreach (var elem in ApproachUI)
                elem.enabled = true;

            CurJointIntersectionCount++;
        }

        // This should be useful: the colliders on fingers are triggers and trigger the OnTrigger*() events
        // while other rigidbodies (such as on Animatable objects) will trigger OnCollision*()
        private void OnCollisionEnter(Collision collision)
        {
            
        }

        private void OnTriggerStay(Collider other)
        {
            PoseManager.OnHandObjectCollisionStay(this, other.gameObject);
        }

        private void OnTriggerExit(Collider other)
        {
            PoseManager.OnHandObjectCollisionEnd(this, other.gameObject);

            PoseManager.GetJointFromGameobjectName(other.name, out OvrAvatar.HandType hand, out PoseManager.HandJoint joint);
            if (joint != PoseManager.HandJoint.IndexTip)
                return;

            CurJointIntersectionCount--;

            if (CurJointIntersectionCount == 0)
            {
                foreach (var elem in ApproachUI)
                    elem.enabled = false;

                if (isSelected)
                    foreach (var elem in SelectUI)
                        elem.enabled = true;
            }
        }

        public void EvaluateTransform(int frame)
        {
            frame = frame - StartFrame;
            float param = FindNearestKeys(TranslationKeys, frame, transform.localPosition, out Vector3 t1, out Vector3 t2);
            Vector3 t = param * t2 + (1 - param) * t1;

            param = FindNearestKeys(ScaleKeys, frame, transform.localScale, out Vector3 s1, out Vector3 s2);
            Vector3 s = param * s2 + (1 - param) * s1;

            param = FindNearestKeys(RotationKeys, frame, transform.localRotation, out Quaternion r1, out Quaternion r2);
            Quaternion r = Quaternion.Slerp(r1, r2, param);

            transform.localPosition = t;
            transform.localScale = s;
            transform.localRotation = r;
        }

        private float FindNearestKeys<T>(List<Tuple<int, T>> keys, int frame, T defaultValue, out T val1, out T val2)
        {
            float param = 0;
            val2 = defaultValue;
            val1 = defaultValue;
            int idx = -1;
            try
            {
                if (keys.Count == 0)
                    val1 = defaultValue;
                else if (keys.Count == 1)
                    val1 = keys[0].Item2;
                else if (frame <= keys[0].Item1)
                    val1 = keys[0].Item2;
                else if (frame >= keys[keys.Count - 1].Item1)
                    val1 = keys[keys.Count - 1].Item2;
                else
                {
                    idx = FindClosestIndex(keys, frame);
                    if (keys[idx].Item1 > frame)
                        idx--;

                    param = (frame - keys[idx].Item1) / (float)(keys[idx + 1].Item1 - keys[idx].Item1);
                    val1 = keys[idx].Item2;
                    val2 = keys[idx + 1].Item2;
                }   
            }
            catch (ArgumentOutOfRangeException e)
            {
                Debug.Log(nameof(this.FindNearestKeys));
                Debug.Log(e.Message + " " + idx.ToString() + " " + keys.Count);
            }

            return Mathf.SmoothStep(0, 1, param);
        }

        public int FindClosestIndex<T>(List<Tuple<int, T>> list, int value)
        {

            if (value < list[0].Item1)
            {
                return 0;
            }
            if (value > list[list.Count - 1].Item1)
            {
                return list.Count - 1;
            }

            int lo = 0;
            int hi = list.Count - 1;

            while (lo <= hi)
            {
                int mid = (hi + lo) / 2;

                if (value < list[mid].Item1)
                {
                    hi = mid - 1;
                }
                else if (value > list[mid].Item1)
                {
                    lo = mid + 1;
                }
                else
                {
                    return mid;
                }
            }
            // lo == hi + 1
            return (list[lo].Item1 - value) < (value - list[hi].Item1) ? lo : hi;
        }

        public bool AddTranslationKey(Vector3 t, int frame)
        {
            return AddKey(TranslationKeys, t, frame);
        }

        public void SetTranslationKeys(List<Tuple<int, Vector3>> keys)
        {
            TranslationKeys = keys;
            EvaluateTransform(TimeManager.CurFrame);
            Debug.Log(name + " #keys = " + TranslationKeys.Count + 
                " Last key: " + TranslationKeys[TranslationKeys.Count - 1]);
        }

        public bool AddRotationKey(Quaternion r, int frame)
        {
            return AddKey(RotationKeys, r, frame);
        }

        public void SetRotationKeys(List<Tuple<int, Quaternion>> keys)
        {
            RotationKeys = keys;
            EvaluateTransform(TimeManager.CurFrame);
        }

        public bool AddScaleKey(Vector3 s, int frame)
        {
            return AddKey(ScaleKeys, s, frame);
        }

        public void SetScaleKeys(List<Tuple<int, Vector3>> keys)
        {
            ScaleKeys = keys;
            EvaluateTransform(TimeManager.CurFrame);
        }

        private bool AddKey<T>(List<Tuple<int, T>> keys, T key, int frame)
        {
            frame = frame - StartFrame;

            int idx = -1;
            try
            {
                if (keys.Count == 0)
                {
                    keys.Add(new Tuple<int, T>(frame, key));
                    return true;
                }
                idx = FindClosestIndex(keys, frame);
                if (keys[idx].Item1 > frame)
                    idx--;
                if (idx >= 0 && keys[idx].Item1 == frame)
                {
                    keys.RemoveAt(idx);
                    keys.Insert(idx, new Tuple<int, T>(frame, key));
                }
                else
                {
                    keys.Insert(idx + 1, new Tuple<int, T>(frame, key));
                }
                return true;
            }
            catch (ArgumentOutOfRangeException e)
            {
                Debug.Log(nameof(this.AddKey));
                Debug.Log(e.Message + " " + idx.ToString() + " " + keys.Count);
                return false;
            }
        }

        public void TrySetEmissionCone(Tuple<List<Vector3>, List<float>, List<Tuple<float, ParticleEmitter.EmissionData>>> cone)
        {
            OnEmissionConeFromUndoStack?.Invoke(cone.Item1, cone.Item2, cone.Item3);
        }

        public void TrySetEmissionNoise(Tuple<float, float> noise)
        {
            OnEmissionNoiseFromUndoStack?.Invoke(noise.Item1, noise.Item2);
        }

        public void TrySetEmissionSpiral(float spiralFrequency)
        {
            OnEmissionSpiralFromUndoStack?.Invoke(spiralFrequency);
        }
    }
}