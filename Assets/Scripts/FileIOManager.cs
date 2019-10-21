//////////////////////////////////////////////////////////////////////////////////////
/// FileIOManager is a pseudo-static class. That is, while it is not a syntactically
/// static class, it should be treated as a semantically static class.
/// FileIOManager handles serialization and deserialization of data structures, and
/// File I/O for saving and loading MagicalHands scenes.
//////////////////////////////////////////////////////////////////////////////////////

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System;
using System.IO;
using System.Text.RegularExpressions;

namespace GestureAnim
{
    // Serializable versions of Vector3 and Quaternion from
    // https://answers.unity.com/questions/956047/serialize-quaternion-or-vector3.html
    [Serializable]
    public struct SerializableVector3
    {
        /// <summary>
        /// x component
        /// </summary>
        public float x;

        /// <summary>
        /// y component
        /// </summary>
        public float y;

        /// <summary>
        /// z component
        /// </summary>
        public float z;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="rX"></param>
        /// <param name="rY"></param>
        /// <param name="rZ"></param>
        public SerializableVector3(float rX, float rY, float rZ)
        {
            x = rX;
            y = rY;
            z = rZ;
        }

        /// <summary>
        /// Returns a string representation of the object
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return String.Format("[{0}, {1}, {2}]", x, y, z);
        }

        /// <summary>
        /// Automatic conversion from SerializableVector3 to Vector3
        /// </summary>
        /// <param name="rValue"></param>
        /// <returns></returns>
        public static implicit operator Vector3(SerializableVector3 rValue)
        {
            return new Vector3(rValue.x, rValue.y, rValue.z);
        }

        /// <summary>
        /// Automatic conversion from Vector3 to SerializableVector3
        /// </summary>
        /// <param name="rValue"></param>
        /// <returns></returns>
        public static implicit operator SerializableVector3(Vector3 rValue)
        {
            return new SerializableVector3(rValue.x, rValue.y, rValue.z);
        }
    }

    [Serializable]
    public struct SerializableQuaternion
    {
        /// <summary>
        /// x component
        /// </summary>
        public float x;

        /// <summary>
        /// y component
        /// </summary>
        public float y;

        /// <summary>
        /// z component
        /// </summary>
        public float z;

        /// <summary>
        /// w component
        /// </summary>
        public float w;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="rX"></param>
        /// <param name="rY"></param>
        /// <param name="rZ"></param>
        /// <param name="rW"></param>
        public SerializableQuaternion(float rX, float rY, float rZ, float rW)
        {
            x = rX;
            y = rY;
            z = rZ;
            w = rW;
        }

        /// <summary>
        /// Returns a string representation of the object
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return String.Format("[{0}, {1}, {2}, {3}]", x, y, z, w);
        }

        /// <summary>
        /// Automatic conversion from SerializableQuaternion to Quaternion
        /// </summary>
        /// <param name="rValue"></param>
        /// <returns></returns>
        public static implicit operator Quaternion(SerializableQuaternion rValue)
        {
            return new Quaternion(rValue.x, rValue.y, rValue.z, rValue.w);
        }

        /// <summary>
        /// Automatic conversion from Quaternion to SerializableQuaternion
        /// </summary>
        /// <param name="rValue"></param>
        /// <returns></returns>
        public static implicit operator SerializableQuaternion(Quaternion rValue)
        {
            return new SerializableQuaternion(rValue.x, rValue.y, rValue.z, rValue.w);
        }
    }


    [Serializable]
    public struct SerializableScene
    {
        public int NumFrame;

        public List<SerializableAnimatable> Objects;

        public SerializableScene(int nF, Animatable[] sceneObjects)
        {
            NumFrame = nF;
            Objects = new List<SerializableAnimatable>(sceneObjects.Length);

            foreach (var sceneObj in sceneObjects)
            {
                // only get top-level objects, the others will be saved in the 
                // Children lists via calls to `new SerializableAnimatable(Animatable)`
                if (sceneObj.GetComponentsInParent<Animatable>().Length != 1)
                    continue;

                var obj = new SerializableAnimatable(sceneObj, true);

                Objects.Add(obj);
            }
        }
    }

    [Serializable]
    public struct SerializableAnimatable
    {
        public string ShelfParent;

        public int StartFrame;

        public List<Tuple<int, SerializableVector3>> TranslationKeys;
        public List<Tuple<int, SerializableQuaternion>> RotationKeys;
        public List<Tuple<int, SerializableVector3>> ScaleKeys;

        public List<SerializableAnimatable> Children;
        public SerializableEmitter Emitter;

        public bool IsTopLevel;

        public SerializableAnimatable(Animatable sceneObj, bool top = false)
        {
            IsTopLevel = top;

            if (sceneObj.Creator != null)
                ShelfParent = sceneObj.Creator.name;
            else
                ShelfParent = "";

            StartFrame = sceneObj.StartFrame;

            TranslationKeys = new List<Tuple<int, SerializableVector3>>(sceneObj.TranslationKeys.Count);
            ScaleKeys = new List<Tuple<int, SerializableVector3>>(sceneObj.ScaleKeys.Count);
            RotationKeys = new List<Tuple<int, SerializableQuaternion>>(sceneObj.RotationKeys.Count);

            foreach (var t in sceneObj.TranslationKeys)
                TranslationKeys.Add(new Tuple<int, SerializableVector3>(t.Item1, t.Item2));
            foreach (var t in sceneObj.ScaleKeys)
                ScaleKeys.Add(new Tuple<int, SerializableVector3>(t.Item1, t.Item2));
            foreach (var t in sceneObj.RotationKeys)
                RotationKeys.Add(new Tuple<int, SerializableQuaternion>(t.Item1, t.Item2));

            Children = new List<SerializableAnimatable>();
            // serialize all immediate children
            foreach (var child in sceneObj.GetComponentsInChildren<Animatable>())
            {
                // make sure to skip the current object
                if (child.GetInstanceID() == sceneObj.GetInstanceID())
                    continue;

                Children.Add(new SerializableAnimatable(child));
            }

            // Assumption: Only top-level Animatable objects can have an emitter
            var sceneEmitter = sceneObj.GetComponentInChildren<ParticleEmitter>();

            if (top == false || sceneEmitter == null)
                Emitter = new SerializableEmitter();
            else
                Emitter = new SerializableEmitter(sceneEmitter);
        }
    };

    [Serializable]
    public struct SerializableEmitter
    {
        public List<SerializableVector3> CurveControlPoints;

        public List<float> TimeCache;
        
        public List<Tuple<float, ParticleEmitter.EmissionData>> EmissionDataKeys;

        public float GravityInfluence;
        public float WindInfluence;

        public float MaxPositionNoise;
        public float MaxRotationNoise;

        public float PositionNoiseFrequency;
        public float RotationNoiseFrequency;

        public float SpiralFrequency;

        public SerializableQuaternion InitialParticleRotation;
        public SerializableVector3 ParticleScale;

        public string ParticleShelfObject;

        public SerializableEmitter(ParticleEmitter sceneEmitter)
        {
            var ctrlPts = sceneEmitter.GetEmissionCurveControlPoints();
            CurveControlPoints = new List<SerializableVector3>(ctrlPts.Length);
            foreach (var pt in ctrlPts)
                CurveControlPoints.Add(pt);

            TimeCache = new List<float> (sceneEmitter.GetParamToTimeCache());

            EmissionDataKeys = sceneEmitter.GetEmissionDataKeys();

            GravityInfluence = sceneEmitter.GravityInfluence;
            WindInfluence = sceneEmitter.WindInfluence;

            MaxPositionNoise = sceneEmitter.MaxPositionNoise;
            MaxRotationNoise = sceneEmitter.MaxRotationNoise;

            PositionNoiseFrequency = sceneEmitter.PositionNoiseFrequency;
            RotationNoiseFrequency = sceneEmitter.RotationNoiseFrequency;
            SpiralFrequency = sceneEmitter.SpiralFrequency;

            InitialParticleRotation = sceneEmitter.InitialParticleRotation;
            ParticleScale = sceneEmitter.ParticleScale;

            if (sceneEmitter.ParticleMeshCreator == null)
                ParticleShelfObject = "";
            else
                ParticleShelfObject = sceneEmitter.ParticleMeshCreator.name;
        }
    }


    public class FileIOManager : MonoBehaviour
    {
        private static bool tryLoadingFile = false;
        private static string sceneFileName = "";

        private static bool saveLocationSpecified = false;
        private static string saveLocation = "";

        public string LoadFile;

        private void Start()
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; ++i)
                if (args[i] == "--File" || args[i] == "--file")
                {
                    tryLoadingFile = true;
                    sceneFileName = args[i + 1];
                }
                else if (args[i] == "--SaveFolder" || args[i] == "--savefolder")
                {
                    saveLocationSpecified = true;
                    saveLocation = args[i + 1];
                }

            // If save location is not specified on the command line,
            // then look for text asset
            if (!saveLocationSpecified)
            {
                string path = Path.Combine(Application.streamingAssetsPath, "save_folder.txt");

                if (File.Exists(path))
                {
                    var reader = new StreamReader(path);
                    saveLocation = reader.ReadLine();
                    saveLocationSpecified = true;
                }
            }

#if UNITY_EDITOR
            if (LoadFile.Length > 0)
            {
                tryLoadingFile = true;
                sceneFileName = LoadFile;
            }
#endif
        }

        // Update is called once per frame
        void Update()
        {
            // Putting it here makes sure that all the Start() routines have been called
            if (tryLoadingFile)
            {
                tryLoadingFile = false;
                enabled = false;
                TryLoadScene();
            }
        }

        public static void TryLoadSceneFromFile(string filename)
        {
            sceneFileName = filename;
            TryLoadScene();
        }

        private static void TryLoadScene()
        {
            // start by deleting current objects
            TimeManager.SelectedObject?.Deselected();
            foreach (var obj in FindObjectsOfType<Animatable>())
            {
                GestureManager.ProcessObjectDeletion(obj);
                Destroy(obj.gameObject);
            }

            // let's try loading the scene now
            try
            {
                string data = System.IO.File.ReadAllText(sceneFileName);
                var scene = JsonConvert.DeserializeObject<SerializableScene>(data);

                TimeManager.SetFrameCount(scene.NumFrame);

                var shelfObjects = FindObjectsOfType<ShelfObject>();
                Dictionary<string, ShelfObject> shelfObjMap = new Dictionary<string, ShelfObject>(shelfObjects.Length);

                foreach (var obj in shelfObjects)
                    shelfObjMap.Add(obj.name, obj);

                foreach(var obj in scene.Objects)
                    InstantiateFromSaved(obj, shelfObjMap);

                TimeManager.PlayAnimationInScript();
            }
            catch(Exception e)
            {
                Debug.LogWarning(e.Message);
                Debug.Log(e.StackTrace);
            }
        }

        private static void InstantiateFromSaved(SerializableAnimatable obj, Dictionary<string, ShelfObject> shelfObjMap)
        {
            bool isShelfObjectAvailable = false;
            ShelfObject shelfObj = null;
            
            isShelfObjectAvailable = shelfObjMap.TryGetValue(obj.ShelfParent, out shelfObj);

            if (!isShelfObjectAvailable)
            {
                Debug.LogWarning("File load warning: Could not find a shelfobject named " + obj.ShelfParent);
                return;
            }
            
            var sceneObject = shelfObj.InstantiateAnimatable();
            SetPropertiesFromJsonObject(sceneObject, obj);

            // Emitters are only supported on top-level objects
            var emitter = obj.Emitter;

            // Assuming that for no "Emitter" in the JSON, its default value 
            // is created, and so a list subfield should be empty
            if (emitter.CurveControlPoints != null &&
                emitter.CurveControlPoints.Count > 0)
            {
                var sceneEmitter = sceneObject.GetComponentInChildren<ParticleEmitter>();
                if (sceneEmitter == null)
                {
                    Debug.LogWarning(
                        "JSON indicated the presence of a particle emitter for " +
                        shelfObj.name + 
                        " but no emitter was found on the object!");
                }

                var ctrlPoints = new List<Vector3>(emitter.CurveControlPoints.Count);
                foreach (var pt in emitter.CurveControlPoints)
                    ctrlPoints.Add(pt);

                sceneEmitter.SetEmissionConeFromFile(
                    ctrlPoints,
                    emitter.TimeCache,
                    emitter.EmissionDataKeys);

                sceneEmitter.GravityInfluence = emitter.GravityInfluence;
                sceneEmitter.WindInfluence = emitter.WindInfluence;

                sceneEmitter.MaxPositionNoise = emitter.MaxPositionNoise;
                sceneEmitter.MaxRotationNoise = emitter.MaxRotationNoise;

                sceneEmitter.PositionNoiseFrequency = emitter.PositionNoiseFrequency;
                sceneEmitter.RotationNoiseFrequency = emitter.RotationNoiseFrequency;

                sceneEmitter.SpiralFrequency = emitter.SpiralFrequency;

                sceneEmitter.InitialParticleRotation = emitter.InitialParticleRotation;
                sceneEmitter.ParticleScale = emitter.ParticleScale;

                if (emitter.ParticleShelfObject != null)
                {
                    bool res = shelfObjMap.TryGetValue(emitter.ParticleShelfObject, out ShelfObject particleMeshParent);
                    if (res)
                        sceneEmitter.TryChangeParticleShapeFromDroppedObject(particleMeshParent);
                }

                sceneEmitter.TestMode = false;
            }

            sceneObject.Init(shelfObj);
        }

        private static void SetPropertiesFromJsonObject(Animatable sceneObject, SerializableAnimatable obj)
        {
            TimeManager.RegisterObject(sceneObject, false);
            sceneObject.StartFrame = obj.StartFrame;

            foreach (var pair in obj.TranslationKeys)
                sceneObject.AddTranslationKey(pair.Item2, pair.Item1 + obj.StartFrame);
            foreach (var pair in obj.RotationKeys)
                sceneObject.AddRotationKey(pair.Item2, pair.Item1 + obj.StartFrame);
            foreach (var pair in obj.ScaleKeys)
                sceneObject.AddScaleKey(pair.Item2, pair.Item1 + obj.StartFrame);

            sceneObject.EvaluateTransform(0);

            // recurse for top-level folks
            if (obj.IsTopLevel)
            {
                var sceneChildren = sceneObject.GetComponentsInChildren<Animatable>();
                Debug.Assert(sceneChildren.Length - 1 == obj.Children.Count,
                    "#children in saved file differs from loaded object! " + 
                    sceneChildren.Length + " != " + obj.Children.Count);

                // Since GetComponentsInChildren<T> uses depth-first search, the first 
                // component must be `obj` itself, which we skip
                for (int i = 1; i < sceneChildren.Length; ++i)
                {
                    SetPropertiesFromJsonObject(sceneChildren[i], obj.Children[i - 1]);
                }
            }
        }

        public static bool TrySaveCurrentScene()
        {
            try
            {
                var sceneObjects = FindObjectsOfType<Animatable>();

                var scene = new SerializableScene(TimeManager.NumFrame, sceneObjects);

                var id = Regex.Replace(Convert.ToBase64String(Guid.NewGuid().ToByteArray()), "[/+=]", "");

                string path;
                if (saveLocationSpecified)
                    path = saveLocation;
                else
                    path = Application.dataPath;

                path = Path.Combine(path, id.ToString() + ".json");

                File.WriteAllText(path, JsonConvert.SerializeObject(scene));

                Debug.Log("Saved to " + path);

                return true;
            }
            catch (Exception e)
            {
                Debug.LogWarning(e.Message);
                return false;
            }
        }
    }
}