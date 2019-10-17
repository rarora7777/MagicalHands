using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using burningmime.curves;
using System.Linq;
using System;
using System.Timers;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.IntegralTransforms;

namespace GestureAnim
{
    [RequireComponent(typeof(ParticleSystem), typeof(Animatable))]
    public class ParticleEmitter : MonoBehaviour
    {
        [Serializable]
        public struct EmissionData
        {
            public float Radius;
            
            public EmissionData(float r)
            {
                Radius = r;
            }

            public static EmissionData Lerp(EmissionData a, EmissionData b, float alpha)
            {
                var r = Mathf.Lerp(a.Radius, b.Radius, alpha);

                return new EmissionData(r);
            }
        };

        readonly BezierFit emissionCurve = new BezierFit();
        
        const int maxNumParticle = Globals.MAX_PARTICLES;
        Dictionary<int, ParticleCustomData> cdata = new Dictionary<int, ParticleCustomData>(maxNumParticle * Globals.PARTICLE_DEFAULT_EMISSION_RATE);

        int uniqueID = 0;
        Timer cdataGCTimer;
        int debugID = 1;

        ParticleSystem ps;
        ParticleSystem.MainModule main;
        ParticleSystem.EmissionModule emission;
        ParticleSystem.ShapeModule emitterShape;

        public Gradient colourGradient;

        List<Tuple<float, EmissionData>> emissionDataKeys;
        List<Vector4> customData = new List<Vector4>();

        float dt;
        float dt2;

        public bool TestMode;

        private float MaxParticleLifetime = 0;

        public LineRenderer RenderedCurve;
        public MeshFilter RenderedConeMeshFilter;
        private EmissionCone emissionCone;

        private List<ParticleCustomForceField> forceFields = new List<ParticleCustomForceField>();

        public float GravityInfluence = 0;
        public float WindInfluence = 0;

        public float MaxPositionNoise = 0.1f;
        public float MaxRotationNoise = 30;

        public float PositionNoiseFrequency = 1;
        public float RotationNoiseFrequency = 1;

        public float SpiralFrequency = 0;

        private Animatable animScript;

        public bool EmissionConeDrawingInProgress { get; private set; } = false;
        DateTime coneDrawingStartTime;

        public bool EnableEmissionConeGestures = false;
        public bool ConeGestureInProgress { get; private set; } = false;

        // Temporary variables to store emission cone drawing data
        private List<Vector3> leftPos, rightPos;
        private List<float> times;

        // Temporary variables to store cone gesture data
        private List<Vector3> coneGesturePositionData, coneGestureVelocityData, coneGestureAccelerationData;
        private List<double> coneGestureTimeData;
        private DateTime coneGestureStartTime;

        // When dropping an Animatable onto the emitter, should set these two
        // from localRotation and localScale
        public Quaternion InitialParticleRotation;
        public Vector3 ParticleScale;

        public ShelfObject ParticleMeshCreator = null;

        //int m_frameCounter = 0;
        //float m_timeCounter = 0.0f;
        //float m_lastFramerate = 0.0f;
        //public float m_refreshTime = 0.5f;

        void Awake()
        {
            animScript = GetComponent<Animatable>();
            animScript.OnInit += Init;
            animScript.OnEmissionConeFromUndoStack += SetEmissionConeFromFile;

            animScript.OnEmissionNoiseFromUndoStack +=
                (amplitude, frequency) => {
                    MaxPositionNoise = amplitude;
                    PositionNoiseFrequency = frequency;

                    MaxRotationNoise = MaxPositionNoise * 360;
                    RotationNoiseFrequency = PositionNoiseFrequency;
                };

            animScript.OnEmissionSpiralFromUndoStack += 
                (frequency) => {
                    SpiralFrequency = frequency;
                };
            
            //RenderedCurve = GetComponent<LineRenderer>();
            RenderedCurve.enabled = false;
            RenderedCurve.widthMultiplier = Globals.EMISSION_CURVE_RENDER_WIDTH;

            animScript.SelectUI.Add(RenderedCurve);
            animScript.SelectUI.Add(RenderedConeMeshFilter.GetComponent<MeshRenderer>());
            emissionCone = RenderedConeMeshFilter.GetComponent<EmissionCone>();
            emissionCone.GetComponent<Rigidbody>().detectCollisions = false;

            foreach (var child in GetComponentsInChildren<MeshRenderer>())
            {
                child.enabled = false;
                
                animScript.SelectUI.Add(child);
                if (child.GetInstanceID() != RenderedConeMeshFilter.GetComponent<MeshRenderer>().GetInstanceID())
                    animScript.ApproachUI.Add(child);
            }

            animScript.OnSelect += ProcessObjectSelection;
            animScript.OnDeselect += ProcessObjectDeselection;

            InitialParticleRotation = Quaternion.identity; // AngleAxis(90, Vector3.right);
            ParticleScale = new Vector3(0.02f, 0.02f, 0.02f);

            ps = GetComponent<ParticleSystem>();
            ps.Stop();

            main = ps.main;
            main.simulationSpace = ParticleSystemSimulationSpace.Local;
            main.maxParticles = maxNumParticle;
            main.startSpeed = Globals.PARTICLE_DEFAULT_START_SPEED;
            main.startLifetime = 20;
            main.startSize3D = true;
            main.startSizeX = ParticleScale.x;
            main.startSizeY = ParticleScale.y;
            main.startSizeZ = ParticleScale.z;

            emission = ps.emission;
            emission.enabled = true;
            //emission.rateOverTime = new ParticleSystem.MinMaxCurve(Globals.PARTICLE_DEFAULT_EMISSION_RATE);

            emitterShape = ps.shape;
            emitterShape.enabled = true;
            emitterShape.shapeType = ParticleSystemShapeType.Cone;
            emitterShape.radius = Globals.PARTICLE_EMITTER_DEFAULT_SIZE;
            emitterShape.angle = 0;

            var colourModule = ps.colorOverLifetime;
            colourModule.color = new ParticleSystem.MinMaxGradient(colourGradient);
            colourModule.enabled = true;

            ParticleCustomForceField gravity =
                new ParticleCustomForceField(
                    ParticleCustomForceField.FieldEffectType.Gravity,
                    x => { return 9.83f * Vector3.down; },
                    x => { return Vector3.zero; },
                    1
                );

            forceFields.Add(gravity);
        }

        private void Init()
        {
            if (TestMode)
            {
                RunTest();
            }
            //else
            //{
            //    // This doesn't work properly, but I can get back to this later. Using the more 
            //    // complex model created by RunTest() for now.
            //    CreateDefaultEmission();
            //}
        }

        private void OnDestroy()
        {
            if(cdataGCTimer != null)
            {
                cdataGCTimer.Stop();
                cdataGCTimer.Dispose();
            }
        }

        private void Update()
        {
            if (EmissionConeDrawingInProgress)
            {
                rightPos.Add(animScript.poseManager.GetHandTransform(OvrAvatar.HandType.Right, Globals.EMISSION_GESTURE_JOINT).position);
                leftPos.Add(animScript.poseManager.GetHandTransform(OvrAvatar.HandType.Left, Globals.EMISSION_GESTURE_JOINT).position);
                times.Add((float)(DateTime.Now - coneDrawingStartTime).TotalSeconds);

                if (times.Count % 10 == 0)
                {
                    AddPointToEmissionUI(transform.InverseTransformPoint(leftPos.Last()), transform.InverseTransformPoint(rightPos.Last()));
                }
            }
            else if (ConeGestureInProgress)
            {
                var pos = animScript.poseManager.GetHandTransform(OvrAvatar.HandType.Right, PoseManager.HandJoint.IndexTip).position;
                var vel = PoseManager.GetHandVelocity(OvrAvatar.HandType.Right);
                var acc = PoseManager.GetHandAcceleration(OvrAvatar.HandType.Right);

                coneGesturePositionData.Add(transform.InverseTransformPoint(pos));
                coneGestureVelocityData.Add(transform.InverseTransformVector(vel));
                coneGestureAccelerationData.Add(transform.InverseTransformVector(acc));
                coneGestureTimeData.Add((DateTime.Now - coneGestureStartTime).TotalSeconds);

                GetComponent<LineRenderer>().positionCount++;
                GetComponent<LineRenderer>().SetPosition(
                    GetComponent<LineRenderer>().positionCount - 1,
                    transform.InverseTransformPoint(pos));
            }
        }


        public void EmissionConeDrawingStarted()
        {
            leftPos = new List<Vector3>(100);
            rightPos = new List<Vector3>(100);
            times = new List<float>(100);

            coneDrawingStartTime = DateTime.Now;
            EmissionConeDrawingInProgress = true;
            RenderedCurve.positionCount = 0;
            RenderedConeMeshFilter.sharedMesh = null;
            ps.Pause();
            emissionCone.GetComponent<Rigidbody>().detectCollisions = false;
            emissionCone.ResetIntersectionCounter();
        }

        public void EmissionConeDrawingFinished()
        {
            RenderedConeMeshFilter.sharedMesh = null;
            if (leftPos.Count > 1)
            {
                TimeManager.StackEmissionCone(Tuple.Create(
                    emissionCurve.ControlPoints.ToList(),
                    emissionCurve.GetFlattenedCache(BezierFit.ParameterType.Time).ToList(),
                    emissionDataKeys),
                animScript
                );

                NewEmissionCone(leftPos, rightPos, times);
            }
            EmissionConeDrawingInProgress = false;
            animScript.Deselected();
        }

        public void ConeGestureStarted()
        {
            coneGesturePositionData = new List<Vector3>(100);
            coneGestureVelocityData = new List<Vector3>(100);
            coneGestureAccelerationData = new List<Vector3>(100);
            coneGestureTimeData = new List<double>(100);

            ConeGestureInProgress = true;
            coneGestureStartTime = DateTime.Now;

            if (GetComponent<LineRenderer>() == null)
                gameObject.AddComponent<LineRenderer>();

            GetComponent<LineRenderer>().widthMultiplier = 0.0025f;
            GetComponent<LineRenderer>().colorGradient.SetKeys(
                new GradientColorKey[] { new GradientColorKey(Color.red, 0.0f) },
                new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f) });
            GetComponent<LineRenderer>().useWorldSpace = false;
            GetComponent<LineRenderer>().positionCount = 0;
            GetComponent<LineRenderer>().enabled = true;
            GetComponent<LineRenderer>().material = Resources.Load<Material>("ConeGestures");
        }

        public void ConeGestureFinished()
        {
            ConeGestureInProgress = false;

            GetComponent<LineRenderer>().enabled = false;

            // Process cone gesture data here
            // Start by converting (local) XYZ to TNB coordinates w.r.t the curve
            var startIdx = coneGesturePositionData.Count / 10;
            var endIdx = coneGesturePositionData.Count * 9 / 10;
            var n = endIdx - startIdx;

            double[] posData = new double[3 * n];
            double[] velData = new double[3 * n];
            double[] accData = new double[3 * n];
            //double[] tData = new double[3 * n];
            //double[] nData = new double[3 * n];
            //double[] bData = new double[3 * n];
            //double[] timeData = new double[n];

            for (int i=startIdx; i<endIdx; ++i)
            {
                var pos = coneGesturePositionData[i];
                var vel = coneGestureVelocityData[i];
                var acc = coneGestureAccelerationData[i];
                emissionCurve.GetClosestPoint(pos, out Vector3 closest, out int bezierIdx, out float param);
                var tangent = emissionCurve.Tangent(bezierIdx, param);
                var normal = emissionCurve.Normal(bezierIdx, param);
                var binormal = Vector3.Cross(tangent, normal);
                var delta = (pos - closest);
                var time = emissionCurve.GetTimeFromSplineParam(bezierIdx, param);

                // For points, first coordinate is time, and the other two given by dot product
                pos = new Vector3(time, Vector3.Dot(delta, normal), Vector3.Dot(delta, binormal));

                // For vectors, all three coordinates given by projecting to the new frame (dot product)
                vel = new Vector3(Vector3.Dot(vel, tangent), Vector3.Dot(vel, normal), Vector3.Dot(vel, binormal));
                acc = new Vector3(Vector3.Dot(acc, tangent), Vector3.Dot(acc, normal), Vector3.Dot(acc, binormal));

                for (var j=0; j<3; ++j)
                {
                    var idx = n * j + i - startIdx;
                    posData[idx] = pos[j];
                    velData[idx] = vel[j];
                    accData[idx] = acc[j];

                    //tData[idx] = tangent[j];
                    //nData[idx] = normal[j];
                    //bData[idx] = binormal[j];
                }

                //timeData[i - startIdx] = coneGestureTimeData[i];
            }

            // Perform SVD to get principal components
            Matrix<double> P = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, posData);
            Matrix<double> V = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, velData);
            Matrix<double> A = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, accData);

            //Matrix<double> T = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, tData);
            //Matrix<double> N = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, nData);
            //Matrix<double> B = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(n, 3, bData);
            //Vector<double> t = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(timeData);

            //var velSvd = V.Svd(true);
            var accSvd = A.Svd(true);

            //var SV = velSvd.S;
            //var VV = velSvd.VT;
            var SA = accSvd.S;
            var VA = accSvd.VT;

            //Debug.Log("Acc. singular values: " + SA[0].ToString("F3") + " " + SA[1].ToString("F3") + " " + SA[2].ToString("F3"));
            //Debug.Log("Acc. rotation matrix: " + VA);

            if (SA[0] / SA[1] > Globals.CONE_GESTURE_CLASSIFICATION_SINGULAR_VALUE_RATIO)
                ProcessEmissionNoiseGesture(P, VA);
            else
                ProcessSpiralForceGesture(P, V);

            Debug.Log("Delsecting because cone gesture finished");
            animScript.Deselected();
        }

        void ProcessEmissionNoiseGesture(Matrix<double> P, Matrix<double> Rot)
        {
            var series = P.Multiply(Rot.Row(0));
            
            var n = series.Count;
            double[] fft = new double[n + 1 + ((n % 2 == 0) ? 1 : 0)];
            series.ToArray().CopyTo(fft, 0);

            Fourier.ForwardReal(fft, n, FourierOptions.Matlab);

            int maxFreq = 0;
            double maxAmp = 0.0f;
            for(int i=1; i < n; ++i)
                if (Math.Abs(fft[i]) > maxAmp)
                {
                    maxAmp = Math.Abs(fft[i]);
                    maxFreq = i;
                }

            float timePeriod = (n / (float)maxFreq) * Globals.OCULUS_TRACKING_UPDATE_TIME;

            TimeManager.StackEmissionNoise(Tuple.Create(MaxPositionNoise, PositionNoiseFrequency), animScript);

            MaxPositionNoise = (float)maxAmp / n;
            PositionNoiseFrequency = 1 / timePeriod;

            MaxRotationNoise = MaxPositionNoise * 360;
            RotationNoiseFrequency = 1 / timePeriod;

            //Debug.Log(string.Join(" ", fft.Select(i=>i.ToString("F2")).ToArray()));
            Debug.Log("Noise freq: " + PositionNoiseFrequency + " Amplitude: " + maxAmp/n);
        }

        void ProcessSpiralForceGesture(Matrix<double> P, Matrix<double> V)
        {
            V.SetColumn(0, new double[V.RowCount]);
            var avgSpeed = MathNet.Numerics.Statistics.Statistics.Mean(V.RowNorms(2.0));

            P.SetColumn(0, V.Column(2));
            

            double[] angles = new double[P.RowCount];

            Func<Vector<double>, Vector<double>, int> CrossXSign = (a, b) => {
                return Math.Sign(a[1]*b[2] - b[1]*a[2]);
            };

            List<int> crossData = new List<int>(P.RowCount);
            for (var i = 0; i < P.RowCount; ++i)
                crossData.Add(CrossXSign(P.Row(i), V.Row(i)));

            // module by avg. direction of P x V
            TimeManager.StackEmissionSpiral(SpiralFrequency, animScript);

            SpiralFrequency = Math.Sign(crossData.Sum()) * (float)(avgSpeed / MathNet.Numerics.Statistics.Statistics.Mean(P.RowNorms(2.0)));

            Debug.Log("Spiral freq: " + SpiralFrequency + "P x V sign: " + (float)crossData.Sum()/P.RowCount);

            //int[] maxFreq = new int[2];
            //double[] maxAmp = new double[2];

            //var n = P.RowCount;

            //for (int r = 0; r < 2; ++r)
            //{
            //    var series = P.Multiply(Rot.Row(r));

            //    double[] fft = new double[n + 1 + ((n % 2 == 0) ? 1 : 0)];
            //    series.ToArray().CopyTo(fft, 0);

            //    Fourier.ForwardReal(fft, n, FourierOptions.Matlab);

            //    maxFreq[r] = 0;
            //    maxAmp[r] = 0.0f;
            //    for (int i = 1; i < n; ++i)
            //        if (Math.Abs(fft[i]) > maxAmp[r])
            //        {
            //            maxAmp[r] = Math.Abs(fft[i]);
            //            maxFreq[r] = i;
            //        }
            //}

            //float timePeriod = (2 * n / maxFreq.Sum()) * Globals.OCULUS_TRACKING_UPDATE_TIME;
            //SpiralFrequency = 1 / timePeriod;
            //Debug.Log("Spiral freq: " + SpiralFrequency + " Amplitude: " + maxAmp.Sum() / (2 * n));
            //return;


            //Matrix<double> series = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(P.RowCount, 2);
            //series.SetColumn(0, P.Multiply(Rot.Row(0)));
            //series.SetColumn(1, P.Multiply(Rot.Row(1)));

            //var n = series.RowCount;

            //var fft = (from i in Enumerable.Range(0, n)
            //        select new MathNet.Numerics.Complex32((float)series[i, 0], (float)series[i, 1])).ToArray();

            //Fourier.Forward(fft, FourierOptions.NoScaling);

            //int maxFreq = 0;
            //double maxAmp = 0.0f;
            //for (int i = 1; i < n; ++i)
            //    if (fft[i].MagnitudeSquared > maxAmp)
            //    {
            //        maxAmp = fft[i].MagnitudeSquared;
            //        maxFreq = i;
            //    }

            //float timePeriod = (n / (float)maxFreq) * Globals.OCULUS_TRACKING_UPDATE_TIME;

            //MaxPositionNoise = (float)Math.Sqrt(maxAmp) / n;
            //SpiralFrequency = 1 / timePeriod;
            //Debug.Log("Spiral freq: " + SpiralFrequency + " Amplitude: " + maxAmp / n);

            //Debug.Log(string.Join(" ", fft.Select(i => i.ToString("F2")).ToArray()));
        }

        void FixedUpdate()
        {
            int particleCount = ps.particleCount;
            if (!ps.isPlaying || particleCount == 0)
                return;

            dt = Time.deltaTime;
            dt2 = dt * dt;

            ParticleSystem.Particle[] particles = new ParticleSystem.Particle[particleCount];
            ps.GetParticles(particles);

            ps.GetCustomParticleData(customData, ParticleSystemCustomData.Custom1);
            
            DateTime curTime = DateTime.Now;

            for (int i = 0; i < particles.Length; ++i)
            {
                int particleIdx = Mathf.RoundToInt(customData[i].x);

                // Init custom data for new particle
                if (particleIdx <= 0)
                {
                    particleIdx = ++uniqueID;
                    customData[i] = new Vector4(particleIdx, 0, 0, 0);
                    cdata[particleIdx] = new ParticleCustomData(forceFields.Count);
                }

                var temp = cdata[particleIdx];
                AdvectParticleDirect(ref particles[i], ref temp, particleIdx);
                temp.NumTimestep++;
                temp.LastTime = curTime;
                cdata[particleIdx] = temp;

                var lifetime = cdata[particleIdx].NumTimestep * dt + cdata[particleIdx].ParamPhase;
                particles[i].remainingLifetime = MaxParticleLifetime - lifetime - dt;
            }

            
            ps.SetCustomParticleData(customData, ParticleSystemCustomData.Custom1);
            ps.SetParticles(particles);
        }

        private void AdvectParticleDirect(ref ParticleSystem.Particle p, ref ParticleCustomData data, int idx)
        {
            p.velocity = Vector3.zero;

            float time, normalizedTime;
            Vector3 T, N, B;
            Tuple<int, float> splineParam;
            float theta;

            // compute pseudo-random parameters
            if (data.NumTimestep == 0)
            {
                data.ParamPhase = dt * UnityEngine.Random.value;

                data.PositionNoiseSeed[0] = UnityEngine.Random.value * 50;
                data.PositionNoiseSeed[1] = UnityEngine.Random.value * 50;
                data.RotationNoiseSeed[0] = UnityEngine.Random.value * 50;
                data.RotationNoiseSeed[1] = UnityEngine.Random.value * 50;
            }

            // find Bishop frame
            time = data.NumTimestep * dt + data.ParamPhase;
            splineParam = emissionCurve.GetSplineParamFromTime(time);
            T = emissionCurve.Tangent(splineParam.Item1, splineParam.Item2);
            N = emissionCurve.Normal(splineParam.Item1, splineParam.Item2);
            B = Vector3.Cross(T, N);
            normalizedTime = time / MaxParticleLifetime;
            
            // compute other init parameters
            if (data.NumTimestep == 0)
            {
                var pos = p.position - Vector3.Dot(p.position, T) * T;
                data.Delta = pos.magnitude / emitterShape.radius;
                if (data.Delta > float.Epsilon)
                    data.Theta0 = Mathf.Atan2(Vector3.Dot(pos, B), Vector3.Dot(pos, N));
                else
                    data.Theta0 = 0;
            }

            theta = data.Theta0 + 2 * Mathf.PI * SpiralFrequency * time/* * normalizedTime*/;

            var emissionData = GetInterpolatedEmissionData(time);
            p.position = emissionCurve.Evaluate(splineParam.Item1, splineParam.Item2) +
                data.Delta * emissionData.Radius * (N * Mathf.Cos(theta) + B * Mathf.Sin(theta));

            
            Vector3 externalForceInfluence = Vector3.zero;
            foreach(var force in forceFields)
            {
                switch(force.EffectType)
                {
                    case ParticleCustomForceField.FieldEffectType.Gravity:
                        externalForceInfluence += transform.InverseTransformVector(force.ApplyField(p.position, ref data, GravityInfluence, idx));
                        break;
                    case ParticleCustomForceField.FieldEffectType.Wind:
                        externalForceInfluence += transform.InverseTransformVector(force.ApplyField(p.position, ref data, WindInfluence, idx));
                        break;
                }
            }

            p.position += externalForceInfluence;

            data.Xn_1[0] = data.Xn[0];
            data.Xn[0] = p.position;
            data.Nn_1 = N;

            Vector2 positionNoise = new Vector2(
                -1 + 2*Mathf.PerlinNoise(data.PositionNoiseSeed[0], PositionNoiseFrequency * time),
                -1 + 2*Mathf.PerlinNoise(data.PositionNoiseSeed[1], PositionNoiseFrequency * time)
                ) * MaxPositionNoise * normalizedTime;

            Vector2 rotationNoise = new Vector2(
                -1 + 2*Mathf.PerlinNoise(data.RotationNoiseSeed[0], RotationNoiseFrequency * time),
                -1 + 2 * Mathf.PerlinNoise(data.RotationNoiseSeed[1], RotationNoiseFrequency * time)
                ) * MaxRotationNoise * normalizedTime;

            p.position += N * positionNoise.x + B * positionNoise.y;

            Quaternion rot = Quaternion.LookRotation(T, B);
            
            rot *= ( Quaternion.AngleAxis(rotationNoise.x, N) * Quaternion.AngleAxis(rotationNoise.y, B) );
            p.rotation3D = (rot * InitialParticleRotation).eulerAngles;


            //if (idx == debugID)
            //    Debug.Log("Pos. " + p.position.ToString("F3") +
            //        " t: " + time.ToString("F5") +
            //        " Param: " + splineParam.Item1 + ", " + splineParam.Item2.ToString("F3") +
            //        " TNB: " + T.ToString("F2") + N.ToString("F2") + B.ToString("F2"));
        }

        public void NewEmissionCone(
            List<Vector3> leftHandPosition, List<Vector3> rightHandPosition,
            List<float> times,
            bool inputInWorldSpace = true)
        {
            Debug.Assert(leftHandPosition.Count == rightHandPosition.Count &&
                rightHandPosition.Count == times.Count);

            if (cdataGCTimer != null)
                cdataGCTimer.Stop();

            cdata.Clear();

            ps.Stop();
            ps.SetParticles(new ParticleSystem.Particle[0]);

            Func<Vector3, Vector3> identity = (x => { return x;});

            Func<Vector3, Vector3> worldToLocalPoint = inputInWorldSpace ? transform.InverseTransformPoint : identity;
            Func<Vector3, Vector3> localToWorldPoint = inputInWorldSpace ? transform.TransformPoint : identity;
            Func<Vector3, Vector3> localToWorldVector = inputInWorldSpace ? transform.TransformVector : identity;


            var twoHandAvgPosition = (from i in Enumerable.Range(0, leftHandPosition.Count)
                            select worldToLocalPoint((leftHandPosition[i] + rightHandPosition[i])/2)).ToList();

            List<int> subsampledIdx = new List<int>(twoHandAvgPosition.Count);
            subsampledIdx.Add(0);
            float distanceFromLastSample = 0;
            for (int i = 1; i < twoHandAvgPosition.Count; ++i)
            {
                distanceFromLastSample += (twoHandAvgPosition[i] - twoHandAvgPosition[i - 1]).magnitude;
                if (distanceFromLastSample > 0.01f)
                {
                    distanceFromLastSample = 0;
                    subsampledIdx.Add(i);
                }
            }

            twoHandAvgPosition = (from i in subsampledIdx select twoHandAvgPosition[i]).ToList();

            leftHandPosition = (from i in subsampledIdx select leftHandPosition[i] ).ToList();

            rightHandPosition = (from i in subsampledIdx select rightHandPosition[i]).ToList();

            times = (from i in subsampledIdx select times[i]).ToList();

            var shiftTwoHandAvg = (Vector3.zero - twoHandAvgPosition[0]);
            var shiftOneHand = localToWorldVector(shiftTwoHandAvg);
            var shiftTime = 0 - times[0];

            for (int i=0; i<times.Count; ++i)
            {
                twoHandAvgPosition[i] += shiftTwoHandAvg;
                leftHandPosition[i] += shiftOneHand;
                rightHandPosition[i] += shiftOneHand;
                times[i] += shiftTime;
            }

            var initialRadius = (leftHandPosition[0] - rightHandPosition[0]).magnitude / 2;
            emitterShape.radius = initialRadius;

            var initialSpeed = (twoHandAvgPosition[1] - twoHandAvgPosition[0]).magnitude / (times[1] - times[0]);
            main.startSpeed = new ParticleSystem.MinMaxCurve(Mathf.Max(Globals.PARTICLE_DEFAULT_START_SPEED, initialSpeed));

            var firstStepDistance = (twoHandAvgPosition[1] - twoHandAvgPosition[0]).magnitude;
            var firstStepTime = firstStepDistance / initialSpeed;

            
            MaxParticleLifetime = times[times.Count - 1];
            main.startLifetime = MaxParticleLifetime;

            emissionCurve.FitBezier(twoHandAvgPosition, times);

            var selectedIdx = emissionCurve.PointsKeepIdx;

            emissionDataKeys = new List<Tuple<float, EmissionData>>(selectedIdx.Count);

            for(int i=0; i<selectedIdx.Count; ++i)
            {
                var idx = selectedIdx[i];
                if (idx==0)
                {
                    emissionDataKeys.Add(new Tuple<float, EmissionData>(0, new EmissionData(initialRadius)));
                    continue;
                }

                var radius = worldToLocalPoint(leftHandPosition[idx]) - twoHandAvgPosition[idx];
                emissionDataKeys.Add(
                        new Tuple<float, EmissionData>(times[idx], 
                            new EmissionData(radius.magnitude)));
            }

            
            cdataGCTimer = new Timer(2000);
            cdataGCTimer.Elapsed += ParticleCustomDataGarbageCollection;
            cdataGCTimer.AutoReset = true;
            cdataGCTimer.Start();

            // track the first particle that will flow along the new curve
            debugID = uniqueID + 1;
            ps.Play();

            RenderedCurve.positionCount = emissionCurve.RenderPoints.Length;
            RenderedCurve.SetPositions(emissionCurve.RenderPoints);
            CreateEmissionConeRenderMesh();
            SetupEmissionConeColliders();
        }

        void SetupEmissionConeColliders()
        {
            foreach (Transform child in emissionCone.transform)
                Destroy(child.gameObject);

            for (int i = 0; i < emissionCurve.Curves.Count; ++i)
            {
                var pt1 = emissionCurve.Evaluate(i, 0.0f);
                var pt2 = emissionCurve.Evaluate(i, 1.0f);

                var t1 = emissionCurve.GetTimeFromSplineParam(i, 0.0f);
                var t2 = emissionCurve.GetTimeFromSplineParam(i, 1.0f);

                var r = Mathf.Max(GetInterpolatedEmissionData(t1).Radius, GetInterpolatedEmissionData(t2).Radius);

                var child = new GameObject(gameObject.name + "_coneColliderMesh_" + i);
                child.transform.SetParent(emissionCone.transform);
                child.transform.localPosition = (pt1 + pt2) / 2;
                child.transform.localRotation = Quaternion.LookRotation((pt2 - pt1));
                child.transform.localScale = new Vector3(r, r, (pt2 - pt1).magnitude);
                var collider = child.AddComponent<CapsuleCollider>();
                collider.radius = 1;
                collider.direction = 2;
                collider.height = 1;
            }
        }

        void CreateEmissionConeRenderMesh()
        {
            var numPts = Mathf.RoundToInt(10 * MaxParticleLifetime);
            var numSegs = numPts - 1;

            Vector3[] vertices = new Vector3[
                numPts * Globals.EMISSION_CONE_RENDER_DETAIL 
                /*+ 2*/];
            Vector3[] normals = new Vector3[vertices.Length];
            int[] triangles = new int[
                6 * numSegs * Globals.EMISSION_CONE_RENDER_DETAIL /*+
                3 * 2 * (Globals.EMISSION_CONE_RENDER_DETAIL)*/];

            Vector3 pt, N1, N2;
            float r;

            for (var idx = 0; idx < numPts; ++idx)
            {
                var arcLength = (float) idx / (numPts - 1);

                var splineParam = emissionCurve.GetSplineParamFromArcLength(arcLength);
                var bezierIdx = splineParam.Item1;
                var param = splineParam.Item2;

                pt = emissionCurve.Evaluate(bezierIdx, param);

                N1 = emissionCurve.Normal(bezierIdx, param);
                N2 = emissionCurve.Binormal(bezierIdx, param);

                r = GetInterpolatedEmissionData(emissionCurve.GetTimeFromSplineParam(bezierIdx, param)).Radius;
                
                for (var i = 0; i < Globals.EMISSION_CONE_RENDER_DETAIL; ++i)
                {
                    vertices[idx * Globals.EMISSION_CONE_RENDER_DETAIL + i] =
                        pt +
                        (float)Math.Cos(2 * Math.PI * (i) / Globals.EMISSION_CONE_RENDER_DETAIL) * r * N1 +
                        (float)Math.Sin(2 * Math.PI * (i) / Globals.EMISSION_CONE_RENDER_DETAIL) * r * N2;

                    normals[idx * Globals.EMISSION_CONE_RENDER_DETAIL + i] =
                        (vertices[idx * Globals.EMISSION_CONE_RENDER_DETAIL + i] - pt).normalized;
                }
            }

            // Add the vertices at the centers of the end caps
            //vertices[vertices.Length - 2] = emissionCurve.Evaluate(0, 0.0f);
            //vertices[vertices.Length - 1] = emissionCurve.Evaluate(emissionCurve.Curves.Count - 1, 1.0f);

            //normals[vertices.Length - 2] = -emissionCurve.Tangent(0, 0.0f);
            //normals[vertices.Length - 1] = emissionCurve.Tangent(emissionCurve.Curves.Count - 1, 1.0f);

            for (var seg = 0; seg < numSegs; ++seg)
            {
                for (var quad = 0; quad < Globals.EMISSION_CONE_RENDER_DETAIL; ++quad)
                {
                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 0] = 
                        Globals.EMISSION_CONE_RENDER_DETAIL * seg + quad;
                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 1] = 
                        Globals.EMISSION_CONE_RENDER_DETAIL * seg + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;
                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 2] =
                        Globals.EMISSION_CONE_RENDER_DETAIL * (seg + 1) + quad;

                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 3] = 
                        Globals.EMISSION_CONE_RENDER_DETAIL * seg + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;
                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 4] =
                        Globals.EMISSION_CONE_RENDER_DETAIL * (seg + 1) + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;
                    triangles[seg * Globals.EMISSION_CONE_RENDER_DETAIL * 6 + quad * 6 + 5] =
                        Globals.EMISSION_CONE_RENDER_DETAIL * (seg + 1) + quad;
                }
            }

            //var capTriStartIdx = 6 * numSegs * Globals.EMISSION_CONE_RENDER_DETAIL;
            //var endCapVertStartIdx = (numPts - 1) * Globals.EMISSION_CONE_RENDER_DETAIL;

            //var nStart = normals[vertices.Length - 2];
            //var nEnd = normals[vertices.Length - 1];

            //// Add the triangle strips at the end caps
            //for (var tri = 0; tri < Globals.EMISSION_CONE_RENDER_DETAIL; ++tri)
            //{
            //    triangles[capTriStartIdx + 3 * tri + 0] = vertices.Length - 2;
            //    triangles[capTriStartIdx + 3 * tri + 1] = tri;
            //    triangles[capTriStartIdx + 3 * tri + 2] = (tri + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;

            //    triangles[capTriStartIdx + 3 * (Globals.EMISSION_CONE_RENDER_DETAIL + tri) + 0] = 
            //        vertices.Length - 1;
            //    triangles[capTriStartIdx + 3 * (Globals.EMISSION_CONE_RENDER_DETAIL + tri) + 1] = 
            //        endCapVertStartIdx + tri;
            //    triangles[capTriStartIdx + 3 * (Globals.EMISSION_CONE_RENDER_DETAIL + tri) + 2] = 
            //        endCapVertStartIdx + (tri + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;

            //    normals[tri] = (normals[tri] + nStart).normalized;
            //    normals[endCapVertStartIdx + tri] = (normals[endCapVertStartIdx + tri] + nEnd).normalized;
            //}

            Mesh mesh = new Mesh();
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.normals = normals;

            RenderedConeMeshFilter.sharedMesh = mesh;
        }

        void AddPointToEmissionUI(Vector3 left, Vector3 right)
        {
            var avg = (left + right) / 2;
            var rad = (left - avg).magnitude;

            // Emission curve
            RenderedCurve.positionCount++;
            RenderedCurve.SetPosition(RenderedCurve.positionCount - 1, avg);

            Vector3[] vertices;
            Vector3[] normals;
            int[] triangles;

            // creating a stroke by storing in a mesh
            if (RenderedConeMeshFilter.sharedMesh)
            {
                vertices = RenderedConeMeshFilter.sharedMesh.vertices;
                triangles = RenderedConeMeshFilter.sharedMesh.triangles;
                normals = RenderedConeMeshFilter.sharedMesh.normals;
                RenderedConeMeshFilter.sharedMesh.Clear();
            }
            else
            {
                vertices = new Vector3[0];
                normals = new Vector3[0];
                triangles = new int[0];
            }

            int oldVertexLength = vertices.Length;
            Array.Resize(ref vertices, oldVertexLength + Globals.EMISSION_CONE_RENDER_DETAIL);
            Array.Resize(ref normals, oldVertexLength + Globals.EMISSION_CONE_RENDER_DETAIL);


            Vector3 N1, T;
            N1 = (left - avg).normalized;
            if (oldVertexLength == 0)
            {
                //First point of the stroke, no way to predict the tangent, so can only make a coarse approximation
                T = Vector3.forward;
            }
            else
            {
                Vector3 prev = RenderedCurve.GetPosition(RenderedCurve.positionCount - 2);
                T = (avg - prev).normalized;
                //    Vector3 tangent = (avg - prev).normalized;
                //    n1 = Vector3.Cross(tangent, targetHit.normal);
            }

            Vector3 N2 = Vector3.Cross(T, N1).normalized;

            for (int i = 0; i < Globals.EMISSION_CONE_RENDER_DETAIL; ++i)
            {
                vertices[oldVertexLength + i] =
                avg +
                (float)Mathf.Cos(2 * Mathf.PI * (i) / Globals.EMISSION_CONE_RENDER_DETAIL) * rad * N1 +
                (float)Mathf.Sin(2 * Mathf.PI * (i) / Globals.EMISSION_CONE_RENDER_DETAIL) * rad * N2;
                normals[oldVertexLength + i] = (vertices[oldVertexLength + i] - avg).normalized;
            }

            if (oldVertexLength > 0)
            {
                int oldTriangleLength = triangles.Length;
                int curVertexRingStartIdx = oldVertexLength;
                int prevVertexRingStartIdx = curVertexRingStartIdx - Globals.EMISSION_CONE_RENDER_DETAIL;

                Array.Resize(ref triangles, oldTriangleLength + Globals.EMISSION_CONE_RENDER_DETAIL * 6);
                for (int quad = 0; quad < Globals.EMISSION_CONE_RENDER_DETAIL; ++quad)
                {
                    triangles[oldTriangleLength + quad * 6 + 0] = prevVertexRingStartIdx + quad;
                    triangles[oldTriangleLength + quad * 6 + 1] = curVertexRingStartIdx + quad;
                    triangles[oldTriangleLength + quad * 6 + 2] = prevVertexRingStartIdx + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;

                    triangles[oldTriangleLength + quad * 6 + 3] = prevVertexRingStartIdx + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;
                    triangles[oldTriangleLength + quad * 6 + 4] = curVertexRingStartIdx + quad;
                    triangles[oldTriangleLength + quad * 6 + 5] = curVertexRingStartIdx + (quad + 1) % Globals.EMISSION_CONE_RENDER_DETAIL;
                }
            }

            Mesh coneMesh = new Mesh();
            coneMesh.vertices = vertices;
            coneMesh.normals = normals;
            coneMesh.triangles = triangles;
            RenderedConeMeshFilter.sharedMesh = coneMesh;
        }

        private void ParticleCustomDataGarbageCollection(object state, ElapsedEventArgs e)
        {
            List<int> keys = new List<int>();
            var curTime = DateTime.Now;
            foreach (var data in cdata)
                if ((curTime - data.Value.LastTime).TotalSeconds > 10.0f)
                    keys.Add(data.Key);

            //Debug.Log("GC'ing #keys: " + keys.Count);

            foreach (var key in keys)
                cdata.Remove(key);
        }

        public int FindClosestIndex<T>(List<Tuple<float, T>> list, float value)
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

        private EmissionData GetInterpolatedEmissionData(float time)
        {
            if (emissionDataKeys.Count == 0)
                return new EmissionData();

            var nearest = FindClosestIndex(emissionDataKeys, time);

            var param0 = emissionDataKeys[nearest].Item1;

            if (param0 >= time && nearest == 0)
            {
                return emissionDataKeys[0].Item2;
            }
            else if (param0 <= time && nearest == emissionDataKeys.Count - 1)
            {
                return emissionDataKeys[emissionDataKeys.Count - 1].Item2;
            }
            else if (param0 > time)
                nearest--;

            var param1 = emissionDataKeys[nearest + 1].Item1;

            var alpha = (time - param0) / (param1 - param0);

            var val0 = emissionDataKeys[nearest].Item2;
            var val1 = emissionDataKeys[nearest + 1].Item2;

            // use smoothstep
            alpha = Mathf.SmoothStep(0, 1, Mathf.Clamp01(alpha));

            return EmissionData.Lerp(val0, val1, alpha);
        }

        private void ProcessObjectSelection()
        {
            emissionCone.GetComponent<Rigidbody>().detectCollisions = true;

            ps.Pause();
            if (cdataGCTimer != null)
                cdataGCTimer.Enabled = false;
        }

        private void ProcessObjectDeselection()
        {
            emissionCone.GetComponent<Rigidbody>().detectCollisions = false;
            emissionCone.ResetIntersectionCounter();

            ps.Play();
            if (cdataGCTimer != null)
                cdataGCTimer.Enabled = true;
        }

        public void TryChangeParticleShapeFromDroppedObject(ShelfObject obj)
        {
            try
            {
                ParticleMeshCreator = obj;
                GameObject particle = Instantiate(obj.ParticlePrefab);
                if (particle == null)
                {
                    Debug.LogWarning("No particle prefab found!");
                    return;
                }
                var mr = particle.GetComponentInChildren<MeshRenderer>();
                //var sizeEstimate = mr.bounds.size.magnitude;
                ParticleScale = mr.transform.lossyScale;
                main.startSizeX = ParticleScale.x;
                main.startSizeY = ParticleScale.y;
                main.startSizeZ = ParticleScale.z;
                InitialParticleRotation = mr.transform.rotation;

                var psr = GetComponent<ParticleSystemRenderer>();
                var mesh = mr.GetComponent<MeshFilter>().mesh;
                var verts = mesh.vertices;
                Vector3 avg = Vector3.zero;
                for (int i = 0; i < verts.Length; ++i)
                    avg += verts[i];
                avg /= verts.Length;
                for (int i = 0; i < verts.Length; ++i)
                    verts[i] -= avg;

                mesh.SetVertices(verts.ToList());
                psr.mesh = mesh;
                //Material newMat = new Material(mr.GetComponent<MeshRenderer>().material);
                //newMat.shader = newMat.shader;
                psr.material = mr.GetComponent<MeshRenderer>().material;
                Destroy(particle);
                ps.Stop();
                ps.Play();
            }
            catch(Exception e)
            {
                Debug.LogWarning(e.Message);
                Debug.Log(e.StackTrace);
            }
        }

        private void RunTest()
        {
            List<Vector3> lhp = new List<Vector3>
                {
                    new Vector3(-.25f, 0, 0.05f),
                    new Vector3(-0.4f, 0, 0.35f),
                    new Vector3(-1.2f, 0, 1.30f),
                    new Vector3( 0.0f, 0, 2.10f),
                    new Vector3(0.65f, 0, 2.00f)
                };

            List<Vector3> rhp = new List<Vector3>
                {
                    new Vector3(0.25f, 0, 0.05f),
                    new Vector3(0.00f, 0, 0.70f),
                    new Vector3(-.60f, 0, 1.30f),
                    new Vector3(0.00f, 0, 1.60f),
                    new Vector3(0.45f, 0, 1.55f)
                };

            List<Vector3> lhv = new List<Vector3>
                {
                    new Vector3(-5, 0, 0.5f),
                    new Vector3(-5, 0, 0.5f),
                    new Vector3(-5, 0, 0),
                    new Vector3(-5, 0, 0),
                    new Vector3( 1, 0,-1)
                };

            List<Vector3> lha = new List<Vector3>
                {
                    new Vector3( 0, 0, 0.5f),
                    new Vector3(-1, 0, 0),
                    new Vector3(-1, 0, 0.5f),
                    new Vector3( 0.5f, 0,-1),
                    new Vector3( 0, 0,-.5f)
                };

            List<float> times = new List<float>
            {
                0, 1, 2, 3, 4
            };

            NewEmissionCone(lhp, rhp, times, false);

            int numCurve = emissionCurve.Curves.Count;
            int n = 50;
            lhp = new List<Vector3>(numCurve * n);
            rhp = new List<Vector3>(numCurve * n);
            times = new List<float>(numCurve * n);

            float radius = 0.05f;
            var lastPoint = Vector3.zero + radius * Vector3.left;
            float lastTime = 0;

            for(int i=0; i<numCurve; ++i)
            {
                for (int j = 1; j < n; ++j)
                {
                    var mul = 1.0f + (float)((n-1)*i + j) / 10;
                    var pt = emissionCurve.Evaluate(i, (float)j / n);
                    var N = emissionCurve.Normal(i, (float)j / n);
                    lhp.Add(pt + mul * radius * Vector3.left);
                    rhp.Add(pt + mul * radius * Vector3.right);

                    var deltaT = (lhp[lhp.Count - 1] - lastPoint).magnitude;
                    times.Add(lastTime + deltaT);

                    lastPoint = lhp[lhp.Count - 1];
                    lastTime = times[times.Count - 1];
                }
            }

            NewEmissionCone(lhp, rhp, times, false);

            //MeshCollider meshCollider = RenderedConeMeshFilter.gameObject.GetComponent<MeshCollider>();
            //if (meshCollider == null)
            //    meshCollider = RenderedConeMeshFilter.gameObject.AddComponent<MeshCollider>();

            //meshCollider.cookingOptions |= (
            //    MeshColliderCookingOptions.CookForFasterSimulation |
            //    MeshColliderCookingOptions.EnableMeshCleaning |
            //    MeshColliderCookingOptions.WeldColocatedVertices);

            //meshCollider.sharedMesh = RenderedConeMeshFilter.sharedMesh;
            //List<Vector3> lhp = new List<Vector3>
            //{
            //    new Vector3(-.50f, 0, .05f),
            //    new Vector3(-.15f, 0, 2.0f)
            //};

            //List<Vector3> lhv = new List<Vector3>
            //{
            //    new Vector3(.5f, 0, .5f),
            //    new Vector3(  0, 0, .5f)
            //};

            //List<Vector3> lha = new List<Vector3>
            //{
            //    new Vector3(.5f, 0, .25f),
            //    new Vector3(.5f, 0, .25f)
            //};


            //var rhp = lhp.Select(i => new Vector3(-i.x, i.y, i.z)).ToList();
            //var rhv = lhv.Select(i => new Vector3(-i.x, i.y, i.z)).ToList();
            //var rha = lha.Select(i => new Vector3(-i.x, i.y, i.z)).ToList();

            //NewEmissionCone(lhp, rhp, lhv, rhv, lha, rha, false);
        }

        void CreateDefaultEmission()
        {
            List<Vector3> lhp = new List<Vector3>
            {
                new Vector3(-.25f, 0, 0.1f),
                new Vector3(-.25f, 0, 0.5f),
                new Vector3(-.25f, 0, 1.0f),
            };

            var rhp = lhp.Select(i => new Vector3(-i.x, i.y, i.z)).ToList();
            var times = lhp.Select(i => i.z).ToList();

            NewEmissionCone(lhp, rhp, times, false);
        }

        public void SetEmissionConeFromFile(List<Vector3> controlPoints, List<float> timeCache, List<Tuple<float, EmissionData>> emissionData)
        {
            emissionDataKeys = emissionData;

            emissionCurve.SetCurves(controlPoints);
            emissionCurve.SetTimeCache(timeCache);

            if (cdataGCTimer != null)
                cdataGCTimer.Stop();

            cdata.Clear();

            ps.Stop();
            ps.SetParticles(new ParticleSystem.Particle[0]);

            emitterShape.radius = GetInterpolatedEmissionData(0.0f).Radius;

            var initialSpeed = emissionCurve.GetArcLengthFromSplineParam(0, 0.1f) / emissionCurve.GetTimeFromSplineParam(0, 0.1f);
            main.startSpeed = new ParticleSystem.MinMaxCurve(Mathf.Max(Globals.PARTICLE_DEFAULT_START_SPEED, initialSpeed));

            MaxParticleLifetime = emissionCurve.GetTimeFromSplineParam(emissionCurve.Curves.Count - 1, 0);
            main.startLifetime = MaxParticleLifetime;

            cdataGCTimer = new Timer(2000);
            cdataGCTimer.Elapsed += ParticleCustomDataGarbageCollection;
            cdataGCTimer.AutoReset = true;
            cdataGCTimer.Start();

            debugID = uniqueID + 1;
            ps.Play();

            RenderedCurve.positionCount = emissionCurve.RenderPoints.Length;
            RenderedCurve.SetPositions(emissionCurve.RenderPoints);
            CreateEmissionConeRenderMesh();
            SetupEmissionConeColliders();
        }

        public Vector3[] GetEmissionCurveControlPoints()
        {
            return emissionCurve.ControlPoints;
        }

        public List<Tuple<float, EmissionData>> GetEmissionDataKeys()
        {
            return emissionDataKeys;
        }

        public float[] GetParamToTimeCache()
        {
            return emissionCurve.GetFlattenedCache(BezierFit.ParameterType.Time);
        }
    }
}