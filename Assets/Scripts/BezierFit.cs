//////////////////////////////////////////////////////////////////////////////////////
/// BezierFit interfaces with the external code (in ./Bezier/) and provides utility
/// functions for storing data on and retrieving data from Bezier splines.
//////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using burningmime.curves;
using g3;

namespace GestureAnim
{
    class PointSet : g3.IPointSet
    {
        //////////////////////////////////////////////////////////////////////////////////////
        /// PointSet implements the g3.IPointSet interface and defines the necessary functions
        /// for using a g3.PointAABBTree for efficiently fitting Bezier splines to an input
        /// set of points.
        //////////////////////////////////////////////////////////////////////////////////////

        private Vector3d[] vertices;

        public PointSet(Vector3d[] vertIn)
        {
            vertices = vertIn;
        }

        public int VertexCount
        {
            get { return vertices.Length; }
        }

        public int MaxVertexID
        {
            get { return vertices.Length - 1; }
        }

        public bool HasVertexNormals { get { return false; } }
        public bool HasVertexColors { get { return false; } }

        public Vector3d GetVertex(int i)
        {
            return i < vertices.Length ? vertices[i] : Vector3d.Zero;
        }

        public Vector3f GetVertexNormal(int i)
        {
            return Vector3f.Zero;
        }

        public Vector3f GetVertexColor(int i)
        {
            return Vector3f.Zero;
        }

        public bool IsVertex(int vID)
        {
            return vID < vertices.Length ? true : false;
        }

        // iterators allow us to work with gaps in index space
        public IEnumerable<int> VertexIndices()
        {
            for (int i = 0; i < vertices.Length; ++i)
                yield return i;
        }

        public int Timestamp { get { return -1; } }
    }

    public class BezierFit
    {
        private List<Vector3> InputPoints;// { get; private set; }
        private List<float> InputPointArcLengths;// { get; private set; }
        public List<int> PointsKeepIdx { get; private set; }

        private List<Vector3d> AllInputPoints;
        private List<float> AllInputPointTimes;

        public List<CubicBezier> Curves { get; private set; } = new List<CubicBezier>();

        public Vector3[] ControlPoints { get; private set; } = new Vector3[0];

        [DllImport("CubicSplineClosestPoint", EntryPoint = "CreateCubicBezierPath")]
        private static extern IntPtr _CreateUnmanagedPathObject(float[] cpData, int numPts);

        [DllImport("CubicSplineClosestPoint", EntryPoint = "FindClosestPoint")]
        private static extern bool _GetClosestPoint(IntPtr path, float[] point, float[] closest, ref int bezierIdx, ref float param);

        [DllImport("CubicSplineClosestPoint", EntryPoint = "DeleteSplinePath")]
        private static extern bool _DeleteUnmanagedPathObject(IntPtr path);

        private IntPtr _path = IntPtr.Zero;

        private float[,] paramToArcLengthCache;

        private float[,] paramToTimeCache;

        public bool TestMode;

        public Vector3[] RenderPoints { get; private set; }

        public enum ParameterType { Time, ArcLength};

        PointAABBTree3 inputPointsTree;

        public BezierFit()
        {
            if (TestMode)
            {
                UnityEngine.XR.XRSettings.enabled = false;
                Vector3[] pts =
                {
                    new Vector3(-.4f, -.4f, 1.6f),
                    new Vector3(-.3f, -.3f, 0.9f),
                    new Vector3(-.2f, -.2f, 0.4f),
                    new Vector3(-.1f, -.1f, 0.1f),
                    new Vector3(0.0f, 0.0f, 0.0f),
                    new Vector3(0.1f, 0.1f, 0.1f),
                    new Vector3(0.2f, 0.2f, 0.4f),
                    new Vector3(0.3f, 0.3f, 0.9f),
                    new Vector3(0.4f, 0.4f, 1.6f),
                    new Vector3(0.0f, 0.0f, 1.6f),
                    new Vector3(0.5f, 0.5f, 2.5f),
                    new Vector3(0.5f, 0.5f, -1.0f)
                };

                float[] times = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

                FitBezier(new List<Vector3>(pts), new List<float>(times));

                foreach (var pt in ControlPoints)
                {
                    Debug.Log(pt.ToString("F2"));
                }

                foreach (var pt in pts)
                {
                    Debug.Log("Input point: " + pt.ToString("F2"));
                    var res = GetClosestPoint(pt, out Vector3 closest, out int bezierIdx, out float param);

                    if (res)
                        Debug.Log("Closest point: " + closest.ToString("F2") + 
                            " found at (" + bezierIdx.ToString() + ", " + param.ToString("F4") + ")." +
                            " s=" + GetArcLengthFromSplineParam(bezierIdx, param).ToString("F4"));
                    else
                        Debug.LogWarning("Closest point search failed!");
                }
            }
        }

        ~BezierFit()
        {
            if (_path != IntPtr.Zero)
                _DeleteUnmanagedPathObject(_path);
        }

        public List<CubicBezier> FitBezier(List<Vector3> pts, List<float> times, bool useRdp = true)
        {
            AllInputPoints = new List<Vector3d>(pts.Count);

            foreach (var pt in pts)
                AllInputPoints.Add(pt);
            inputPointsTree = new PointAABBTree3(new PointSet(AllInputPoints.ToArray()));

            AllInputPointTimes = times;

            if (useRdp)
            {
                var reduced = CurvePreprocess.RdpReduce(pts, Globals.RDP_ERROR, out List<int> keepIdx);
                PointsKeepIdx = keepIdx;
                InputPoints = reduced;
                Curves = new List<CubicBezier>(CurveFit.Fit(reduced, Globals.BEZIER_FIT_MAX_ERROR));
            }
            else
            {
                InputPoints = pts;
                Curves = new List<CubicBezier>(CurveFit.Fit(pts, Globals.BEZIER_FIT_MAX_ERROR));
            }

            Curves[0] = new CubicBezier(
                Curves[0].p0,
                Curves[0].p0 + (Curves[0].p1 - Curves[0].p0).magnitude * Vector3.forward,
                Curves[0].p2,
                Curves[0].p3);

            if (Curves.Count == 0)
                ControlPoints = new Vector3[0];
            else
            {
                ControlPoints = new Vector3[1 + 3 * Curves.Count];
                ControlPoints[0] = Curves[0].p0;
            }
            for (int i = 0; i < Curves.Count; ++i)
            {
                ControlPoints[3 * i + 1] = Curves[i].p1;
                ControlPoints[3 * i + 2] = Curves[i].p2;
                ControlPoints[3 * i + 3] = Curves[i].p3;
            }

            
            float[] cpData = new float[ControlPoints.Length * 3];
            for (int i = 0; i < ControlPoints.Length; ++i)
            {
                cpData[3 * i + 0] = ControlPoints[i].x;
                cpData[3 * i + 1] = ControlPoints[i].y;
                cpData[3 * i + 2] = ControlPoints[i].z;
            }

            if (_path != IntPtr.Zero)
                _DeleteUnmanagedPathObject(_path);
            _path = _CreateUnmanagedPathObject(cpData, ControlPoints.Length);

            CacheArcLength();

            if (times.Count == 0)
                paramToTimeCache = paramToArcLengthCache;
            else
                CacheTime();

            InputPointArcLengths = new List<float>(InputPoints.Count);
            foreach(var pt in InputPoints)
            {
                var res = GetClosestPoint(pt, out Vector3 closest, out int bezierIdx, out float param);
                if (res)
                    InputPointArcLengths.Add(GetArcLengthFromSplineParam(bezierIdx, param));
                else
                    throw new Exception("Something horrible happened!");
            }

            return Curves;
        }

        void CacheTime()
        {
            paramToTimeCache = new float[Curves.Count, Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES];

            for (int i = 0; i < Curves.Count; ++i)
            {
                for (int j = 0; j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES; ++j)
                {
                    var thisPoint = Curves[i].Sample(((float)j) / (Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1));

                    int minIdx = inputPointsTree.FindNearestPoint(thisPoint);

                    paramToTimeCache[i, j] = AllInputPointTimes[minIdx];
                }

                for (int j=1; j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES; ++j)
                {
                    if (paramToTimeCache[i, j] - paramToTimeCache[i, j-1] < 1e-6)
                    {
                        int s = j - 1;
                        int e1 = j + 1;
                        while (e1 < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES &&
                            paramToTimeCache[i, e1] - paramToTimeCache[i, j] < 1e-6)
                            e1++;
                        e1--;

                        int e2 = e1 + 1;
                        while (e2 < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES &&
                            paramToTimeCache[i, e2] - paramToTimeCache[i, e1 + 1] < 1e-6)
                            e2++;
                        e2--;

                        if (s == 0 && e2 == e1)
                            throw new Exception("time Param is degenerate across Bezier " + i);
                        else if (s == 0)
                            _RedistParam(i, s, e2);
                        else if (e2 == e1)
                            _RedistParam(i, s - 1, e1);
                        else
                        {
                            int m = (s + e1) / 2;
                            _RedistParam(i, s - 1, m);
                            _RedistParam(i, m + 1, e2);
                        }

                        j = e2 + 1;
                    }
                }
            }
        }

        void _RedistParam(int r, int s, int e)
        {
            var p0 = paramToTimeCache[r, s];
            var pDiff = paramToTimeCache[r, e] - paramToTimeCache[r, s];
            for (int i = s + 1; i < e; ++i)
                paramToTimeCache[r, i] = p0 + (i - s) * pDiff / (s - e);
        }

        void CacheArcLength()
        {
            float totalArcLength = 0.0f;
            RenderPoints = new Vector3[Curves.Count * (Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1) + 1];

            paramToArcLengthCache = new float[Curves.Count, Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES];

            Vector3 lastPoint = Curves[0].p0;
            
            for (int i = 0; i < Curves.Count; ++i)
            {
                for (int j = 0; j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES; ++j)
                {
                    var thisPoint = Curves[i].Sample(((float)j) / (Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1));
                    var thisLen = (thisPoint - lastPoint).magnitude;
                    paramToArcLengthCache[i, j] = totalArcLength + thisLen;
                    totalArcLength += thisLen;

                    lastPoint = thisPoint;
                    if (j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1)
                        RenderPoints[j + i * (Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1)] = thisPoint;
                }
            }

            RenderPoints[RenderPoints.Length - 1] = Curves[Curves.Count - 1].Sample(1.0f);

            for (int i = 0; i < Curves.Count; ++i)
                for (int j = 0; j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES; ++j)
                    paramToArcLengthCache[i, j] /= totalArcLength;
        }

        public void SetCurves(List<Vector3> pts)
        {
            Debug.Assert(pts.Count % 3 == 1, "Number of input control points must be 3|C| + 1");
            ControlPoints = pts.ToArray();
            Curves = new List<CubicBezier>((pts.Count - 1)/ 3);
            for (int i = 0; i < pts.Count - 3; i += 3)
                Curves.Add(new CubicBezier(pts[i+0], pts[i+1], pts[i+2], pts[i+3]));

            CacheArcLength();
        }

        public void SetTimeCache(List<float> timeCache)
        {
            var ptsPerCurve = Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES;
            Debug.Assert(timeCache.Count == ptsPerCurve * Curves.Count, 
                "Length of input time cache is incorrect! " + 
                timeCache.Count + " != " + ptsPerCurve * Curves.Count);

            paramToTimeCache = new float[Curves.Count, ptsPerCurve];

            for(int i=0; i<Curves.Count; ++i)
                for (int j = 0; j < ptsPerCurve; ++j)
                    paramToTimeCache[i, j] = timeCache[i * ptsPerCurve + j];
        }

        public Tuple<int, float> GetSplineParamFromOther(float otherParam, ParameterType type)
        {
            switch(type)
            {
                case ParameterType.ArcLength:
                    return _GetSplineParamFromOther(otherParam, paramToArcLengthCache);
                case ParameterType.Time:
                    return _GetSplineParamFromOther(otherParam, paramToTimeCache);
                default:
                    return _GetSplineParamFromOther(otherParam, paramToArcLengthCache);
            }
        }

        public Tuple<int, float> _GetSplineParamFromOther(float other, float[,] paramCache)
        {
            var maxParam = paramCache[Curves.Count - 1, Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1];

            if (other <= float.Epsilon)
                return new Tuple<int, float>(0, 0.0f);
            else if (other >= (maxParam - float.Epsilon))
                return new Tuple<int, float>(Curves.Count - 1, 1.0f);

            int bezierIdx = 0;
            while (bezierIdx < Curves.Count &&
                paramCache[bezierIdx, 0] < other)
                bezierIdx++;
            bezierIdx--;

            int paramIdx = 0;
            while (paramIdx < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES &&
                paramCache[bezierIdx, paramIdx] < other)
                paramIdx++;
            paramIdx--;

            int param0Idx = paramIdx;
            int param1Idx = paramIdx + 1;

            
            float alpha =
                (other - paramCache[bezierIdx, param0Idx]) /
                (paramCache[bezierIdx, param1Idx] - paramCache[bezierIdx, param0Idx]);

            alpha = Mathf.Clamp01(alpha);

            float param0 = (float)param0Idx / Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES;
            float param1 = (float)param1Idx / Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES;

            float param = (1 - alpha) * param0 + alpha * param1;

            return new Tuple<int, float>(bezierIdx, param);
        }

        public Tuple<int, float> GetSplineParamFromArcLength(float arcLength)
        {
            return GetSplineParamFromOther(arcLength, ParameterType.ArcLength);
        }

        public float GetArcLengthFromSplineParam(Tuple<int, float> splineParam)
        {
            return GetArcLengthFromSplineParam(splineParam.Item1, splineParam.Item2);
        }

        public float GetCachedParamFromSplineParam(int bezierIdx, float param, ParameterType type)
        {
            switch (type)
            {
                case ParameterType.ArcLength:
                    return _GetCachedParamFromSplineParam(bezierIdx, param, paramToArcLengthCache);
                case ParameterType.Time:
                    return _GetCachedParamFromSplineParam(bezierIdx, param, paramToTimeCache);
                default:
                    return _GetCachedParamFromSplineParam(bezierIdx, param, paramToArcLengthCache);
            }
        }

        private float _GetCachedParamFromSplineParam(int bezierIdx, float param, float[,] paramCache)
        {
            try
            {
                if (paramCache == null)
                    throw new UnauthorizedAccessException("Param cache not initialized yet!");

                if (param <= float.Epsilon)
                    return paramCache[bezierIdx, 0];
                else if (param >= (1.0 - float.Epsilon))
                    return paramCache[bezierIdx, Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1];


                int paramIdx = Mathf.FloorToInt(param * (Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES - 1));

                float param0 = (float)paramIdx / Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES;
                float param1 = (float)(paramIdx + 1) / Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES;

                float alpha = (param - param0) / (param1 - param0);

                return (1 - alpha) * paramCache[bezierIdx, paramIdx] + alpha * paramCache[bezierIdx, paramIdx + 1];
            }
            catch (Exception e)
            {
                Debug.Log(bezierIdx + ", " + param.ToString("F3") + ": " + e.Message);
                return 0;
            }
        }

        public float GetArcLengthFromSplineParam(int bezierIdx, float param)
        {
            return GetCachedParamFromSplineParam(bezierIdx, param, ParameterType.ArcLength);
        }

        public float GetTimeFromSplineParam(Tuple <int, float> splineParam)
        {
            return GetTimeFromSplineParam(splineParam.Item1, splineParam.Item2);
        }

        public float GetTimeFromSplineParam(int bezierIdx, float param)
        {
            return GetCachedParamFromSplineParam(bezierIdx, param, ParameterType.Time);
        }

        public Tuple<int, float> GetSplineParamFromTime(float time)
        {
            return GetSplineParamFromOther(time, ParameterType.Time);
        }

        public int GetNearestControlPointIdx(Vector3 point)
        {
            float minDist = float.MaxValue;
            int minIdx = -1;

            for (int i = 0; i < ControlPoints.Length; ++i)
            {
                var dist = (point - ControlPoints[i]).sqrMagnitude;
                if (dist < minDist)
                {
                    minDist = dist;
                    minIdx = i;
                }
            }

            return minIdx;
        }

        public CubicBezier GetCurvesFromControlPointIdx(int idx, out bool twoCurves, out CubicBezier curve2)
        {
            curve2 = new CubicBezier();
            twoCurves = false;

            if (idx > ControlPoints.Length)
                throw new ArgumentOutOfRangeException();

            if (idx > 0 && idx % 3 == 0)
            {
                twoCurves = true;
                curve2 = Curves[idx / 3];
            }

            return Curves[idx / 3 - 1];
        }

        public CubicBezier GetNearestCurves(Vector3 point, out bool twoCurves, out CubicBezier curve2)
        {
            var idx = GetNearestControlPointIdx(point);
            return GetCurvesFromControlPointIdx(idx, out twoCurves, out curve2);
        }

        public bool GetClosestPoint(Vector3 point, out Vector3 closest, out int bezierIdx, out float param)
        {
            bezierIdx = -1;
            param = 0.0f;
            closest = Vector3.zero;
            if (_path == IntPtr.Zero)
                return false;

            float[] ptdata = { point.x, point.y, point.z };
            float[] closestData = new float[3];

            bool res = _GetClosestPoint(_path, ptdata, closestData, ref bezierIdx, ref param);

            closest.x = closestData[0]; closest.y = closestData[1]; closest.z = closestData[2];

            return res;
        }

        public Vector3 Evaluate(int bezierIdx, float param)
        {
            _FixSplineParam(ref bezierIdx, ref param);

            return Curves[bezierIdx].Sample(param);
        }

        public Vector3 Tangent(float arcLength)
        {
            var splineParam = GetSplineParamFromArcLength(arcLength);
            return Curves[splineParam.Item1].Tangent(splineParam.Item2);
        }

        public Vector3 Tangent(int bezierIdx, float param)
        {
            _FixSplineParam(ref bezierIdx, ref param);

            return Curves[bezierIdx].Tangent(param);
        }

        public Vector3 Normal(int bezierIdx, float param)
        {
            _FixSplineParam(ref bezierIdx, ref param);

            var N = Vector3.Cross(Tangent(bezierIdx, param), Vector3.up);

            return (N == Vector3.zero) ? Vector3.Cross(Curves[bezierIdx].Tangent(param), Vector3.right) : N;
        }

        public Vector3 Binormal(int bezierIdx, float param)
        {
            _FixSplineParam(ref bezierIdx, ref param);

            return Vector3.Cross(Tangent(bezierIdx, param), Normal(bezierIdx, param));
        }

        private void _FixSplineParam(ref int bezierIdx, ref float param)
        {
            if (bezierIdx == Curves.Count && param <= float.Epsilon)
            {
                bezierIdx = Curves.Count - 1;
                param = 1.0f;
            }
        }

        public float[] GetFlattenedCache(ParameterType type)
        {
            var paramCache = paramToTimeCache;
            switch(type)
            {
                case ParameterType.ArcLength:
                    paramCache = paramToArcLengthCache;
                    break;
            }

            var flatCache = new float[paramCache.Length];
            for (int i = 0; i < Curves.Count; ++i)
                for (int j = 0; j < Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES; ++j)
                    flatCache[i * Globals.BEZIER_PARAM_CACHE_NUM_SAMPLES + j] = paramCache[i, j];

            return flatCache;
        }
    }
}
