using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using Grasshopper.Kernel.Parameters;
using KangarooSolver;

namespace KangarooSolver.Goals
{
    public class SphereCollide_wqs : GoalObject
    {
        public double Strength;
        //public double SqDiam;
        //public double Diam;
        public double r0;
        public List<Point3d> specialPoints;
        public double kSV = 1;
        public double gSV = 1;
        public List<Curve> specialCurves;
        public double kSC = 1;
        public double gSC = 1;
        public List<Point3d> curveReferPoints;
        public Curve referCurve;
        public double kCR = 1;
        public List<double> kCRs;
        public double gCR = 1;
        public Surface referSurface;
        public List<Point3d> surfaceReferPoints;
        public double kSR = 1;
        public List<double> kSRs;
        public double gSR = 1;
        static public List<double> ars;
        //控制气泡大小和特定点的距离相关
        public SphereCollide_wqs(List<Point3d> V, double r, double w,
            List<Point3d> SV = null, double kP = 1, double gP = 1,
             List<Curve> SC = null, double kL = 1, double gL = 1,
              Curve rCurve = null, List<Point3d> CRP = null, double kCC = 1, double gCC = 1,
              Surface rSurface = null, List<Point3d> SRP = null, double kCS = 1, double gCS = 1)
        {
            int L = V.Count;
            PPos = V.ToArray();
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int i = 0; i < L; i++)
            {
                Weighting[i] = w;
            }
            r0 = r;
            ars = new List<double>(V.Count);
            for (int i = 0; i < V.Count; i += 1)
                ars.Add(r0);
            //Diam = r + r;
            // SqDiam = Diam * Diam;
            Strength = w;
            specialPoints = SV;
            kSV = kP;
            gSV = gP;
            specialCurves = SC;
            kSC = kL;
            gSC = gL;

            double clb = 0.1 / r0;//考虑的曲率下线
            curveReferPoints = CRP;
            referCurve = rCurve;
            kCR = kCC;
            gCR = gCC;
            foreach (var rp in curveReferPoints)
            {
                double t;
                if (referCurve.ClosestPoint(rp, out t))
                {
                    double cCurvature = referCurve.CurvatureAt(t).Length;
                    double k = kCR+(1-kCR)*Math.Abs(clb/ cCurvature);
                    kCRs.Add(k);
                }
            }
            surfaceReferPoints = SRP;
            referSurface = rSurface;
            kSR = kCS;
            gSR = gCS;
            foreach (var rp in surfaceReferPoints)
            {
                double u,v;
                if (referSurface.ClosestPoint(rp, out u,out v))
                {
                    double sCurvature = referSurface.CurvatureAt(u,v).Gaussian;
                    double k = kSR + (1 - kSR) * Math.Abs(clb / sCurvature);
                    kSRs.Add(k);
                }
            }
        }
        private double RadiusFactor(Point3d p3d)
        {
            double hmin = 1;
            if (specialPoints != null)
            {
                double minD = gSC * r0;
                for (int j = 0; j < specialPoints.Count; j++)
                {
                    Vector3d Separation = specialPoints[j] - p3d;
                    double d = Separation.Length;
                    if (minD > d)
                        minD = d;
                }
                hmin = Math.Min(hmin, kSV + (1 - kSV) * minD / (gSV * r0));
            }
            if (specialCurves != null)
            {
                double minD = gSC * r0;
                for (int j = 0; j < specialCurves.Count; j++)
                {
                    double t;
                    Curve curve = specialCurves[j];
                    if (curve.ClosestPoint(p3d, out t, gSC * r0))
                    {

                        Point3d closedP = curve.PointAt(t);
                        Vector3d Separation = closedP - p3d;
                        double d = Separation.Length;
                        if (minD > d)
                            minD = d;
                    }
                }
                hmin = Math.Min(hmin, kSC + (1 - kSC) * minD / (gSC * r0));
            }
            if (curveReferPoints != null)
            {
                double kmin = 1;
                for (int j = 0; j < curveReferPoints.Count; j++)
                {
                    Vector3d Separation = curveReferPoints[j] - p3d;
                    double d = Separation.Length;
                    double k = Math.Min(hmin, kCRs[j] + (1 - kCRs[j]) * d / (gCR * r0));
                    if (kmin > k)
                        kmin = k;
                }
                hmin = Math.Min(hmin, kmin);
            }
            if (surfaceReferPoints != null)
            {
                double kmin = 1;
                for (int j = 0; j < surfaceReferPoints.Count; j++)
                {
                    Vector3d Separation = surfaceReferPoints[j] - p3d;
                    double d = Separation.Length;
                    double k = Math.Min(hmin, kSRs[j] + (1 - kSRs[j]) * d / (gSR * r0));
                    if (kmin > k)
                        kmin = k;
                }
                hmin = Math.Min(hmin, kmin);
            }
            return hmin;
        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            double[] Xcoord = new double[L];
            // List<double> ars=new List<double>(); 
            for (int i = 0; i < (PIndex.Length); i++)
            {

                double minD2 = 1000000;
                for (int j = 0; j < specialPoints.Count; j++)
                {
                    Vector3d Separation = specialPoints[j] - p[PIndex[i]].Position;
                    double d2 = Separation.SquareLength;
                    if (minD2 > d2)
                        minD2 = d2;
                }
                double d = Math.Sqrt(minD2);
                double ar = r0 * Math.Min(1, kSV + (1 - kSV) * d / (gSV * r0));
                ars[PIndex[i]] = ar;
            }
            for (int i = 0; i < L; i++)
            {
                Xcoord[i] = p[PIndex[i]].Position.X;
            }
            Array.Sort(Xcoord, PIndex);

            for (int i = 0; i < L; i++)
            {
                Move[i] = Vector3d.Zero;
                Weighting[i] = 0;
            }

            for (int i = 0; i < (PIndex.Length - 1); i++)
            {
                for (int k = i + 1; k < PIndex.Length; k++)
                {
                    double Diam = ars[PIndex[i]] + ars[PIndex[k]];
                    Vector3d Separation = p[PIndex[k]].Position - p[PIndex[i]].Position;
                    if (Separation.X < Diam)
                    {
                        if (Separation.SquareLength < Diam * Diam)
                        {
                            double LengthNow = Separation.Length;
                            double stretchfactor = 1.0 - Diam / LengthNow;
                            Vector3d SpringMove = 0.5 * Separation * stretchfactor;
                            Move[i] += SpringMove;
                            Move[k] -= SpringMove;
                            Weighting[i] = Strength;
                            Weighting[k] = Strength;
                        }
                    }

                    else { break; }
                }
            }
        }

    }
}
