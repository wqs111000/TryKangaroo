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
    public class SpringNet_wqs: GoalObject
    {
        const double maximumDistance = 0.001, clb_basic=0.01;//点是否在曲线或者曲面上，曲率下限计算参考值
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
        public List<Curve> referCurves;
        public double kCR = 1;
        public List<double> kCRs;
        public double gCR = 1;
        public List<Surface> referSurfaces;
        public List<Point3d> surfaceReferPoints;
        public double kSR = 1;
        public List<double> kSRs;
        public double gSR = 1;
        static public List<double> ars;
        //控制气泡大小和特定点的距离相关
        public SpringNet_wqs(List<Point3d> V, double row, double w)
        {
            int L = V.Count;
            PPos = V.ToArray();
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int i = 0; i < L; i++)
            {
                Weighting[i] = w;
            }

        }
        private double RadiusFactor(Point3d p3d)
        {
            double hmin = 1;
            if (specialPoints != null && specialPoints.Count != 0)
            {
                double hmin_sp = 1;
                double minD = gSC * r0;
                for (int j = 0; j < specialPoints.Count; j++)
                {
                    Vector3d Separation = specialPoints[j] - p3d;
                    double d = Separation.Length;
                    if (minD > d)
                        minD = d;
                }
                hmin_sp = kSV + (1 - kSV) * minD / (gSV * r0);
                hmin = Math.Min(hmin, hmin_sp);
            }
            if (specialCurves != null && specialCurves.Count != 0)
            {
                double hmin_sc = 1;
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
                hmin_sc = kSC + (1 - kSC) * minD / (gSC * r0);
                hmin = Math.Min(hmin, hmin_sc);
            }
            if (curveReferPoints != null && curveReferPoints.Count != 0)
            {
                double hmin_cr = 1;
                for (int j = 0; j < curveReferPoints.Count; j++)
                {
                    Vector3d Separation = curveReferPoints[j] - p3d;
                    double d = Separation.Length;
                    double k = Math.Min(hmin, kCRs[j] + (1 - kCRs[j]) * d / (gCR * r0));
                    if (hmin_cr > k)
                        hmin_cr = k;
                }
                hmin = Math.Min(hmin, hmin_cr);
            }
            if (surfaceReferPoints != null)
            {
                double hmin_sr = 1;
                for (int j = 0; j < surfaceReferPoints.Count; j++)
                {
                    Vector3d Separation = surfaceReferPoints[j] - p3d;
                    double d = Separation.Length;
                    double k = Math.Min(hmin, kSRs[j] + (1 - kSRs[j]) * d / (gSR * r0));
                    if (hmin_sr > k)
                        hmin_sr = k;
                }
                hmin = Math.Min(hmin, hmin_sr);
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
                if (i == 47)
                {
                    int aaa = 333;
                }
                double hmin = RadiusFactor(p[PIndex[i]].Position);
                double ar = r0 * hmin;
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
