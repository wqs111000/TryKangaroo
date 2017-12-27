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
        static public List<double> ars ;
        double kSV = 0.5;
        double gSV = 4;
        //控制气泡大小和特定点的距离相关
        public SphereCollide_wqs(List<Point3d> V, List<Point3d> SV,double r, double w,double k,double g)
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
            for (int i=0; i < V.Count; i += 1)
                ars.Add(r0);
            //Diam = r + r;
            // SqDiam = Diam * Diam;
            Strength = k;
            specialPoints = SV;
            kSV = k;
            gSV = g;
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
                ars[PIndex[i]]=ar;
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
                for (int k = i+1; k < PIndex.Length; k++)
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
