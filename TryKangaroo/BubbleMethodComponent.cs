using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Collections;

using GH_IO;
using GH_IO.Serialization;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System;
using System.IO;
//using System.Xml;
//using System.Xml.Linq;
using System.Linq;
//using System.Data;
using System.Drawing;
using System.Reflection;
using System.Collections;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Runtime.InteropServices;


using KangarooSolver;
using KangarooSolver.Goals;

namespace TryKangaroo
{
    public class BubbleMethodComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public BubbleMethodComponent()
          : base("BubbleMethod", "BubbleMethod",
              "to using BubbleMethod in c# code",
              "ZD-MESH", "C#")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // pManager.AddTextParameter("String", "S", "String to reverse", GH_ParamAccess.item);
            //0
            pManager.AddGenericParameter("TargetMesh", "TM", "吸附的曲面", GH_ParamAccess.item);
            //1
            pManager.AddGenericParameter("Points", "Pts", "Pts", GH_ParamAccess.list);
            //2
            pManager.AddNumberParameter("PullStrengthM", "PM", "Strength of pull to target Mesh", GH_ParamAccess.item, 1);
            pManager[2].Optional = true;
            //3
            pManager.AddNumberParameter("CollideStrength", "C", "Strength of Collide", GH_ParamAccess.item, 1);
            pManager[3].Optional = true;
            //4
            pManager.AddNumberParameter("radius", "r", "basic radius", GH_ParamAccess.item);
            //5
            pManager.AddNumberParameter("zankong", "r", "zanwu", GH_ParamAccess.item); pManager[5].Optional = true;
            //6
            pManager.AddNumberParameter("threshold", "T", "threshold", GH_ParamAccess.item, 0.0001);
            pManager[6].Optional = true;
            //7
            pManager.AddNumberParameter("subIteration", "si", "subIteration", GH_ParamAccess.item, 10);
            pManager[7].Optional = true;
            //8
            pManager.AddNumberParameter("maxIteration", "mi", "maxIteration", GH_ParamAccess.item, 100);
            pManager[8].Optional = true;
            //9
            pManager.AddBooleanParameter("Reset", "R", "Reset", GH_ParamAccess.item,true);
            //10
            pManager.AddGenericParameter("SpecialPoints", "SP", "Special Points", GH_ParamAccess.list);
            pManager[10].Optional = true;
            //11
            pManager.AddNumberParameter("k_SpecialPoint", "ksp", "kp", GH_ParamAccess.item, 1);
            pManager[11].Optional = true;
            //12
            pManager.AddNumberParameter("g_SpecialPoint", "gsp", "gp", GH_ParamAccess.item, 1);
            pManager[12].Optional = true; 
            //13
            pManager.AddGenericParameter("SpecialCurves", "SC", "Special Curves", GH_ParamAccess.list);
            pManager[13].Optional = true;
            //14
            pManager.AddNumberParameter("k_SpecialCurve", "ksc", "ksc", GH_ParamAccess.item, 1);
            pManager[14].Optional = true;
            //15
            pManager.AddNumberParameter("g_SpecialCurve", "gsc", "gsc", GH_ParamAccess.item, 1);
            pManager[15].Optional = true;
            //16
            pManager.AddGenericParameter("ReferCurves", "RC", "Refer Curves", GH_ParamAccess.list);
            pManager[16].Optional = true;
            //17
            pManager.AddGenericParameter("ReferCurvePoints", "RCP", "Refer Curve Points", GH_ParamAccess.list);
            pManager[17].Optional = true;
            //18
            pManager.AddNumberParameter("k_ReferCurve", "krc", "k_ReferCurve", GH_ParamAccess.item, 1);
            pManager[18].Optional = true;
            //19
            pManager.AddNumberParameter("g_ReferCurve", "grc", "g_ReferCurve", GH_ParamAccess.item, 1);
            pManager[19].Optional = true;
            //20
            pManager.AddGenericParameter("ReferSurfaces", "RS", "Refer Surface", GH_ParamAccess.list);
            pManager[20].Optional = true;
            //21
            pManager.AddGenericParameter("ReferSurfacePoints", "RSP", "Refer Surface Points", GH_ParamAccess.list);
            pManager[21].Optional = true;
            //22
            pManager.AddNumberParameter("k_ReferSurface", "krs", "k_ReferSurface", GH_ParamAccess.item, 1);
            pManager[22].Optional = true;
            //23
            pManager.AddNumberParameter("g_ReferSurface", "grs", "k_ReferSurface", GH_ParamAccess.item, 1);
            pManager[23].Optional = true; 
        }
        //public SphereCollide_wqs(List<Point3d> V, double r, double w,
        //    List<Point3d> SV = null, double kP = 1, double gP = 1,
        //     List<Curve> SC = null, double kL = 1, double gL = 1,
        //      Curve rCurve = null, List<Point3d> CRP = null, double kCC = 1, double gCC = 1,
        //      Surface rSurface = null, List<Point3d> SRP = null, double kCS = 1, double gCS = 1)
        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddTextParameter("Reverse", "R", "Reversed string", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "Point", GH_ParamAccess.list);
            pManager.AddIntegerParameter("iteration", "I", "迭代次数", GH_ParamAccess.item);
            pManager.AddNumberParameter("adjustedRadius", "AR", "各气泡半径", GH_ParamAccess.list);
            pManager.AddPointParameter("bubbleCenters", "BC", "各气泡中心", GH_ParamAccess.list);

        }

        //private Mesh M = new Mesh();
        // List<Point3d> FV = new List<Point3d>();
        // List<Line> FL = new List<Line>();
        //int onMeshStrength = 100    ;
        // int restLength = 0;
        KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
        List<IGoal> Goals = new List<IGoal>();
        List<IGoal> GoalList = new List<IGoal>();
        bool initialized;
        int counter = 0;
        double threshold = 1e-3, subIteration=10, maxIteration=100;
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh TargetMesh = new Mesh();
            List<Point3d> Points = new List<Point3d>();
            double  PullStrengthM = 1, CollideStrength=1;
            double radius = 1;
            bool reset = false;
            double k_SpecialPoint=1, g_SpecialPoint=1, k_SpecialCurve=1, g_SpecialCurve=1,
                k_ReferCurve=1, g_ReferCurve=1, k_ReferSurface=1, g_ReferSurface=1;
            List<Point3d> SpecialPoints = new List<Point3d>();
            List<Curve> SpecialCurves = new List<Curve>();
            List<Curve> ReferCurves = new List<Curve>();
            List<Point3d> ReferCurvePoints = new List<Point3d>();
            List<Surface> ReferSurfaces = new List<Surface>();
            List<Point3d> ReferSurfacePoints = new List<Point3d>();
            DA.GetData<Mesh>(0, ref TargetMesh); //获取第0个输入值
            DA.GetDataList<Point3d>(1, Points);
            DA.GetData<double>(2, ref PullStrengthM);
            DA.GetData<double>(3, ref CollideStrength);
            DA.GetData<double>(4, ref radius);
            DA.GetData<double>(6, ref threshold);
            DA.GetData<double>(7, ref subIteration);
            DA.GetData<double>(8, ref maxIteration);
            DA.GetData<bool>(9, ref reset);
            DA.GetDataList<Point3d>(10, SpecialPoints);
            DA.GetData<double>(11, ref k_SpecialPoint);
            DA.GetData<double>(12, ref g_SpecialPoint);
            DA.GetDataList<Curve >(13, SpecialCurves);
            DA.GetData<double>(14, ref k_SpecialCurve);
            DA.GetData<double>(15, ref g_SpecialCurve);
            DA.GetDataList<Curve>(16, ReferCurves);
            DA.GetDataList<Point3d>(17, ReferCurvePoints);
            DA.GetData<double>(18, ref k_ReferCurve);
            DA.GetData<double>(19, ref g_ReferCurve);
            DA.GetDataList<Surface >(20, ReferSurfaces);
            DA.GetDataList<Point3d>(21, ReferSurfacePoints);
            DA.GetData<double>(22, ref k_ReferSurface);
            DA.GetData<double>(23, ref g_ReferSurface);
            //内部计算过程start   

            //initialize the solver

            if (reset || initialized == false)
            {
                #region reset
                counter = 0;
                PS = new KangarooSolver.PhysicalSystem();
                Goals = new List<IGoal>();
                initialized = true;
                //var IndexList = Enumerable.Range(0, PS.ParticleCount()).ToList();
                //Goals.Add(new KangarooSolver.Goals.OnCurve(NakedPts, BoundaryCurves[0], PullStrengthC));
                //Goals.Add(new KangarooSolver.Goals.OnMesh(Pts, TargetMesh, 1));
                Goals.Add(new KangarooSolver.Goals.OnMesh(Points, TargetMesh, PullStrengthM));
                Goals.Add(new KangarooSolver.Goals.SphereCollide_wqs(Points, radius, CollideStrength,
                    SpecialPoints, k_SpecialPoint, g_SpecialPoint,
                    SpecialCurves, k_SpecialCurve, g_SpecialCurve,
                    ReferCurves, ReferCurvePoints, k_ReferCurve, g_ReferCurve,
                    ReferSurfaces, ReferSurfacePoints, k_ReferSurface, g_ReferSurface
                    ));
                 //public SphereCollide_wqs(List<Point3d> V, List<Point3d> SV, double radius, double k, double kSV0, double gSV0)
                foreach (IGoal G in Goals) //Assign indexes to the particles in each Goal:
                {
                    PS.AssignPIndex(G, 0.0001); // the second argument is the tolerance distance below which points combine into a single particle
                    GoalList.Add(G);
                }
                #endregion
            }
            else
            {
                //Step forward, using these goals, with multi-threading on, and stopping if the threshold is reached
                if (counter == 0 || (PS.GetvSum() > threshold && counter < 100))
                {
                    for (int i = 0; i < subIteration; i += 1)
                    {
                        PS.Step(Goals, true, threshold);
                        counter++;
                    }
                    double sum = PS.GetvSum();
                }
                //内部计算过程end   
                //Output the mesh, and how many iterations it took to converge
            }
            List<Point3d> pout = new List<Point3d>();
            pout = PS.GetPositions().ToList();
            DA.SetDataList(0, pout); //输出第一个输出值
            DA.SetData(1, counter);
            List<double> ars = SphereCollide_wqs.ars;
            DA.SetDataList(2, ars); //输出第一个输出值
            List<Point3d> pp = Goals[1].PPos.ToList();
            DA.SetDataList(3, pout); //输出第一个输出值


        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                return Properties.Resources.tryKangaroo;
                //return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("0d31e28b-439b-4df9-a8d6-2e63d8033746"); }
        }
    }
}
