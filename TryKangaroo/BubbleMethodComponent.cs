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
            pManager.AddGenericParameter("Point", "P", "Point", GH_ParamAccess.list);
            //2
            pManager.AddGenericParameter("SpecialPoint", "SP", "Special Points", GH_ParamAccess.list);
            //3
            pManager.AddNumberParameter("radius", "r", "basic radius", GH_ParamAccess.item);
            //4
            pManager.AddBooleanParameter("Reset", "R", "Reset", GH_ParamAccess.item);
            pManager[4].Optional = true;
            //5
            pManager.AddNumberParameter("PullStrengthM", "PM", "Strength of pull to target Mesh", GH_ParamAccess.item, 1);
            pManager[5].Optional = true;
            //6
            pManager.AddNumberParameter("threshold", "T", "threshold", GH_ParamAccess.item, 1);
            pManager[6].Optional = true;
            //7
            pManager.AddNumberParameter("k_SpecialPoint", "k", "k", GH_ParamAccess.item, 1);
            pManager[7].Optional = true;
            //8
            pManager.AddNumberParameter("g_SpecialPoint", "g", "g", GH_ParamAccess.item, 1);
            pManager[8].Optional = true;
            //9
            pManager.AddNumberParameter("CollideStrength", "C", "Strength of Collide", GH_ParamAccess.item, 1);
            pManager[5].Optional = true;

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddTextParameter("Reverse", "R", "Reversed string", GH_ParamAccess.item);
            pManager.AddPointParameter("Point", "P", "Point", GH_ParamAccess.list);
            pManager.AddIntegerParameter("iterations", "I", "迭代次数", GH_ParamAccess.item);
            pManager.AddNumberParameter("adjustedRadio", "AR", "各气泡半径", GH_ParamAccess.list);
            pManager.AddPointParameter("bubbleCenter", "BC", "各气泡中心", GH_ParamAccess.list);

        }

        //private Mesh M = new Mesh();
        // List<Point3d> FV = new List<Point3d>();
        // List<Line> FL = new List<Line>();
        //int onMeshStrength = 100    ;
        // int restLength = 0;
        KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
        List<IGoal> Goals = new List<IGoal>();
        List<IGoal> GoalList = new List<IGoal>();
        Mesh TargetMesh = new Mesh();
        bool initialized;
        int counter = 0;
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reset = false;
            double threshold = 1e-3;
            double r =1, k_SpecialPoint=1, g_SpecialPoint=2;
            List<Point3d> Pts = new List<Point3d>();
            List<Point3d> SPts = new List<Point3d>();
            List<Curve> BoundaryCurves = new List<Curve>();
            double  PullStrengthM = 1, CollideStrength=1;
            DA.GetData<Mesh>(0, ref TargetMesh); //获取第0个输入值
            DA.GetDataList<Point3d>(1, Pts);
            DA.GetDataList<Point3d>(2, SPts);
            DA.GetData<double>(3, ref r);
            DA.GetData<bool>(4, ref reset);
            DA.GetData<double>(5, ref PullStrengthM);
            DA.GetData<double>(6, ref threshold);
            DA.GetData<double>(7, ref k_SpecialPoint);
            DA.GetData<double>(8, ref g_SpecialPoint);
            DA.GetData<double>(9, ref CollideStrength);

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
                Goals.Add(new KangarooSolver.Goals.OnMesh(Pts, TargetMesh, PullStrengthM));
                Goals.Add(new KangarooSolver.Goals.SphereCollide_wqs(Pts,SPts,r, CollideStrength, k_SpecialPoint, g_SpecialPoint));
                 //public SphereCollide_wqs(List<Point3d> V, List<Point3d> SV, double r, double k, double kSV0, double gSV0)
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
                    PS.Step(Goals, true, threshold);
                    double sum = PS.GetvSum();
                    counter++;
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
