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
    public class TryKangarooComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public TryKangarooComponent()
          : base("TryKangaroo", "Nickname",
              "to using kangaroo in c# code",
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
            pManager.AddGenericParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            //1
            pManager.AddGenericParameter("TargetMesh", "TM", "吸附的曲面", GH_ParamAccess.item);
            //pManager.AddGenericParameter("Point", "P", "Points", GH_ParamAccess.list);
            //2
            pManager.AddCurveParameter("BoundaryCurves", "BC", "BoundaryCurves", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("TargetLength", "L", "A function determining local edge length", GH_ParamAccess.item);
            //3
            pManager.AddNumberParameter("PullStrengthM", "PM", "Strength of pull to target Mesh", GH_ParamAccess.item, 1);
            //pManager.AddLineParameter("Line", "L", "Lines", GH_ParamAccess.list);
            //4
            pManager.AddNumberParameter("PullStrengthC", "PC", "Strength of pull to Boundary Curves", GH_ParamAccess.item, 1);
            //5
            pManager.AddNumberParameter("SpringStrength", "S", "Strength of Springs of edges", GH_ParamAccess.item, 1);
            //6
            pManager.AddBooleanParameter("Reset", "R", "Reset", GH_ParamAccess.item);
            //7
            pManager.AddGenericParameter("FixedPoint", "P", "FixedPoint", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddTextParameter("Reverse", "R", "Reversed string", GH_ParamAccess.item);
            pManager.AddGenericParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Point", "P", "Point", GH_ParamAccess.list);
            pManager.AddIntegerParameter("iterations", "I", "迭代次数", GH_ParamAccess.item);

        }

        //private Mesh M = new Mesh();
        // List<Point3d> FV = new List<Point3d>();
        // List<Line> FL = new List<Line>();
        //int onMeshStrength = 100    ;
        // int restLength = 0;
        KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
        List<IGoal> Goals = new List<IGoal>();
        List<IGoal> GoalList = new List<IGoal>();
        Mesh M = new Mesh();
        Mesh TargetMesh = new Mesh();
        bool initialized;
        int counter = 0;
        double threshold = 1e-3;
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reset = false;
            List<Point3d> FixedPoint = new List<Point3d>();
            List<Curve> BoundaryCurves = new List<Curve>();
            double SpringStrength = 1, PullStrengthM = 1, PullStrengthC = 1;
            DA.GetData<Mesh>(0, ref M); //获取第0个输入值
            DA.GetData<Mesh>(1, ref TargetMesh); //获取第0个输入值
            //DA.GetDataList<Point3d>(1, FV);
            DA.GetDataList<Curve>(2, BoundaryCurves);
            //DA.GetDataList<Line>(1, FL);
            // DA.GetData<int>(2, ref restLength);
            DA.GetData<double>(3, ref PullStrengthM);
            DA.GetData<double>(4, ref PullStrengthC);
            DA.GetData<double>(5, ref SpringStrength);
            DA.GetData<bool>(6, ref reset);
            DA.GetDataList<Point3d>(7, FixedPoint);

            //内部计算过程start   

            //initialize the solver

            if (reset || initialized == false)
            {
                #region reset
                counter = 0;
                PS = new KangarooSolver.PhysicalSystem();
                Goals = new List<IGoal>();
                initialized = true;
                //get the Mesh points and boundary status
                //数组转列表
                List<Point3d> Pts = new List<Point3d>(M.Vertices.ToPoint3dArray());
                List<Point3d> NakedPts = new List<Point3d>();
                //var IndexList = Enumerable.Range(0, PS.ParticleCount()).ToList();
                List<int> Is = new List<int>();
                List<int> NakedIs = new List<int>();
                bool[] Naked = M.GetNakedEdgePointStatus();

                for (int i = 0; i < M.Vertices.Count; i++)
                {
                    PS.AddParticle(Pts[i], 1); //add a particle for every mesh vertex
                    Is.Add(i);
                    if (Naked[i])
                    //{ Goals.Add(new KangarooSolver.Goals.Anchor(i, Pts[i], 10000)); }// fix the boundaries strongly in place
                    {
                        NakedPts.Add(Pts[i]);
                        NakedIs.Add(i);
                    }
                }
                //Goals.Add(new KangarooSolver.Goals.OnCurve(NakedPts, BoundaryCurves[0], PullStrengthC));
                Goals.Add(new KangarooSolver.Goals.OnCurve(NakedIs, BoundaryCurves[0], PullStrengthC));
                //Goals.Add(new KangarooSolver.Goals.OnMesh(Pts, TargetMesh, 1));
                Goals.Add(new KangarooSolver.Goals.OnMesh(Is, TargetMesh, PullStrengthM));
                double sum_length = 0;
                for (int i = 0; i < M.TopologyEdges.Count; i++)
                {
                    Line line = M.TopologyEdges.EdgeLine(i);
                    double length = line.Length;
                    sum_length += length;
                }
                double average_length = sum_length / M.TopologyEdges.Count;
                for (int i = 0; i < M.TopologyEdges.Count; i++)
                {
                    var Ends = M.TopologyEdges.GetTopologyVertices(i);

                    int Start = M.TopologyVertices.MeshVertexIndices(Ends.I)[0];
                    int End = M.TopologyVertices.MeshVertexIndices(Ends.J)[0];
                    //for each edge, a spring with rest length average_length, and strength 1
                    //Goals.Add(new KangarooSolver.Goals.Spring(Pts[Start], Pts[End], average_length, 1));
                    Goals.Add(new KangarooSolver.Goals.Spring(Start, End, average_length, SpringStrength));
                }
                for (int j = 0; j < FixedPoint.Count; j++)
                {

                    int index = PS.FindParticleIndex(FixedPoint[j], threshold, true);
                    Goals.Add(new KangarooSolver.Goals.Anchor(index, FixedPoint[j], 10000));
                }// fix the boundaries strongly in place
                 // foreach (IGoal G in Goals)
                 //  {
                 //     PS.AssignPIndex(G, 0.01);
                 // }
                 //GH_ObjectWrapper ow=new GH_ObjectWrapper(M);
                 //var gl = new KangarooSolver.Goals.Locator(ow);
                 // Goals.Add(gl);
                 //foreach (IGoal G in Goals) //Assign indexes to the particles in each Goal:
                 //{
                 //    PS.AssignPIndex(G, 0.0001); // the second argument is the tolerance distance below which points combine into a single particle
                 //    GoalList.Add(G);
                 //}
                #endregion
            }
            else
            {
                /*
                else
                {
                    int count = Springs.Count;

                    for (int i = 0; i < count; i++)
                    {
                        Spring S = Springs[i] as Spring;
                        Point3d Start = PS.GetPosition(S.PIndex[0]);
                        Point3d End = PS.GetPosition(S.PIndex[1]);

                        double LineLength = Start.DistanceTo(End);
                        if (LineLength > splitLength)
                        {
                            Point3d MidPt = 0.5 * (Start + End);
                            PS.AddParticle(MidPt, 1);
                            int newParticleIndex = PS.ParticleCount() - 1;
                            int endIndex = S.PIndex[1];
                            //set the end of the original spring to be the newly created midpoint
                            //and make a new spring from the midpoint to the original endpoint
                            S.PIndex[1] = newParticleIndex;
                            S.RestLength = 0.5 * LineLength;
                            Spring otherHalf = new Spring(newParticleIndex, endIndex, 0.5 * LineLength, 1);
                            Springs[i] = S;
                            Springs.Add(otherHalf);
                        }
                        else
                        {
                            S.RestLength += increment;
                            Springs[i] = S;
                        }
                    }
                    */


                //Step forward, using these goals, with multi-threading on, and stopping if the threshold is reached
                if (counter == 0 || (PS.GetvSum() > threshold && counter < 100))
                {
                    PS.Step(Goals, true, threshold);
                    double sum = PS.GetvSum();
                    counter++;
                }
                //内部计算过程end   
                //Output the mesh, and how many iterations it took to converge
                //object Out = PS.GetOutput(GoalList);
                //DA.SetData(0, Out);
                M.Vertices.Clear();
                M.Vertices.AddVertices(PS.GetPositions());
            }
            DA.SetData(0, M);
            List<Point3d> pout = new List<Point3d>();
            pout = PS.GetPositions().ToList();
            DA.SetDataList(1, pout); //输出第一个输出值
            DA.SetData(2, counter);
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
            get { return new Guid("0d31e28b-439b-4df9-a8d6-2e63d8033745"); }
        }
    }
}
