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
    public class SpringMethodComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public SpringMethodComponent()
          : base("SpringMethod", "SpringMethod",
              "to using Spring Method in c# code",
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
            pManager.AddNumberParameter("SpringStiffness", "SS", "Spring Stiffness", GH_ParamAccess.item, 1);
            pManager[3].Optional = true;
            //4
            pManager.AddIntegerParameter("Row", "r", "Row of Points", GH_ParamAccess.item);
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
        List<List<Point3d>> grids;
        List<int> indexs = new List<int>();
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double threshold = 1e-3, subIteration = 10, maxIteration = 100;
            Mesh TargetMesh = new Mesh();
            List<Point3d> Points = new List<Point3d>();
            double  PullStrengthM = 1, SpringStiffness = 1;
            int row = 1,column=1;
            bool reset = false;
            DA.GetData<Mesh>(0, ref TargetMesh); //获取第0个输入值
            DA.GetDataList<Point3d>(1, Points);
            DA.GetData<double>(2, ref PullStrengthM);
            DA.GetData<double>(3, ref SpringStiffness);
            DA.GetData<int>(4, ref row);
            column = Points.Count / row;
            DA.GetData<double>(6, ref threshold);
            DA.GetData<double>(7, ref subIteration);
            DA.GetData<double>(8, ref maxIteration);
            DA.GetData<bool>(9, ref reset);
            //内部计算过程start   
            //initialize the solver

            if (reset || initialized == false)
            {
                counter = 0;
                PS = new KangarooSolver.PhysicalSystem();
                Goals = new List<IGoal>();
                initialized = true;
                grids = new List<List<Point3d>>();
                indexs = new List<int>();
                for (int i = 0; i < row; i++)
                {
                    List<Point3d> ps = new List<Point3d>();
                    for (int j = 0; j < column; j++)
                    {
                        ps.Add(Points[(i * row + j)]);
                        PS.AddParticle(Points[i * row + j], 1);
                        indexs.Add(i * row + j);
                    }
                    grids.Add(ps);
                }
            }
            else
            {
                //var IndexList = Enumerable.Range(0, PS.ParticleCount()).ToList();
                //Goals.Add(new KangarooSolver.Goals.OnCurve(NakedPts, BoundaryCurves[0], PullStrengthC));
                //Goals.Add(new KangarooSolver.Goals.OnMesh(Pts, TargetMesh, 1));
                Goals.Add(new KangarooSolver.Goals.OnMesh(indexs, TargetMesh, PullStrengthM));
                List<double> length_row = new List<double>();
                List<double> length_column = new List<double>();
                List<List<double>> gridLengths = new List<List<double>>();
                //计算各条多线段的长度
                for (int i = 0; i < row; i++)
                {
                    double sumLength = 0;
                    List<double> lengths = new List<double>();
                    for (int j = 0; j < column-1; j++)
                    {
                        double length = grids[i][j].DistanceTo(grids[i][j + 1]);
                        lengths.Add(length);
                        sumLength += length;
                    }
                    gridLengths.Add(lengths);
                    length_row.Add(sumLength);
                }
                List<double> length_row2 = length_row.GetRange(1, row-2);
                double length_row_sum=(length_row.Sum()+ length_row2.Sum())/2;
                for (int j = 0; j < column; j++)
                {
                    double sumLength = 0;
                    for (int i = 0; i < row - 1; i++)
                    {
                        double length = grids[i][j].DistanceTo(grids[i+1][j]);
                        sumLength += length;
                    }
                    length_column.Add(sumLength);
                }
                List<double> length_column2 = length_column.GetRange(1, column - 2);
                double length_column_sum = (length_column.Sum() + length_column2.Sum()) / 2;
                //弹簧
                for (int i = 0; i < row; i++)
                {
                    for (int j = 0; j < column - 1; j++)
                    {
                        double relativeLength = (length_column[j] + length_column[j + 1]) / 2 / length_column_sum;
                        Goals.Add(new KangarooSolver.Goals.Spring(i * row + j, i * row + j+1, relativeLength* length_row[i], SpringStiffness));
                    }
                }
                //弹簧
                    for (int j = 0; j < column ; j++)
                {
                    for (int i = 0; i < row- 1; i++)
                    {
                        double relativeLength = (length_row[j] + length_row[j + 1]) / 2/length_row_sum;
                        Goals.Add(new KangarooSolver.Goals.Spring((i + 1)* row + j, i * row + j , relativeLength * length_column[j], SpringStiffness));
                    }
                }
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
            //List<double> ars = SphereCollide_wqs.ars;
            //DA.SetDataList(2, ars); //输出第一个输出值
            //List<Point3d> pp = Goals[1].PPos.ToList();
           // DA.SetDataList(3, pout); //输出第一个输出值


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
            get { return new Guid("0d31e28b-439b-4df9-a8d6-2e63d8033750 "); }
        }
    }
}
