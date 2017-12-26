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
            pManager.AddGenericParameter("Point", "P", "Point", GH_ParamAccess.list);
            //2
            pManager.AddIntegerParameter("TargetLengthFunction", "L", "A function determining local edge length", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddTextParameter("Reverse", "R", "Reversed string", GH_ParamAccess.item);
          //  pManager.AddGenericParameter("Mesh", "A", "Mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Point", "P", "Point", GH_ParamAccess.list);
     
        }

        //private Mesh M = new Mesh();
       // List<Point3d> FV = new List<Point3d>();
       // List<Line> FL = new List<Line>();
        int onMeshStrength = 100;
        int restLength = 0;
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh M = new Mesh();
            List<Point3d> FV = new List<Point3d>();
            DA.GetData<Mesh>(0, ref M); //获取第0个输入值
            DA.GetDataList<Point3d>(1, FV);
            //DA.GetDataList<Line>(1, FL);
            DA.GetData<int>(2, ref restLength);
            //内部计算过程start   

            //initialize the solver
            var PS = new KangarooSolver.PhysicalSystem();
            var Goals = new List<IGoal>();

            for (int i = 0; i < FV.Count; i++)
            {
                PS.AddParticle(FV[i], 1); //add a particle for every mesh vertex
            }
            //Goals.Add(new KangarooSolver.Goals.Anchor(i, Pts[i], 10000)); 
            /*
            for (int i = 0; i < FV.Count; i++)
            {
                for (int j = i+1; j < FV.Count; j++)
                {
                   // Goals.Add(new KangarooSolver.Goals.Spring(i, j, restLength, 100)); //for each edge, a spring with rest length 0, and strength 1
                    Goals.Add(new KangarooSolver.Goals.ClampLength(i, j, 10000, restLength, 100));
                }     
            }
            */
            Goals.Add(new KangarooSolver.Goals.OnMesh(FV, M, onMeshStrength));
            foreach (IGoal G in Goals)
            {
                PS.AssignPIndex(G, 0.01);
            }
            int counter = 0;
            double threshold = 1e-6;
            do
            {
                //Step forward, using these goals, with multi-threading on, and stopping if the threshold is reached
                PS.Step(Goals, true, threshold);
                counter++;
            } while (PS.GetvSum() > threshold && counter < 100);
            //GetvSum returns the current kinetic energy
            //always include a counter to prevent it getting stuck forever in case it cannot reach the given threshold

            //replace the mesh vertices with the relaxed ones
            //内部计算过程end   

            //Output the mesh, and how many iterations it took to converge
            List<Point3d> pout = new List<Point3d>();
            pout = PS.GetPositions().ToList();
            DA.SetDataList(0, pout); //输出第一个输出值
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
