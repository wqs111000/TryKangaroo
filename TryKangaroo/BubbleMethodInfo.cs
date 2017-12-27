using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace TryKangaroo
{
    public class BubbleMethodInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "BubbleMethod";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "    ";


            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("0e336084-e010-4fd1-9b4f-f710cb079c06");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "Microsoft";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
