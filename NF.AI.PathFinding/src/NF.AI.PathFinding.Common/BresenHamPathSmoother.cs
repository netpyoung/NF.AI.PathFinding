using NF.Mathematics;

using System.Collections.Generic;

namespace NF.AI.PathFinding.Common
{
    public class BresenHamPathSmoother
    {
        private readonly BresenHam mBresenHam = new BresenHam();

        public delegate bool DelIsWalkable(in Int2 p);

        public List<Int2> SmoothPath(List<Int2> path, DelIsWalkable fnIsWalkable)
        {
            List<Int2> ret = new List<Int2>();
            if (path.Count < 2)
            {
                return ret;
            }

            Int2 bp = new Int2();
            Int2 prevTargetP = new Int2();

            Int2 beginP = path[0];
            Int2 targetP = path[1];
            int index = 1;
            mBresenHam.Init(beginP, targetP);
            ret.Add(beginP);

            while (true)
            {
                if (!mBresenHam.TryGetNext(ref bp))
                {
                    prevTargetP = targetP;
                    index++;
                    if (index == path.Count)
                    {
                        ret.Add(targetP);
                        break;
                    }
                    targetP = path[index];
                    mBresenHam.Init(beginP, targetP);
                }
                else if (!fnIsWalkable(bp))
                {
                    ret.Add(prevTargetP);
                    beginP = prevTargetP;
                    mBresenHam.Init(beginP, targetP);
                }
            }
            return ret;
        }
    }
}
