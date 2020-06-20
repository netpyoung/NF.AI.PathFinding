using NF.Mathematics;
using System;
using System.Collections.Generic;

namespace NF.AI.PathFinding.Common
{
    public class BresenHamPathSmoother
    {
        BresenHam mBresenHam = new BresenHam();

        public List<Int2> SmoothPath(List<Int2> path, Func<Int2, bool> fnIsWalkable)
        {
            var ret = new List<Int2>();
            if (path.Count < 2)
            {
                return ret;
            }

            Int2 bp = new Int2();
            Int2 prevTargetP = new Int2();

            var beginP = path[0];
            var targetP = path[1];
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