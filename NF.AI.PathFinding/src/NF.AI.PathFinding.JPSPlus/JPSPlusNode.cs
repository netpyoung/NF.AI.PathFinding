using NF.AI.PathFinding.Common;
using NF.Mathematics;
using System;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusNode : AStarNode
    {
        int[] mJumpDistances;

        public JPSPlusNode(in Int2 p, int[] jumpDistances) : base(p)
        {
            this.mJumpDistances = jumpDistances;
        }

        public int GetDistance(EDirFlags dir)
        {
            return mJumpDistances[DirFlags.ToArrayIndex(dir)];
        }

        internal void Refresh(int[] jumpDistances)
        {
            mJumpDistances = jumpDistances;
            base.Refresh();
        }
    }
}
