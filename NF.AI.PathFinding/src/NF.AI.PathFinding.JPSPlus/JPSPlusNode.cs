using NF.AI.PathFinding.Common;
using NF.Mathematics;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusNode : AStarNode
    {
        private int[] mJumpDistances;

        public JPSPlusNode(in Int2 p, int[] jumpDistances) : base(p)
        {
            mJumpDistances = jumpDistances;
        }

        public int GetDistance(EDirFlags dir)
        {
            return mJumpDistances[DirFlags.ToArrayIndex(dir)];
        }

        internal void Refresh(int[] jumpDistances)
        {
            mJumpDistances = jumpDistances;
            Refresh();
        }
    }
}
