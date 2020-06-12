using NF.Mathematics;
using NF.AI.PathFinding.Common;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusNode
    {
        public JPSPlusNode(Int2 p, int[] jumpDistances)
        {
            this.G = 0;
            this.H = 0;
            this.Position = p;
            this.JumpDistances = jumpDistances;
        }

        public void Refresh()
        {
            this.G = 0;
            this.H = 0;
            this.Parent = null;
        }

        public Int2 Position { get; private set; }
        public int[] JumpDistances;

        public int G { get; internal set; } = 0;
        public int H { get; internal set; } = 0;
        public long F => G + H;

        public JPSPlusNode Parent { get; internal set; }

        internal int GetDistance(EDirFlags dir)
        {
            return JumpDistances[DirFlags.ToArrayIndex(dir)];
        }
    }
}
