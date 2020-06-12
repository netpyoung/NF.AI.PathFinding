using NF.Mathematics;
namespace NF.AI.PathFinding.Common
{
    public class AStarNode
    {
        public AStarNode(Int2 p)
        {
            this.G = 0;
            this.H = 0;
            this.Position = p;
        }

        public AStarNode(int x, int y) :
            this(new Int2 { X = x, Y = y })
        {
        }

        public void Refresh()
        {
            this.G = 0;
            this.H = 0;
            this.Parent = null;
        }

        public Int2 Position { get; private set; }
        public int G { get; internal set; } = 0;
        public int H { get; internal set; } = 0;
        public long F => G + H;

        public AStarNode Parent { get; internal set; }
    }
}
