using NF.Mathematics;

namespace NF.AI.PathFinding.Common
{
    public static class ExInt2
    {
        public static Int2 Foward(this Int2 x, EDirFlags dir)
        {
            return x + DirFlags.ToPos(dir);
        }

        public static Int2 Backward(this Int2 x, EDirFlags dir)
        {
            return x - DirFlags.ToPos(dir);
        }
    }
}
