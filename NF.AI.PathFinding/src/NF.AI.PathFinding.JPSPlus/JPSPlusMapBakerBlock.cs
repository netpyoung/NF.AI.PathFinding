using NF.AI.PathFinding.Common;
using NF.Mathematics;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusMapBakerBlock
    {
        public bool IsWall;
        public EDirFlags JumpDirFlags;
        public int[] JumpDistances = new int[8];
        public Int2 Pos;

        public JPSPlusMapBakerBlock(in Int2 pos)
        {
            Pos = pos;
        }

        public bool IsJumpable(EDirFlags dir)
        {
            return (JumpDirFlags & dir) == dir;
        }

        public void SetDistance(EDirFlags dir, int distance)
        {
            JumpDistances[DirFlags.ToArrayIndex(dir)] = distance;
        }

        public int GetDistance(EDirFlags dir)
        {
            return JumpDistances[DirFlags.ToArrayIndex(dir)];
        }
    }
}
