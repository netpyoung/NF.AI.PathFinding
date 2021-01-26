using NF.Mathematics;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusBakedMap
    {
        public class JPSPlusBakedMapBlock
        {
            public readonly int[] JumpDistances; // for 8 direction distance;
            public readonly Int2 Pos;

            public JPSPlusBakedMapBlock(in Int2 pos, int[] jumpDistances)
            {
                JumpDistances = jumpDistances;
                Pos = pos;
            }
        }

        public readonly int[,] BlockLUT;
        public readonly JPSPlusBakedMapBlock[] Blocks;
        public int Width => BlockLUT.GetLength(1);
        public int Height => BlockLUT.GetLength(0);

        public JPSPlusBakedMap(int[,] blockLUT, JPSPlusBakedMapBlock[] blocks)
        {
            this.BlockLUT = blockLUT;
            this.Blocks = blocks;
        }
    }
}
