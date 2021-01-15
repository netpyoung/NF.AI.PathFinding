using NF.Mathematics;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusBakedMap
    {
        public JPSPlusBakedMap(int[,] blockLUT, JPSPlusBakedMapBlock[] blocks)
        {
            this.BlockLUT = blockLUT;
            this.Blocks = blocks;
        }

        public class JPSPlusBakedMapBlock
        {
            public JPSPlusBakedMapBlock(in Int2 pos, int[] jumpDistances)
            {
                JumpDistances = jumpDistances;
                Pos = pos;
            }

            public int[] JumpDistances; // for 8 direction distance;
            public Int2 Pos;
        }

        public int Width => BlockLUT.GetLength(1);
        public int Height => BlockLUT.GetLength(0);
        public int[,] BlockLUT;
        public JPSPlusBakedMapBlock[] Blocks;
    }
}
