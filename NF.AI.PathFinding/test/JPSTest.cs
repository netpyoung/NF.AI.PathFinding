using Xunit;
using NF.AI.PathFinding.Common;
using NF.Mathematics;

namespace NF.AI.PathFinding.JPS.Test
{
    public class JPSTest
    {
        [Fact]
        public void TestJumpRecursive()
        {
            // . f .
            // . X j
            // . . .
            // p . g

            // p   : (0, 3)
            // g   : (2, 3)
            // dir : NORTHEAST
            // j   : (2, 1)

            JPS jps = new JPS(new bool[5, 4] {
                { false, false, false, false },
                { false, true, false, false },
                { false, false, false, false },
                { false, false, false, false },
                { false, false, false, false },
            });

            Int2 p = new Int2(0, 3);
            Int2 g = new Int2(2, 3);
            EDirFlags dir = EDirFlags.NORTHEAST;

            Int2 result = new Int2(0, 0);
            Assert.True(jps.TryJump(p, dir, g, ref result));
            Assert.Equal(new Int2(2, 1), result);
        }

        [Fact]
        public void TESTSuccesorsDir()
        {
            JPS jps = new JPS(new bool[5, 5] {
                { false, false, false, false,false },
                { false, false, false, false,false },
                { false, false, false, true,false },
                { false, false, false, false,false },
                { false, false, false, false,false },
            });

            Int2 o = new Int2(2, 2);

            // . . F
            // . o X
            // . . P
            EDirFlags forcedNeighbourDir = jps.ForcedNeighbourDir(o, EDirFlags.NORTHWEST);
            Assert.Equal(EDirFlags.NORTHEAST, forcedNeighbourDir);

            // N N .
            // N o X
            // . . P
            EDirFlags naturalNeighbours = JPS.NaturalNeighbours(EDirFlags.NORTHEAST);
            Assert.Equal(EDirFlags.NORTHEAST | EDirFlags.NORTH | EDirFlags.EAST, naturalNeighbours);

            // S S S
            // S o X
            // . . P
            EDirFlags succesorsDir = jps.SuccesorsDir(o, EDirFlags.NORTHWEST);
            Assert.Equal(EDirFlags.NORTHWEST | EDirFlags.NORTH | EDirFlags.NORTHEAST | EDirFlags.WEST, succesorsDir);
        }
    }
}