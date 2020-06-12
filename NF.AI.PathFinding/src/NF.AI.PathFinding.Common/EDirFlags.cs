using NF.Mathematics;
using System.Collections.Generic;

namespace NF.AI.PathFinding.Common
{
    public enum EDirFlags : int
    {
        NONE = 0,
        NORTH = 1,
        SOUTH = 2,
        EAST = 4,
        WEST = 8,
        NORTHEAST = 16,
        NORTHWEST = 32,
        SOUTHEAST = 64,
        SOUTHWEST = 128,
        ALL = NORTH | SOUTH | EAST | WEST | NORTHEAST | NORTHWEST | SOUTHEAST | SOUTHWEST,
    };

    public static class DirFlags
    {
        public static int ToArrayIndex(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTHWEST: return 0;
                case EDirFlags.NORTH: return 1;
                case EDirFlags.NORTHEAST: return 2;
                case EDirFlags.WEST: return 3;
                case EDirFlags.EAST: return 4;
                case EDirFlags.SOUTHWEST: return 5;
                case EDirFlags.SOUTH: return 6;
                case EDirFlags.SOUTHEAST: return 7;
                default: return -1;
            }
        }
    }

    public static class ExInt2
    {
        public static Dictionary<EDirFlags, Int2> Dic = new Dictionary<EDirFlags, Int2> {
                { EDirFlags.NORTH, new Int2(0, -1) },
                { EDirFlags.SOUTH, new Int2(0, 1) },
                { EDirFlags.EAST, new Int2(1, 0) },
                { EDirFlags.WEST, new Int2(-1, 0) },
                { EDirFlags.NORTHEAST, new Int2(1, -1) },
                { EDirFlags.NORTHWEST, new Int2(-1, -1) },
                { EDirFlags.SOUTHEAST, new Int2(1, 1) },
                { EDirFlags.SOUTHWEST, new Int2(-1, 1) },
            };

        public static Int2 Foward(this Int2 x, EDirFlags dir)
        {
            return x + Dic[dir];
        }

        public static Int2 Backward(this Int2 x, EDirFlags dir)
        {
            return x - Dic[dir];
        }
    }
}
