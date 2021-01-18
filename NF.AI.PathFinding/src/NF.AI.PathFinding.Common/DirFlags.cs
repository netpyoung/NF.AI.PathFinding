using NF.Mathematics;
using System.Collections.Generic;

namespace NF.AI.PathFinding.Common
{
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

        public static bool IsStraight(EDirFlags dir)
        {
            return (dir & (EDirFlags.NORTH | EDirFlags.SOUTH | EDirFlags.EAST | EDirFlags.WEST)) != EDirFlags.NONE;
        }

        public static bool IsDiagonal(EDirFlags dir)
        {
            return (dir & (EDirFlags.NORTHEAST | EDirFlags.NORTHWEST | EDirFlags.SOUTHEAST | EDirFlags.SOUTHWEST)) != EDirFlags.NONE;
        }

        public static EDirFlags DiagonalToEastWest(EDirFlags dir)
        {
            if ((dir & (EDirFlags.NORTHEAST | EDirFlags.SOUTHEAST)) != EDirFlags.NONE)
            {
                return EDirFlags.EAST;
            }

            if ((dir & (EDirFlags.NORTHWEST | EDirFlags.SOUTHWEST)) != EDirFlags.NONE)
            {
                return EDirFlags.WEST;
            }

            return EDirFlags.NONE;
        }

        public static EDirFlags DiagonalToNorthSouth(EDirFlags dir)
        {
            if ((dir & (EDirFlags.NORTHEAST | EDirFlags.NORTHWEST)) != EDirFlags.NONE)
            {
                return EDirFlags.NORTH;
            }

            if ((dir & (EDirFlags.SOUTHEAST | EDirFlags.SOUTHWEST)) != EDirFlags.NONE)
            {
                return EDirFlags.SOUTH;
            }

            return EDirFlags.NONE;
        }

        static Dictionary<EDirFlags, Int2> DirToPos = new Dictionary<EDirFlags, Int2>()
        {
            { EDirFlags.NORTH,new Int2(0, -1) },
            { EDirFlags.SOUTH,new Int2(0, 1) },
            { EDirFlags.EAST,new Int2(1, 0) },
            { EDirFlags.WEST,new Int2(-1, 0) },
            { EDirFlags.NORTHEAST,new Int2(1, -1) },
            { EDirFlags.NORTHWEST,new Int2(-1, -1) },
            { EDirFlags.SOUTHEAST,new Int2(1, 1) },
            { EDirFlags.SOUTHWEST,new Int2(-1, 1) },
        };

        public static Int2 ToPos(EDirFlags dir)
        {
            return DirToPos[dir];
        }
    }
}
