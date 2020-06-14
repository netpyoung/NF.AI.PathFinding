using NF.Mathematics;
using System;

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
            switch (dir)
            {
                case EDirFlags.NORTH:
                case EDirFlags.SOUTH:
                case EDirFlags.EAST:
                case EDirFlags.WEST:
                    return true;
                default:
                    return false;
            }
        }

        public static bool IsDiagonal(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTHEAST:
                case EDirFlags.NORTHWEST:
                case EDirFlags.SOUTHEAST:
                case EDirFlags.SOUTHWEST:
                    return true;
                default:
                    return false;
            }
        }

        public static EDirFlags DiagonalToEastWest(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTHEAST:
                case EDirFlags.SOUTHEAST: return EDirFlags.EAST;
                case EDirFlags.NORTHWEST:
                case EDirFlags.SOUTHWEST: return EDirFlags.WEST;
                default:
                    return EDirFlags.NONE;
            }
        }

        public static EDirFlags DiagonalToNorthSouth(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTHEAST:
                case EDirFlags.NORTHWEST: return EDirFlags.NORTH;
                case EDirFlags.SOUTHEAST:
                case EDirFlags.SOUTHWEST: return EDirFlags.SOUTH;
                default:
                    return EDirFlags.NONE;
            }
        }

        public static Int2 ToPos(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTH: return new Int2(0, -1);
                case EDirFlags.SOUTH: return new Int2(0, 1);
                case EDirFlags.EAST: return new Int2(1, 0);
                case EDirFlags.WEST: return new Int2(-1, 0);
                case EDirFlags.NORTHEAST: return new Int2(1, -1);
                case EDirFlags.NORTHWEST: return new Int2(-1, -1);
                case EDirFlags.SOUTHEAST: return new Int2(1, 1);
                case EDirFlags.SOUTHWEST: return new Int2(-1, 1);
                default:
                    throw new ArgumentException($"invalid dir: {dir}");
            }
        }
    }
}
