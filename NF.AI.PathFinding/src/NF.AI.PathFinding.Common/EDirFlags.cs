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
}
