using System;
using System.Collections.Generic;
using System.Linq;
using NF.Collections.Generic;
using NF.Mathematics;
using NF.AI.PathFinding.Common;
using System.Runtime.CompilerServices;

namespace NF.AI.PathFinding.JPSOrthogonal
{
    public class JPSOrthogonal
    {
        public JPSOrthogonal()
        {

        }

        public JPSOrthogonal(int width, int height)
        {
            Init(new bool[height, width]);
        }
        public JPSOrthogonal(bool[,] walls)
        {
            Init(walls);
        }

        public void Init(bool[,] walls)
        {
            this.Width = walls.GetLength(1);
            this.Height = walls.GetLength(0);
            this.mWalls = walls;
        }

        public bool StepAll(int stepCount = int.MaxValue)
        {
            mOpenList.Clear();
            mCloseList.Clear();
            foreach (var AStarNode in mCreatedAStarNodes.Values)
            {
                AStarNode.Refresh();
            }
            mOpenList.Enqueue((mStart, EDirFlags.ALL), mStart.F);
            return Step(stepCount);
        }

        public bool Step(int stepCount)
        {
            int step = stepCount;
            while (true)
            {
                if (step <= 0)
                {
                    return false;
                }
                if (mOpenList.Count == 0)
                {
                    return false;
                }
                (AStarNode AStarNode, EDirFlags Dir) curr = mOpenList.First();
                AStarNode currAStarNode = curr.AStarNode;
                EDirFlags currDir = curr.Dir;
                mOpenList.Remove(curr);
                mCloseList.Add(currAStarNode);

                Int2 currPos = currAStarNode.Position;
                Int2 goalPos = mGoal.Position;
                if (currPos == goalPos)
                {
                    return true;
                }

                EDirFlags succesors = SuccesorsDir(currPos, currDir);
                for (int i = 0b10000000; i > 0; i >>= 1)
                {
                    EDirFlags succesorDir = (EDirFlags)i;
                    if ((succesorDir & succesors) == EDirFlags.NONE)
                    {
                        continue;
                    }

                    Int2? jumpResult = JumpOrNull(currPos, succesorDir, goalPos);
                    if (jumpResult == null)
                    {
                        continue;
                    }

                    Int2 jumpPos = jumpResult.Value;
                    AStarNode jumpAStarNode = GetOrCreatedAStarNode(jumpPos);
                    if (mCloseList.Contains(jumpAStarNode))
                    {
                        continue;
                    }
                    int jumpG = G(currAStarNode, jumpAStarNode);
                    (AStarNode, EDirFlags) openJump = (jumpAStarNode, succesorDir);
                    if (mOpenList.Contains(openJump))
                    {
                        if (jumpG > jumpAStarNode.G)
                        {
                            continue;
                        }
                        mOpenList.Remove(openJump);
                    }
                    jumpAStarNode.Parent = currAStarNode;
                    jumpAStarNode.G = jumpG;
                    jumpAStarNode.H = H(jumpAStarNode, mGoal);
                    mOpenList.Enqueue(openJump, jumpAStarNode.F);
                }
                step--;
            }
        }
        public void SetStart(Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mStart = GetOrCreatedAStarNode(p);
        }

        public void SetGoal(Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mGoal = GetOrCreatedAStarNode(p);
        }

        public void ToggleWall(Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mWalls[p.Y, p.X] = !mWalls[p.Y, p.X];
        }

        public IReadOnlyList<AStarNode> GetPaths()
        {
            List<AStarNode> ret = new List<AStarNode>();
            AStarNode n = mGoal;
            while (n != null)
            {
                ret.Add(n);
                n = n.Parent;
            }
            ret.Reverse();
            return ret;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AStarNode GetStart()
        {
            return this.mStart;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AStarNode GetGoal()
        {
            return this.mGoal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PriorityQueue<(AStarNode, EDirFlags)> GetOpenList()
        {
            return mOpenList;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HashSet<AStarNode> GetCloseList()
        {
            return mCloseList;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool[,] GetWalls()
        {
            return mWalls;
        }

        public int Width { get; private set; }
        public int Height { get; private set; }


        // =======================
        // Private Methods
        // =======================
        private AStarNode GetOrCreatedAStarNode(Int2 p)
        {
            if (mCreatedAStarNodes.TryGetValue(p, out AStarNode createdAStarNode))
            {
                return createdAStarNode;
            }
            AStarNode newAStarNode = new AStarNode(p);
            mCreatedAStarNodes.Add(p, newAStarNode);
            return newAStarNode;
        }

        bool IsWalkable(Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            return !mWalls[p.Y, p.X + p.Y];
        }

        bool IsInBoundary(Int2 p)
        {
            return (0 <= p.X && p.X < this.Width) && (0 <= p.Y && p.Y < this.Height);
        }

        internal EDirFlags NeighbourDir(Int2 pos)
        {
            EDirFlags ret = EDirFlags.NONE;
            for (int i = 0b10000000; i > 0; i >>= 1)
            {
                EDirFlags dir = (EDirFlags)i;
                if (!IsWalkable(pos.Foward(dir)))
                {
                    continue;
                }

                if (IsDiagonal(dir))
                {
                    Int2 dp = GetDirectionByDirFlags(dir);
                    if (!IsWalkable(new Int2(pos.X + dp.X, pos.Y)) &&
                        !IsWalkable(new Int2(pos.X, pos.Y + dp.Y)))
                    {
                        continue;
                    }
                }
                ret |= dir;
            }
            return ret;
        }

        internal EDirFlags OrthogonalNeighbourDir(Int2 pos)
        {
            EDirFlags ret = EDirFlags.NONE;
            for (int i = 0b00001000; i > 0; i >>= 1)
            {
                EDirFlags dir = (EDirFlags)i;
                if (!IsWalkable(pos.Foward(dir)))
                {
                    continue;
                }
                ret |= dir;
            }
            return ret;
        }

        internal EDirFlags ForcedNeighbourDir(Int2 n, EDirFlags dir)
        {
            EDirFlags ret = EDirFlags.NONE;

            Int2 next = new Int2(0, 0);
            if (((int)dir & 0b1111) != 0)
            {
                next = n.Backward(dir);
            }

            switch (dir)
            {
                case EDirFlags.NORTH:
                    // F . F
                    // X N X
                    // . P .
                    if (!IsWalkable(next.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.EAST | EDirFlags.NORTHEAST;
                    }
                    if (!IsWalkable(next.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.WEST | EDirFlags.NORTHWEST;
                    }
                    break;
                case EDirFlags.SOUTH:
                    // . P .
                    // X N X
                    // F . F
                    if (!IsWalkable(next.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.EAST | EDirFlags.SOUTHEAST;
                    }
                    if (!IsWalkable(next.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.WEST | EDirFlags.SOUTHWEST;
                    }
                    break;
                case EDirFlags.EAST:
                    // . X F
                    // P N .
                    // . X F
                    if (!IsWalkable(next.Foward(EDirFlags.NORTH)))
                    {
                        ret |= EDirFlags.NORTH | EDirFlags.NORTHEAST;
                    }
                    if (!IsWalkable(next.Foward(EDirFlags.SOUTH)))
                    {
                        ret |= EDirFlags.SOUTH | EDirFlags.SOUTHEAST;
                    }
                    break;
                case EDirFlags.WEST:
                    // F X .
                    // . N P
                    // F X .
                    if (!IsWalkable(next.Foward(EDirFlags.NORTH)))
                    {
                        ret |= EDirFlags.NORTH | EDirFlags.NORTHWEST;
                    }
                    if (!IsWalkable(next.Foward(EDirFlags.SOUTH)))
                    {
                        ret |= EDirFlags.SOUTH | EDirFlags.SOUTHWEST;
                    }
                    break;
                case EDirFlags.NORTHWEST:
                    break;
                case EDirFlags.NORTHEAST:
                    break;
                case EDirFlags.SOUTHWEST:
                    break;
                case EDirFlags.SOUTHEAST:
                    break;
                case EDirFlags.ALL:
                    ret |= EDirFlags.ALL;
                    break;
                case EDirFlags.NONE:
                default:
                    throw new ArgumentOutOfRangeException($"[ForcedNeighbourDir] invalid dir - {dir}");
            }
            return ret;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal EDirFlags SuccesorsDir(Int2 pos, EDirFlags dir)
        {
            return NeighbourDir(pos) & (NaturalNeighbours(dir) | ForcedNeighbourDir(pos, dir));
        }

        internal Int2? JumpOrNull(Int2 p, EDirFlags dir, Int2 goal)
        {
            Int2 next = p.Foward(dir);
            while (true)
            {
                if (!IsWalkable(next))
                {
                    return null;
                }
                if ((dir & NeighbourDir(p)) == EDirFlags.NONE)
                {
                    return null;
                }
                if (next == goal)
                {
                    return next;
                }

                if ((ForcedNeighbourDir(next, dir) & OrthogonalNeighbourDir(next)) != EDirFlags.NONE)
                {
                    return next;
                }

                if (IsDiagonal(dir))
                {
                    // TODO(pyoung): TOC 가능하게 수정 할 수 있나?
                    // d1: EAST  | WEST
                    if (JumpOrNull(next, DiagonalToEastWest(dir), goal).HasValue)
                    {
                        return next;
                    }
                    // d2: NORTH | SOUTH
                    if (JumpOrNull(next, DiagonalToNorthSouth(dir), goal).HasValue)
                    {
                        return next;
                    }
                }
                next = next.Foward(dir);
            }
        }

        // =========================================
        // Statics
        // =========================================
        static bool IsDiagonal(EDirFlags dir)
        {
            switch (dir)
            {
                case EDirFlags.NORTHEAST:
                case EDirFlags.NORTHWEST:
                case EDirFlags.SOUTHEAST:
                case EDirFlags.SOUTHWEST:
                    return true;
                case EDirFlags.NONE:
                case EDirFlags.NORTH:
                case EDirFlags.SOUTH:
                case EDirFlags.EAST:
                case EDirFlags.WEST:
                case EDirFlags.ALL:
                default:
                    return false;
            }
        }

        static Int2[] DIRECTIONS = new Int2[] {
            new Int2(0, -1),	// NORTH
	        new Int2(0, 1),		// SOUTH
	        new Int2(1, 0),		// EAST
	        new Int2(-1, 0),	// WEST
	        new Int2(1, -1),	// NORTHEAST
	        new Int2(-1, -1),	// NORTHWEST
	        new Int2(1, 1),		// SOUTHEAST
	        new Int2(-1, 1),	// SOUTHWEST
        };
        static Int2 GetDirectionByDirFlags(EDirFlags dir)
        {
            for (int i = 0; i < 8; ++i)
            {
                if ((dir & (EDirFlags)(1 << i)) == EDirFlags.NONE)
                {
                    continue;
                }
                return DIRECTIONS[i];
            }
            throw new ArgumentOutOfRangeException($"invalid range - {dir}");
        }

        static EDirFlags NaturalNeighbours(EDirFlags dir)
        {

            switch (dir)
            {
                case EDirFlags.NORTH:
                case EDirFlags.SOUTH:
                case EDirFlags.EAST:
                case EDirFlags.WEST: return dir;
                case EDirFlags.NORTHEAST: return EDirFlags.NORTHEAST | EDirFlags.NORTH | EDirFlags.EAST;
                case EDirFlags.NORTHWEST: return EDirFlags.NORTHWEST | EDirFlags.NORTH | EDirFlags.WEST;
                case EDirFlags.SOUTHEAST: return EDirFlags.SOUTHEAST | EDirFlags.SOUTH | EDirFlags.EAST;
                case EDirFlags.SOUTHWEST: return EDirFlags.SOUTHWEST | EDirFlags.SOUTH | EDirFlags.WEST;
                case EDirFlags.NONE:
                case EDirFlags.ALL:
                default:
                    return dir;
            }
        }

        static EDirFlags DiagonalToEastWest(EDirFlags dir)
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

        static EDirFlags DiagonalToNorthSouth(EDirFlags dir)
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int G(AStarNode from, AStarNode adjacent)
        {
            // cost so far to reach n 
            Int2 p = from.Position - adjacent.Position;
            if (p.X == 0 || p.Y == 0)
            {
                return from.G + 10;
            }
            else
            {
                return from.G + 14;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int H(AStarNode n, AStarNode goal)
        {
            // calculate estimated cost
            return Math.Abs(goal.Position.X - n.Position.X) + Math.Abs(goal.Position.Y - n.Position.Y) * 10;
        }

        // =======================
        // Members
        // =======================
        AStarNode mStart = null;
        AStarNode mGoal = null;
        readonly Dictionary<Int2, AStarNode> mCreatedAStarNodes = new Dictionary<Int2, AStarNode>();
        readonly PriorityQueue<(AStarNode AStarNode, EDirFlags Dir)> mOpenList = new PriorityQueue<(AStarNode AStarNode, EDirFlags Dir)>();
        readonly HashSet<AStarNode> mCloseList = new HashSet<AStarNode>();
        bool[,] mWalls = null;
    }
}
