using NF.AI.PathFinding.Common;
using NF.Collections.Generic;
using NF.Mathematics;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace NF.AI.PathFinding.JPS
{
    public class JPS
    {
        // =======================
        // Members
        // =======================
        AStarNode mStart = null;
        AStarNode mGoal = null;
        readonly Dictionary<Int2, AStarNode> mCreatedNodes = new Dictionary<Int2, AStarNode>();
        readonly PriorityQueue<(AStarNode Node, EDirFlags Dir)> mOpenList = new PriorityQueue<(AStarNode Node, EDirFlags Dir)>();
        readonly HashSet<AStarNode> mCloseList = new HashSet<AStarNode>();
        bool[,] mWalls = null;

        public int Width { get; private set; }
        public int Height { get; private set; }
        public Int2 StartP => mStart.Position;
        public Int2 GoalP => mGoal.Position;

        public JPS()
        {

        }

        public JPS(int width, int height)
        {
            Init(new bool[height, width]);
        }
        public JPS(bool[,] walls)
        {
            Init(walls);
        }

        public void Init(bool[,] walls)
        {
            mWalls = walls;
            Width = walls.GetLength(1);
            Height = walls.GetLength(0);
        }

        public bool StepAll(int stepCount = int.MaxValue)
        {
            mOpenList.Clear();
            mCloseList.Clear();
            foreach (var node in mCreatedNodes.Values)
            {
                node.Refresh();
            }
            mOpenList.Enqueue((mStart, EDirFlags.ALL), mStart.F);
            return Step(stepCount);
        }

        public bool Step(int stepCount)
        {
            int step = stepCount;
            Int2 jumpPos = new Int2(0, 0);

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
                (AStarNode Node, EDirFlags Dir) curr = mOpenList.Dequeue();
                AStarNode currNode = curr.Node;
                EDirFlags currDir = curr.Dir;
                mCloseList.Add(currNode);

                Int2 currPos = currNode.Position;
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

                    if (!TryJump(currPos, succesorDir, goalPos, ref jumpPos))
                    {
                        continue;
                    }

                    AStarNode jumpNode = GetOrCreateNode(jumpPos);
                    if (mCloseList.Contains(jumpNode))
                    {
                        continue;
                    }

                    int jumpG = G(currNode, jumpNode);
                    (AStarNode, EDirFlags) openJump = (jumpNode, succesorDir);
                    if (!mOpenList.Contains(openJump))
                    {
                        jumpNode.Parent = currNode;
                        jumpNode.G = jumpG;
                        jumpNode.H = H(jumpNode, mGoal);
                        mOpenList.Enqueue(openJump, jumpNode.F);
                    }
                    else if (jumpG < jumpNode.G)
                    {
                        jumpNode.Parent = currNode;
                        jumpNode.G = jumpG;
                        jumpNode.H = H(jumpNode, mGoal);
                        mOpenList.UpdatePriority(openJump, jumpNode.F);
                    }
                }
                step--;
            }
        }
        public void SetStart(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mStart = GetOrCreateNode(p);
        }

        public void SetGoal(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mGoal = GetOrCreateNode(p);
        }

        public void SetWall(in Int2 p, bool isWall)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mWalls[p.Y, p.X] = isWall;
        }

        public void ToggleWall(in Int2 p)
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

        public bool IsWalkable(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            return !mWalls[p.Y, p.X];
        }

        // =======================
        // Private Methods
        // =======================
        private AStarNode GetOrCreateNode(in Int2 p)
        {
            if (mCreatedNodes.TryGetValue(p, out AStarNode createdNode))
            {
                return createdNode;
            }
            AStarNode newNode = new AStarNode(p);
            mCreatedNodes.Add(p, newNode);
            return newNode;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool IsInBoundary(in Int2 p)
        {
            return (0 <= p.X && p.X < this.Width) && (0 <= p.Y && p.Y < this.Height);
        }

        internal EDirFlags NeighbourDir(in Int2 pos)
        {
            EDirFlags ret = EDirFlags.NONE;
            for (int i = 0b10000000; i > 0; i >>= 1)
            {
                EDirFlags dir = (EDirFlags)i;
                if (!IsWalkable(pos.Foward(dir)))
                {
                    continue;
                }

                if (DirFlags.IsDiagonal(dir))
                {
                    Int2 dp = DirFlags.ToPos(dir);
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

        internal EDirFlags ForcedNeighbourDir(in Int2 n, EDirFlags dir)
        {
            EDirFlags ret = EDirFlags.NONE;
            switch (dir)
            {
                case EDirFlags.NORTH:
                    // F . F
                    // X N X
                    // . P .
                    if (!IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.NORTHEAST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.NORTHWEST;
                    }
                    break;
                case EDirFlags.SOUTH:
                    // . P .
                    // X N X
                    // F . F
                    if (!IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.SOUTHEAST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.SOUTHWEST;
                    }
                    break;
                case EDirFlags.EAST:
                    // . X F
                    // P N .
                    // . X F
                    if (!IsWalkable(n.Foward(EDirFlags.NORTH)))
                    {
                        ret |= EDirFlags.NORTHEAST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.SOUTH)))
                    {
                        ret |= EDirFlags.SOUTHEAST;
                    }
                    break;
                case EDirFlags.WEST:
                    // F X .
                    // . N P
                    // F X .
                    if (!IsWalkable(n.Foward(EDirFlags.NORTH)))
                    {
                        ret |= EDirFlags.NORTHWEST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.SOUTH)))
                    {
                        ret |= EDirFlags.SOUTHWEST;
                    }
                    break;
                case EDirFlags.NORTHWEST:
                    // . . F
                    // . N X
                    // F X P
                    if (IsWalkable(n.Foward(EDirFlags.NORTH)) && !IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.NORTHEAST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.SOUTH)) && IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.SOUTHWEST;
                    }
                    break;
                case EDirFlags.NORTHEAST:
                    // F . .
                    // X N .
                    // P X F
                    if (IsWalkable(n.Foward(EDirFlags.NORTH)) && !IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.NORTHWEST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.SOUTH)) && IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.SOUTHEAST;
                    }
                    break;
                case EDirFlags.SOUTHWEST:
                    // F X P
                    // . N X
                    // . . F
                    if (IsWalkable(n.Foward(EDirFlags.SOUTH)) && !IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.SOUTHEAST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.NORTH)) && IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.NORTHWEST;
                    }
                    break;
                case EDirFlags.SOUTHEAST:
                    // P X F
                    // X N .
                    // F . .
                    if (IsWalkable(n.Foward(EDirFlags.SOUTH)) && !IsWalkable(n.Foward(EDirFlags.WEST)))
                    {
                        ret |= EDirFlags.SOUTHWEST;
                    }
                    if (!IsWalkable(n.Foward(EDirFlags.NORTH)) && IsWalkable(n.Foward(EDirFlags.EAST)))
                    {
                        ret |= EDirFlags.NORTHEAST;
                    }
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
        internal EDirFlags SuccesorsDir(in Int2 pos, EDirFlags dir)
        {
            return NeighbourDir(pos) & (NaturalNeighbours(dir) | ForcedNeighbourDir(pos, dir));
        }

        internal bool TryJump(in Int2 p, EDirFlags dir, in Int2 goal, ref Int2 outJumped)
        {
            if (DirFlags.IsStraight(dir))
            {
                return TryJumpStraight(p, dir, goal, ref outJumped);
            }
            else
            {
                return TryJumpDiagonal(p, dir, goal, ref outJumped);
            }
        }

        internal bool TryJumpStraight(in Int2 p, EDirFlags dir, in Int2 goal, ref Int2 outJumped)
        {
            Int2 curr = p;
            Int2 next = curr.Foward(dir);

            while (true)
            {
                if (!IsWalkable(next))
                {
                    return false;
                }

                if ((dir & NeighbourDir(curr)) == EDirFlags.NONE)
                {
                    return false;
                }

                if (next == goal)
                {
                    outJumped = next;
                    return true;
                }

                if ((ForcedNeighbourDir(next, dir) & NeighbourDir(next)) != EDirFlags.NONE)
                {
                    outJumped = next;
                    return true;
                }

                curr = next;
                next = curr.Foward(dir);
            }
        }

        internal bool TryJumpDiagonal(in Int2 p, EDirFlags dir, in Int2 goal, ref Int2 outJumped)
        {
            Int2 curr = p;
            Int2 next = curr.Foward(dir);

            while (true)
            {
                if (!IsWalkable(next))
                {
                    return false;
                }

                if ((dir & NeighbourDir(curr)) == EDirFlags.NONE)
                {
                    return false;
                }

                if (next == goal)
                {
                    outJumped = next;
                    return true;
                }

                if ((ForcedNeighbourDir(next, dir) & NeighbourDir(next)) != EDirFlags.NONE)
                {
                    outJumped = next;
                    return true;
                }

                // d1: EAST  | WEST
                if (TryJumpStraight(next, DirFlags.DiagonalToEastWest(dir), goal, ref outJumped))
                {
                    outJumped = next;
                    return true;
                }

                // d2: NORTH | SOUTH
                if (TryJumpStraight(next, DirFlags.DiagonalToNorthSouth(dir), goal, ref outJumped))
                {
                    outJumped = next;
                    return true;
                }

                curr = next;
                next = curr.Foward(dir);
            }
        }

        // =========================================
        // Statics
        // =========================================
        internal static EDirFlags NaturalNeighbours(EDirFlags dir)
        {

            switch (dir)
            {
                case EDirFlags.NORTHEAST: return EDirFlags.NORTHEAST | EDirFlags.NORTH | EDirFlags.EAST;
                case EDirFlags.NORTHWEST: return EDirFlags.NORTHWEST | EDirFlags.NORTH | EDirFlags.WEST;
                case EDirFlags.SOUTHEAST: return EDirFlags.SOUTHEAST | EDirFlags.SOUTH | EDirFlags.EAST;
                case EDirFlags.SOUTHWEST: return EDirFlags.SOUTHWEST | EDirFlags.SOUTH | EDirFlags.WEST;
                default:
                    return dir;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int G(AStarNode from, AStarNode adjacent)
        {
            // cost so far to reach n 
            Int2 p = from.Position - adjacent.Position;
            if (p.X == 0 || p.Y == 0)
            {
                return from.G + Math.Max(Math.Abs(p.X), Math.Abs(p.Y)) * 10;
            }
            else
            {
                return from.G + Math.Max(Math.Abs(p.X), Math.Abs(p.Y)) * 14;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int H(AStarNode n, AStarNode goal)
        {
            // calculate estimated cost
            return (Math.Abs(goal.Position.X - n.Position.X) + Math.Abs(goal.Position.Y - n.Position.Y)) * 10;
        }
    }
}
