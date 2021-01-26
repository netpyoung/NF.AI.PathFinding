using NF.AI.PathFinding.Common;
using NF.Collections.Generic;
using NF.Mathematics;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace NF.AI.PathFinding.JPSOrthogonal
{
    public class JPSOrthogonal
    {

        // =======================
        // Members
        // =======================
        AStarNode mStart = null;
        AStarNode mGoal = null;
        readonly Dictionary<Int2, AStarNode> mCreateNodes = new Dictionary<Int2, AStarNode>();
        readonly PriorityQueue<(AStarNode AStarNode, EDirFlags Dir)> mOpenList = new PriorityQueue<(AStarNode AStarNode, EDirFlags Dir)>();
        readonly HashSet<AStarNode> mCloseList = new HashSet<AStarNode>();
        bool[,] mWalls = null;

        public int Width { get; private set; }
        public int Height { get; private set; }

        public Int2 StartP => mStart.Position;
        public Int2 GoalP => mGoal.Position;

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
            foreach (var AStarNode in mCreateNodes.Values)
            {
                AStarNode.Refresh();
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
                (AStarNode AStarNode, EDirFlags Dir) curr = mOpenList.Dequeue();
                AStarNode currNode = curr.AStarNode;
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
        public bool SetStart(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            mStart = GetOrCreateNode(p);
            return true;
        }

        public bool SetGoal(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            mGoal = GetOrCreateNode(p);
            return true;
        }
        public void SetWall(in Int2 p, bool isWall)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mWalls[p.Y, p.X] = isWall;
        }

        public bool ToggleWall(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            mWalls[p.Y, p.X] = !mWalls[p.Y, p.X];
            return true;
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


        // =======================
        // Private Methods
        // =======================
        private AStarNode GetOrCreateNode(in Int2 p)
        {
            if (mCreateNodes.TryGetValue(p, out AStarNode node))
            {
                return node;
            }
            AStarNode newNode = new AStarNode(p);
            mCreateNodes.Add(p, newNode);
            return newNode;
        }

        public bool IsWalkable(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            return !mWalls[p.Y, p.X];
        }

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
                    if (!IsWalkable(new Int2(pos.X + dp.X, pos.Y)) ||
                        !IsWalkable(new Int2(pos.X, pos.Y + dp.Y)))
                    {
                        continue;
                    }
                }
                ret |= dir;
            }
            return ret;
        }

        internal EDirFlags OrthogonalNeighbourDir(in Int2 pos)
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

        internal EDirFlags OrthogonalForcedNeighbourDir(in Int2 n, EDirFlags dir)
        {
            if (dir == EDirFlags.ALL)
            {
                return EDirFlags.ALL;
            }

            EDirFlags ret = EDirFlags.NONE;

            Int2 next = new Int2(0, 0);
            if (DirFlags.IsStraight(dir))
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
        internal EDirFlags SuccesorsDir(in Int2 pos, EDirFlags dir)
        {
            return NeighbourDir(pos) & (NaturalNeighbours(dir) | OrthogonalForcedNeighbourDir(pos, dir));
        }

        internal bool TryJump(in Int2 p, EDirFlags dir, in Int2 goal, ref Int2 outJumped)
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

                if ((OrthogonalForcedNeighbourDir(next, dir) & OrthogonalNeighbourDir(next)) != EDirFlags.NONE)
                {
                    outJumped = next;
                    return true;
                }

                if (DirFlags.IsDiagonal(dir))
                {
                    // TODO(pyoung): TOC 가능하게 수정 할 수 있나?
                    // d1: EAST  | WEST
                    if (TryJump(next, DirFlags.DiagonalToEastWest(dir), goal, ref outJumped))
                    {
                        outJumped = next;
                        return true;
                    }
                    // d2: NORTH | SOUTH
                    if (TryJump(next, DirFlags.DiagonalToNorthSouth(dir), goal, ref outJumped))
                    {
                        outJumped = next;
                        return true;
                    }
                }
                curr = next;
                next = next.Foward(dir);
            }
        }

        // =========================================
        // Statics
        // =========================================
        static EDirFlags NaturalNeighbours(EDirFlags dir)
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
