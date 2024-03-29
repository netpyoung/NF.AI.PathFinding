using NF.AI.PathFinding.Common;
using NF.Collections.Generic;
using NF.Mathematics;

using System;
using System.Collections.Generic;

namespace NF.AI.PathFinding.AStar
{
    public class AStar
    {
        // ============================
        // Private
        // ============================
        private AStarNode mStart = null;
        private AStarNode mGoal = null;
        private readonly AStarNode[,] mNodes;
        private readonly PriorityQueue<AStarNode> mOpenList = new PriorityQueue<AStarNode>();
        private readonly HashSet<AStarNode> mCloseList = new HashSet<AStarNode>();
        private readonly bool[,] mWalls;

        // ============================
        // Properties
        // ============================
        public int Width { get; private set; }
        public int Height { get; private set; }
        public Int2 StartP => mStart.Position;
        public Int2 GoalP => mGoal.Position;

        public AStar(int width, int height)
        {
            Width = width;
            Height = height;
            mNodes = new AStarNode[Height, Width];
            mWalls = new bool[Height, Width];
        }

        public AStar(bool[,] walls)
        {
            Height = walls.GetLength(0);
            Width = walls.GetLength(1);
            mNodes = new AStarNode[Height, Width];
            mWalls = walls;
        }

        // ============================
        // Public Methods
        // ============================

        public bool StepAll()
        {
            mOpenList.Clear();
            mCloseList.Clear();
            mOpenList.Enqueue(mStart, mStart.F);
            mGoal.Parent = null;
            return Step(int.MaxValue);
        }

        public bool Step(int stepCount)
        {
            for (int step = stepCount; step > 0; --step)
            {
                if (mOpenList.Count == 0)
                {
                    return false;
                }

                AStarNode curr = mOpenList.Dequeue();
                if (curr == mGoal)
                {
                    return true;
                }

                _ = mCloseList.Add(curr);

                for (int i = 0b10000000; i > 0; i >>= 1)
                {

                    EDirFlags dir = (EDirFlags)i;
                    Int2 dp = DirFlags.ToPos(dir);
                    AStarNode adjacent = GetNodeOrNull(curr.Position + dp);
                    if (adjacent == null)
                    {
                        continue;
                    }
                    if (IsWall(adjacent.Position))
                    {
                        continue;
                    }

                    if (DirFlags.IsDiagonal(dir))
                    { // for prevent corner cutting
                        if (IsWall(curr.Position + new Int2(dp.X, 0)) || IsWall(curr.Position + new Int2(0, dp.Y)))
                        {
                            continue;
                        }
                    }

                    if (mCloseList.Contains(adjacent))
                    {
                        continue;
                    }

                    int nextG = G(curr, adjacent);
                    if (!mOpenList.Contains(adjacent))
                    {
                        adjacent.Parent = curr;
                        adjacent.G = nextG;
                        adjacent.H = H(adjacent, mGoal);
                        mOpenList.Enqueue(adjacent, adjacent.F);
                    }
                    else if (nextG < adjacent.G)
                    {
                        adjacent.Parent = curr;
                        adjacent.G = nextG;
                        adjacent.H = H(adjacent, mGoal);
                        mOpenList.UpdatePriority(adjacent, adjacent.F);
                    }
                }
            }
            return false;
        }

        public bool SetStart(in Int2 pos)
        {
            if (!IsInBoundary(pos))
            {
                return false;
            }

            mStart = GetNodeOrNull(pos);
            return true;
        }

        public bool SetGoal(in Int2 pos)
        {
            if (!IsInBoundary(pos))
            {
                return false;
            }

            mGoal = GetNodeOrNull(pos);
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

        public bool ToggleWall(in Int2 pos)
        {
            if (!IsInBoundary(pos))
            {
                return false;
            }
            mWalls[pos.Y, pos.X] = !mWalls[pos.Y, +pos.X];
            return true;
        }

        public List<AStarNode> GetPaths()
        {
            List<AStarNode> ret = new List<AStarNode>();
            AStarNode node = mGoal;
            while (node != null)
            {
                ret.Add(node);
                node = node.Parent;
            }
            ret.Reverse();
            return ret;
        }

        public AStarNode GetStart()
        {
            return mStart;
        }

        public AStarNode GetGoal()
        {
            return mGoal;

        }

        public PriorityQueue<AStarNode> GetOpenList()
        {
            return mOpenList;
        }

        public HashSet<AStarNode> GetCloseList()
        {
            return mCloseList;
        }

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
            return !IsWall(p);
        }

        // ============================
        // Private Methods
        // ============================
        private bool IsInBoundary(in Int2 pos)
        {
            return IsInBoundary(pos.X, pos.Y);
        }

        private bool IsInBoundary(int x, int y)
        {
            return 0 <= x && x < Width && 0 <= y && y < Height;
        }

        private AStarNode GetNodeOrNull(in Int2 pos)
        {
            int x = pos.X;
            int y = pos.Y;
            if (!IsInBoundary(x, y))
            {
                return null;
            }
            AStarNode node = mNodes[y, x];
            if (node != null)
            {
                return node;
            }
            node = new AStarNode(x, y);
            mNodes[y, x] = node;
            return node;
        }

        private bool IsWall(in Int2 pos)
        {
            return mWalls[pos.Y, pos.X];
        }

        private static int G(AStarNode from, AStarNode adjacent)
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

        internal static int H(AStarNode n, AStarNode goal)
        {
            // calculate estimated cost
            return Math.Abs(goal.Position.X - n.Position.X) + (Math.Abs(goal.Position.Y - n.Position.Y) * 10);
        }
    }
}
