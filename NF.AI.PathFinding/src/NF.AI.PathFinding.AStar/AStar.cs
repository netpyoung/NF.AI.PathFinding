using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NF.Collections.Generic;
using NF.Mathematics;
using NF.AI.PathFinding.Common;

namespace NF.AI.PathFinding.AStar
{
    public class AStar
    {
        public AStar(int width, int height)
        {
            mNodes = new AStarNode[width * height];
            mWalls = new bool[width * height];
        }

        // ============================
        // Public Methods
        // ============================


        public void StepAll()
        {
            mOpenList.Clear();
            mCloseList.Clear();
            mOpenList.Enqueue(mStart, mStart.F);
            Step(int.MaxValue);
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

                mCloseList.Add(curr);

                for (int i = 0; i < 8; ++i)
                {
                    Int2 dir = DIRECTIONS[i];
                    AStarNode adjacent = GetNodeOrNull(curr.Position + dir);
                    if (adjacent == null)
                    {
                        continue;
                    }
                    if (IsWall(adjacent.Position))
                    {
                        continue;
                    }
                    if (mCloseList.Contains(adjacent))
                    {
                        continue;
                    }

                    int g = G(curr, adjacent);
                    if (mOpenList.Contains(adjacent))
                    {
                        if (g >= adjacent.G)
                        {
                            continue;
                        }
                        mOpenList.Remove(adjacent);
                    }
                    adjacent.Parent = curr;
                    adjacent.G = g;
                    adjacent.H = H(adjacent, mGoal);
                    mOpenList.Enqueue(adjacent, adjacent.F);
                }
            }
            return false;
        }

        public void SetStart(Int2 pos)
        {
            if (IsInBoundary(pos))
            {
                mStart = GetNodeOrNull(pos);
            }
        }

        public void SetGoal(Int2 pos)
        {
            if (IsInBoundary(pos))
            {
                mGoal = GetNodeOrNull(pos);
            }
        }

        public void ToggleWall(Int2 pos)
        {
            if (IsInBoundary(pos))
            {
                mWalls[pos.X + pos.Y * this.Width] = !mWalls[pos.X + pos.Y * this.Width];
            }
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

        public bool[] GetWalls()
        {
            return mWalls;
        }

        // ============================
        // Private Methods
        // ============================
        bool IsInBoundary(Int2 pos) => IsInBoundary(pos.X, pos.Y);

        bool IsInBoundary(int x, int y)
        {
            return (0 <= x && x < this.Width) && (0 <= y && y < this.Width);
        }

        AStarNode GetNodeOrNull(Int2 pos)
        {
            int x = pos.X;
            int y = pos.Y;
            if (!IsInBoundary(x, y))
            {
                return null;
            }
            AStarNode node = mNodes[x + y * this.Width];
            if (node != null)
            {
                return node;
            }
            node = new AStarNode(x, y);
            mNodes[x + y * this.Width] = node;
            return node;
        }
        bool IsWall(Int2 pos)
        {
            return mWalls[pos.X + pos.Y * this.Width];
        }
        // ============================
        // Properties
        // ============================
        public int Width { get; private set; }
        public int Height { get; private set; }

        // ============================
        // Private
        // ============================
        AStarNode mStart = null;
        AStarNode mGoal = null;
        readonly AStarNode[] mNodes;
        readonly PriorityQueue<AStarNode> mOpenList = new PriorityQueue<AStarNode>();
        readonly HashSet<AStarNode> mCloseList = new HashSet<AStarNode>();
        readonly bool[] mWalls;

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

        internal static int H(AStarNode n, AStarNode goal)
        {
            // calculate estimated cost
            return Math.Abs(goal.Position.X - n.Position.X) + Math.Abs(goal.Position.Y - n.Position.Y) * 10;
        }
    }
}