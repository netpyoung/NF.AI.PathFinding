using NF.AI.PathFinding.Common;
using NF.Collections.Generic;
using NF.Mathematics;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlus
    {
        public JPSPlus()
        {

        }

        public void Init(JPSPlusBakedMap bakedMap)
        {
            mBakedMap = bakedMap;
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

                (JPSPlusNode Node, EDirFlags fromDir) curr = mOpenList.Dequeue();
                JPSPlusNode currNode = curr.Node;
                EDirFlags fromDir = curr.fromDir;
                mCloseList.Add(currNode);

                Int2 currPos = currNode.Position;
                Int2 goalPos = this.mGoal.Position;

                if (currPos == goalPos)
                {
                    return true;
                }

                EDirFlags validDirs = ValidLookUPTable(fromDir);
                for (int i = 0b10000000; i > 0; i >>= 1)
                {
                    EDirFlags processDir = (EDirFlags)i;
                    if ((processDir & validDirs) == EDirFlags.NONE)
                    {
                        continue;
                    }

                    bool isDiagonalDir = DirFlags.IsDiagonal(processDir);
                    int dirDistance = currNode.GetDistance(processDir);
                    int lengthX = RowDiff(currNode, mGoal);
                    int lengthY = ColDiff(currNode, mGoal);


                    JPSPlusNode nextNode;
                    int nextG;
                    if (!isDiagonalDir
                        && IsGoalInExactDirection(currPos, processDir, goalPos)
                        && Math.Max(lengthX, lengthY) <= Math.Abs(dirDistance))
                    {
                        // 직선이동중
                        // 골과 같은 방향
                        // 골 노드거리 방향거리보다 같거나 작으면 그게 바로 골
                        nextNode = mGoal;
                        nextG = currNode.G + Math.Max(lengthX, lengthY) * 10;
                    }
                    else if (isDiagonalDir
                        && IsGoalInGeneralDirection(currPos, processDir, goalPos)
                        && (lengthX <= Math.Abs(dirDistance) || lengthY <= Math.Abs(dirDistance))
                        )
                    {
                        // Target Jump Point
                        // 대각 이동중
                        // 골과 일반적 방향
                        int minDiff = Math.Min(lengthX, lengthY);
                        nextNode = GetNode(currNode, minDiff, processDir);
                        nextG = currNode.G + (minDiff * 14); // 대각길이 비용.
                    }
                    else if (dirDistance > 0)
                    {
                        // 점프가 가능하면 점프!
                        nextNode = GetNode(currNode, processDir);
                        if (isDiagonalDir)
                        {
                            nextG = currNode.G + Math.Max(lengthX, lengthY) * 14;
                        }
                        else
                        {
                            nextG = currNode.G + Math.Max(lengthX, lengthY) * 10;
                        }
                    }
                    else
                    {
                        // 찾지못하면 다음 것으로.
                        continue;
                    }

                    (JPSPlusNode, EDirFlags) openJump = (nextNode, processDir);

                    if (!mOpenList.Contains(openJump) || !mCloseList.Contains(nextNode))
                    {
                        nextNode.Parent = currNode;
                        nextNode.G = nextG;
                        nextNode.H = H(nextNode, mGoal);
                        mOpenList.Enqueue(openJump, nextNode.F);
                    }
                    else if (nextG < nextNode.G)
                    {
                        nextNode.Parent = currNode;
                        nextNode.G = nextG;
                        nextNode.H = H(nextNode, mGoal);
                        mOpenList.UpdatePriority(openJump, nextNode.F);
                    }
                }
                step--;
            }
        }

        public bool SetStart(Int2 p)
        {
            if (mBakedMap == null)
            {
                return false;
            }

            if (!IsInBoundary(p))
            {
                return false;
            }

            mStart = GetOrCreatedNode(p);
            return true;
        }

        public bool SetGoal(Int2 p)
        {
            if (mBakedMap == null)
            {
                return false;
            }

            if (!IsInBoundary(p))
            {
                return false;
            }
            mGoal = GetOrCreatedNode(p);
            return true;
        }
        
        public JPSPlusNode GetJPSPlusNode(Int2 p)
        {
            return new JPSPlusNode(p, mBakedMap.Blocks[mBakedMap.BlockLUT[p.Y, p.X]].JumpDistances);
        }

        public IReadOnlyList<JPSPlusNode> GetPaths()
        {
            List<JPSPlusNode> ret = new List<JPSPlusNode>();
            JPSPlusNode n = mGoal;
            while (n != null)
            {
                ret.Add(n);
                n = n.Parent;
            }
            ret.Reverse();
            return ret;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public JPSPlusNode GetStart()
        {
            return this.mStart;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public JPSPlusNode GetGoal()
        {
            return this.mGoal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PriorityQueue<(JPSPlusNode, EDirFlags)> GetOpenList()
        {
            return mOpenList;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public HashSet<JPSPlusNode> GetCloseList()
        {
            return mCloseList;
        }

        public int Width => mBakedMap.Width;
        public int Height => mBakedMap.Height;

        // =======================
        // Private Methods
        // =======================
        private JPSPlusNode GetOrCreatedNode(Int2 p)
        {
            if (mCreatedNodes.TryGetValue(p, out JPSPlusNode createdNode))
            {
                return createdNode;
            }
            JPSPlusNode newNode = GetJPSPlusNode(p);
            mCreatedNodes.Add(p, newNode);
            return newNode;
        }

        bool IsInBoundary(Int2 p)
        {
            return (0 <= p.X && p.X < this.Width) && (0 <= p.Y && p.Y < this.Height);
        }

        #region plus
        EDirFlags ValidLookUPTable(EDirFlags dir)
        {
            switch (dir)
            {
                // . N .
                // W . E
                // . S .
                case EDirFlags.NORTH:
                    return EDirFlags.EAST | EDirFlags.NORTHEAST | EDirFlags.NORTH | EDirFlags.NORTHWEST | EDirFlags.WEST;
                case EDirFlags.WEST:
                    return EDirFlags.NORTH | EDirFlags.NORTHWEST | EDirFlags.WEST | EDirFlags.SOUTHWEST | EDirFlags.SOUTH;
                case EDirFlags.EAST:
                    return EDirFlags.SOUTH | EDirFlags.SOUTHEAST | EDirFlags.EAST | EDirFlags.NORTHEAST | EDirFlags.NORTH;
                case EDirFlags.SOUTH:
                    return EDirFlags.WEST | EDirFlags.SOUTHWEST | EDirFlags.SOUTH | EDirFlags.SOUTHEAST | EDirFlags.EAST;
                case EDirFlags.NORTHWEST:
                    return EDirFlags.NORTH | EDirFlags.NORTHWEST | EDirFlags.WEST;
                case EDirFlags.NORTHEAST:
                    return EDirFlags.EAST | EDirFlags.NORTHEAST | EDirFlags.NORTH;
                case EDirFlags.SOUTHWEST:
                    return EDirFlags.WEST | EDirFlags.SOUTHWEST | EDirFlags.SOUTH;
                case EDirFlags.SOUTHEAST:
                    return EDirFlags.SOUTH | EDirFlags.SOUTHEAST | EDirFlags.EAST;
                default:
                    return dir;
            }
        }

        bool IsGoalInExactDirection(Int2 curr, EDirFlags processDir, Int2 goal)
        {
            int dx = goal.X - curr.X;
            int dy = goal.Y - curr.Y;

            switch (processDir)
            {
                case EDirFlags.NORTH:
                    return (dx == 0 && dy < 0);
                case EDirFlags.SOUTH:
                    return (dx == 0 && dy > 0);
                case EDirFlags.WEST:
                    return (dx < 0 && dy == 0);
                case EDirFlags.EAST:
                    return (dx > 0 && dy == 0);
                case EDirFlags.NORTHWEST:
                    return (dx < 0 && dy < 0) && (Math.Abs(dx) == Math.Abs(dy));
                case EDirFlags.NORTHEAST:
                    return (dx > 0 && dy < 0) && (Math.Abs(dx) == Math.Abs(dy));
                case EDirFlags.SOUTHWEST:
                    return (dx < 0 && dy > 0) && (Math.Abs(dx) == Math.Abs(dy));
                case EDirFlags.SOUTHEAST:
                    return (dx > 0 && dy > 0) && (Math.Abs(dx) == Math.Abs(dy));
                default:
                    return false;
            }
        }

        bool IsGoalInGeneralDirection(Int2 curr, EDirFlags processDir, Int2 goal)
        {
            int dx = goal.X - curr.X;
            int dy = goal.Y - curr.Y;

            switch (processDir)
            {
                case EDirFlags.NORTH:
                    return (dx == 0 && dy < 0);
                case EDirFlags.SOUTH:
                    return (dx == 0 && dy > 0);
                case EDirFlags.WEST:
                    return (dx < 0 && dy == 0);
                case EDirFlags.EAST:
                    return (dx > 0 && dy == 0);
                case EDirFlags.NORTHWEST:
                    return (dx < 0 && dy < 0);
                case EDirFlags.NORTHEAST:
                    return (dx > 0 && dy < 0);
                case EDirFlags.SOUTHWEST:
                    return (dx < 0 && dy > 0);
                case EDirFlags.SOUTHEAST:
                    return (dx > 0 && dy > 0);
                default:
                    return false;
            }
        }

        JPSPlusNode GetNode(JPSPlusNode node, EDirFlags dir)
        {
            Int2 pos = node.Position;
            return GetOrCreatedNode(pos + (DirFlags.ToPos(dir) * node.GetDistance(dir)));
        }

        JPSPlusNode GetNode(JPSPlusNode node, int dist, EDirFlags dir)
        {
            Int2 pos = node.Position;
            return GetOrCreatedNode(pos + (DirFlags.ToPos(dir) * dist));
        }

        int ColDiff(JPSPlusNode currNode, JPSPlusNode goalNode)
        {
            Int2 currP = currNode.Position;
            Int2 goalP = goalNode.Position;
            return Math.Abs(goalP.X - currP.X);
        }

        int RowDiff(JPSPlusNode currNode, JPSPlusNode goalNode)
        {
            Int2 currP = currNode.Position;
            Int2 goalP = goalNode.Position;
            return Math.Abs(goalP.Y - currP.Y);
        }
        #endregion plus

        // =========================================
        // Statics
        // =========================================
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int H(JPSPlusNode n, JPSPlusNode goal)
        {
            // calculate estimated cost
            return (Math.Abs(goal.Position.X - n.Position.X) + Math.Abs(goal.Position.Y - n.Position.Y)) * 10;
        }

        // =======================
        // Members
        // =======================
        JPSPlusNode mStart = null;
        JPSPlusNode mGoal = null;
        readonly Dictionary<Int2, JPSPlusNode> mCreatedNodes = new Dictionary<Int2, JPSPlusNode>();
        readonly PriorityQueue<(JPSPlusNode Node, EDirFlags Dir)> mOpenList = new PriorityQueue<(JPSPlusNode Node, EDirFlags Dir)>();
        readonly HashSet<JPSPlusNode> mCloseList = new HashSet<JPSPlusNode>();
        JPSPlusBakedMap mBakedMap;
    }
}
