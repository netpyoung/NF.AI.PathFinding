using NF.AI.PathFinding.Common;
using NF.Mathematics;

using System.Collections.Generic;

namespace NF.AI.PathFinding.JPSPlus
{
    public class JPSPlusRunner
    {
        private readonly JPSPlus mJpsPlus = new JPSPlus();
        private readonly JPSPlusMapBaker mBaker = new JPSPlusMapBaker();
        private bool[,] mWalls = null;

        public int Width { get; private set; }
        public int Height { get; private set; }

        public Int2 StartP => mStartP.Value;
        public Int2 GoalP => mGoalP.Value;

        private bool mIsWallChanged = true;
        private Int2? mStartP = null;
        private Int2? mGoalP = null;

        public JPSPlusRunner(int width, int height)
        {
            Init(new bool[height, width]);
        }

        public JPSPlusRunner(bool[,] walls)
        {
            Init(walls);
        }

        public void Init(bool[,] walls)
        {
            mWalls = walls;
            Width = walls.GetLength(1);
            Height = walls.GetLength(0);
        }

        public void ToggleWall(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mWalls[p.Y, p.X] = !mWalls[p.Y, p.X];
            mIsWallChanged = true;
        }

        public void SetStart(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mStartP = p;
        }

        public void SetGoal(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mGoalP = p;
        }

        public bool IsWalkable(in Int2 p)
        {
            if (!IsInBoundary(p))
            {
                return false;
            }
            return !mWalls[p.Y, p.X];
        }

        public bool StepAll(int stepCount = int.MaxValue)
        {
            if (mIsWallChanged)
            {
                mBaker.Init(mWalls);
                mJpsPlus.Init(mBaker.Bake());
                mIsWallChanged = false;
            }
            _ = mJpsPlus.SetStart(mStartP.Value);
            _ = mJpsPlus.SetGoal(mGoalP.Value);
            return mJpsPlus.StepAll(stepCount);
        }

        public void SetWall(in Int2 p, bool isWall)
        {
            if (!IsInBoundary(p))
            {
                return;
            }
            mWalls[p.Y, p.X] = isWall;
            mIsWallChanged = true;
        }

        public bool[,] GetWalls()
        {
            return mWalls;
        }

        public IReadOnlyList<AStarNode> GetPaths()
        {
            return mJpsPlus.GetPaths();
        }

        private bool IsInBoundary(in Int2 p)
        {
            return 0 <= p.X && p.X < Width && 0 <= p.Y && p.Y < Height;
        }

    }
}
