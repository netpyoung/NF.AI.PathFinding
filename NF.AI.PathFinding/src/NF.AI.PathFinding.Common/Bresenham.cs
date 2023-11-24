using NF.Mathematics;

using System;

namespace NF.AI.PathFinding.Common
{
    public class BresenHam
    {
        // ref: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        private int mDx;
        private int mSx;
        private int mDy;
        private int mSy;
        private int mErr;
        private Int2 mCurr;
        private Int2 mDest;

        public void Init(in Int2 src, in Int2 dst)
        {
            mDx = Math.Abs(dst.X - src.X);
            mDy = -Math.Abs(dst.Y - src.Y);
            mSx = (src.X < dst.X) ? 1 : -1;
            mSy = (src.Y < dst.Y) ? 1 : -1;
            mErr = mDx + mDy;
            mCurr = src;
            mDest = dst;
        }

        public bool TryGetNext(ref Int2 nextP)
        {
            if (mCurr == mDest)
            {
                return false;
            }

            int e2 = 2 * mErr;

            if (e2 >= mDy)
            {
                mErr += mDy;
                mCurr.X += mSx;
            }
            if (e2 <= mDx)
            {
                mErr += mDx;
                mCurr.Y += mSy;
            }

            nextP = mCurr;
            return true;
        }
    }
}
