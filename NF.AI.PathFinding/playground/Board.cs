using NF.AI.PathFinding.Common;
using NF.Mathematics;

using SFML.Graphics;
using SFML.System;

using System.Collections.Generic;
using System.Linq;

namespace NF.AI.PathFinding.Playground
{
    internal class Board : Drawable
    {
        private readonly DrawableNode[,] mDrawbleNodes;
        private readonly BresenHamPathSmoother mPathSmoother = new BresenHamPathSmoother();

        //AStar.AStar mPathFinder;
        //JPS.JPS mPathFinder;
        //JPSOrthogonal.JPSOrthogonal mPathFinder;
        private readonly JPSPlus.JPSPlusRunner mPathFinder;
        private readonly List<Line> mGridLines = new List<Line>();
        private readonly List<Line> mPathLines = new List<Line>();
        private Vector2f mHalfNodeP;

        public int Width { get; }
        public int Height { get; }
        public int NodeSize { get; }
        public Int2 StartP => mPathFinder.StartP;
        public Int2 GoalP => mPathFinder.GoalP;

        public Board(int screenWidth, int screenHeight, int nodeSize)
        {
            //mPathFinder = new AStar.AStar(screenWidth / nodeSize, screenHeight / nodeSize);
            //mPathFinder = new JPS.JPS(screenWidth / nodeSize, screenHeight / nodeSize);
            //mPathFinder = new JPSOrthogonal.JPSOrthogonal(screenWidth / nodeSize, screenHeight / nodeSize);
            mPathFinder = new JPSPlus.JPSPlusRunner(screenWidth / nodeSize, screenHeight / nodeSize);

            Width = screenWidth / nodeSize;
            Height = screenHeight / nodeSize;
            NodeSize = nodeSize;
            mHalfNodeP = new Vector2f(NodeSize / 2, NodeSize / 2);

            for (int x = 0; x <= screenWidth; x += nodeSize)
            {
                mGridLines.Add(new Line(new Vector2f(x, 0), new Vector2f(x, screenHeight)));
            }

            for (int y = 0; y <= screenHeight; y += nodeSize)
            {
                mGridLines.Add(new Line(new Vector2f(0, y), new Vector2f(screenWidth, y)));
            }

            mDrawbleNodes = new DrawableNode[Height, Width];
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    mDrawbleNodes[y, x] = new DrawableNode(nodeSize)
                    {
                        Position = new Vector2f(x * nodeSize, y * nodeSize)
                    };
                }
            }
        }

        public void Draw(RenderTarget target, RenderStates states)
        {
            foreach (Line line in mGridLines)
            {
                target.Draw(line);
            }
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    target.Draw(mDrawbleNodes[y, x]);
                }
            }
            foreach (Line line in mPathLines)
            {
                target.Draw(line);
            }
        }

        public void ToggleWall(in Int2 mp)
        {
            mPathFinder.ToggleWall(mp);
        }

        public void SetStart(in Int2 p)
        {
            mPathFinder.SetStart(p);
        }

        public void SetGoal(in Int2 p)
        {
            mPathFinder.SetGoal(p);
        }

        internal void StepAll()
        {
            _ = mPathFinder.StepAll();
        }

        public bool IsWall(in Int2 p)
        {
            return !mPathFinder.IsWalkable(p);
        }

        public void CreateWall(in Int2 p)
        {
            mPathFinder.SetWall(p, true);
        }

        public void RemoveWall(in Int2 p)
        {
            mPathFinder.SetWall(p, false);
        }

        internal void Update()
        {
            bool[,] walls = mPathFinder.GetWalls();
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    if (walls[y, x])
                    {
                        mDrawbleNodes[y, x].SetFillColor(Color.Blue);
                    }
                    else
                    {
                        mDrawbleNodes[y, x].SetFillColor(Color.White);
                    }
                }
            }

            Int2 sp = mPathFinder.StartP;
            mDrawbleNodes[sp.Y, sp.X].SetFillColor(Color.Yellow);
            Int2 gp = mPathFinder.GoalP;
            mDrawbleNodes[gp.Y, gp.X].SetFillColor(Color.Green);
        }

        public void FindPath()
        {
            bool[,] walls = mPathFinder.GetWalls();
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    if (walls[y, x])
                    {
                        mDrawbleNodes[y, x].SetFillColor(Color.Blue);
                    }
                    else
                    {
                        mDrawbleNodes[y, x].SetFillColor(Color.White);
                    }
                }
            }

            { // show path
                mPathLines.Clear();
                DrawableNode prevNode = null;
                List<Int2> pp1 = mPathFinder.GetPaths().Select(x =>
                {
                    return x.Position;
                }).ToList();
                //var pp2 = mPathSmoother.SmoothPath(pp1, mPathFinder.IsWalkable);
                foreach (Int2 p in pp1)
                {
                    DrawableNode pathNode = mDrawbleNodes[p.Y, p.X];
                    pathNode.SetFillColor(Color.Cyan);
                    if (prevNode != null)
                    {
                        Line line = new Line(pathNode.Position + mHalfNodeP, prevNode.Position + mHalfNodeP);
                        line.SetColor(Color.Red);
                        mPathLines.Add(line);
                    }
                    prevNode = pathNode;
                }
            } // show path

            Int2 sp = mPathFinder.StartP;
            mDrawbleNodes[sp.Y, sp.X].SetFillColor(Color.Yellow);
            Int2 gp = mPathFinder.GoalP;
            mDrawbleNodes[gp.Y, gp.X].SetFillColor(Color.Green);
        }
    }
}
