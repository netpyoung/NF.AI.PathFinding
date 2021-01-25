using NF.AI.PathFinding.Common;
using NF.Mathematics;
using SFML.Graphics;
using SFML.System;
using System.Collections.Generic;
using System.Linq;

namespace NF.AI.PathFinding.Playground
{
    class Board : Drawable
    {
        DrawableNode[,] mDrawbleNodes;
        BresenHamPathSmoother mPathSmoother = new BresenHamPathSmoother();
        //AStar.AStar mPathFinder;
        JPS.JPS mPathFinder;
        //JPSOrthogonal.JPSOrthogonal mPathFinder;
        List<Line> mGridLines = new List<Line>();
        List<Line> mPathLines = new List<Line>();
        Vector2f mHalfNodeP;

        public Board(int screenWidth, int screenHeight, int nodeSize)
        {
            //mPathFinder = new AStar.AStar(screenWidth / nodeSize, screenHeight / nodeSize);
            mPathFinder = new JPS.JPS(screenWidth / nodeSize, screenHeight / nodeSize);
            //mPathFinder = new JPSOrthogonal.JPSOrthogonal(screenWidth / nodeSize, screenHeight / nodeSize);

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
                    mDrawbleNodes[y, x] = new DrawableNode(nodeSize);
                    mDrawbleNodes[y, x].Position = new Vector2f(x * nodeSize, y * nodeSize);
                }
            }
        }

        public void Draw(RenderTarget target, RenderStates states)
        {
            foreach (var line in mGridLines)
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
            foreach (var line in mPathLines)
            {
                target.Draw(line);
            }
        }

        public int Width { get; }
        public int Height { get; }

        public int NodeSize { get; }

        public Int2 StartP => mPathFinder.GetStart().Position;
        public Int2 GoalP => mPathFinder.GetGoal().Position;

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
            mPathFinder.StepAll();
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
            var walls = mPathFinder.GetWalls();
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

            var sp = mPathFinder.GetStart().Position;
            mDrawbleNodes[sp.Y, sp.X].SetFillColor(Color.Yellow);
            var gp = mPathFinder.GetGoal().Position;
            mDrawbleNodes[gp.Y, gp.X].SetFillColor(Color.Green);
        }

        public void FindPath()
        {
            var walls = mPathFinder.GetWalls();
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
                var pp1 = mPathFinder.GetPaths().Select(x => x.Position).ToList();
                var pp2 = mPathSmoother.SmoothPath(pp1, mPathFinder.IsWalkable);
                foreach (var p in pp1)
                {
                    var pathNode = mDrawbleNodes[p.Y, p.X];
                    pathNode.SetFillColor(Color.Cyan);
                    if (prevNode != null)
                    {
                        var line = new Line(pathNode.Position + mHalfNodeP, prevNode.Position + mHalfNodeP);
                        line.SetColor(Color.Red);
                        mPathLines.Add(line);
                    }
                    prevNode = pathNode;
                }
            } // show path

            var sp = mPathFinder.GetStart().Position;
            mDrawbleNodes[sp.Y, sp.X].SetFillColor(Color.Yellow);
            var gp = mPathFinder.GetGoal().Position;
            mDrawbleNodes[gp.Y, gp.X].SetFillColor(Color.Green);
        }
    }
}