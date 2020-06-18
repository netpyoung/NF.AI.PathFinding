using System;
using System.Collections.Generic;
using NF.Mathematics;
using SFML.Graphics;
using SFML.System;

namespace NF.AI.PathFinding.Playground
{
    class Board : Drawable
    {
        JPS.JPS jps;
        //JPSOrthogonal.JPSOrthogonal jps;
        public Board(int screenWidth, int screenHeight, int nodeSize)
        {
            jps = new JPS.JPS(screenWidth / nodeSize, screenHeight / nodeSize);
            //jps = new JPSOrthogonal.JPSOrthogonal(screenWidth / nodeSize, screenHeight / nodeSize);
            Width = screenWidth / nodeSize;
            Height = screenHeight / nodeSize;
            NodeSize = nodeSize;

            for (int x = 0; x <= screenWidth; x += nodeSize)
            {
                mGridLines.Add(new Line(new Vector2f(x, 0), new Vector2f(x, screenHeight)));
            }

            for (int y = 0; y <= screenHeight; y += nodeSize)
            {
                mGridLines.Add(new Line(new Vector2f(0, y), new Vector2f(screenWidth, y)));
            }

            nodes = new DrawableNode[Height, Width];
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    nodes[y, x] = new DrawableNode(nodeSize);
                    nodes[y, x].Position = new Vector2f(x * nodeSize, y * nodeSize);
                }
            }
        }
        DrawableNode[,] nodes;
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
                    target.Draw(nodes[y, x]);
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

        List<Line> mGridLines = new List<Line>();

        internal void ToggleWall(Int2 mp)
        {
            jps.ToggleWall(mp);
        }

        public void SetStart(Int2 p)
        {
            jps.SetStart(p);
        }

        public void SetGoal(Int2 p)
        {
            jps.SetGoal(p);
        }

        internal void StepAll()
        {
            jps.StepAll();
        }

        internal void Update()
        {
            foreach (var path in jps.GetPaths())
            {
                Console.WriteLine(path.Position);
            }

            var walls = jps.GetWalls();
            for (int y = 0; y < Height; ++y)
            {
                for (int x = 0; x < Width; ++x)
                {
                    if (walls[y, x])
                    {
                        nodes[y, x].SetFillColor(Color.Blue);
                    }
                    else
                    {
                        nodes[y, x].SetFillColor(Color.White);
                    }
                }
            }


            mPathLines.Clear();
            DrawableNode prevNode = null;
            var halfNodeP = new Vector2f(NodeSize / 2, NodeSize / 2);
            foreach (var path in jps.GetPaths())
            {
                var pathNode = nodes[path.Position.Y, path.Position.X];
                pathNode.SetFillColor(Color.Cyan);
                if (prevNode != null)
                {
                    var line = new Line(pathNode.Position + halfNodeP, prevNode.Position + halfNodeP);
                    line.SetColor(Color.Red);
                    mPathLines.Add(line);
                }
                prevNode = pathNode;
            }
            var sp = jps.GetStart().Position;
            nodes[sp.Y, sp.X].SetFillColor(Color.Yellow);
            var gp = jps.GetGoal().Position;
            nodes[gp.Y, gp.X].SetFillColor(Color.Green);

        }
        List<Line> mPathLines = new List<Line>();
    }
}
