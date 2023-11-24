using NF.Mathematics;

using SFML.Graphics;
using SFML.System;
using SFML.Window;

using System;
using System.Diagnostics;

namespace NF.AI.PathFinding.Playground
{
    public enum E_ClickState
    {
        None,
        MoveStartNode,
        MoveGoalNode,
        MakeWall,
        EraseWall,
    }

    internal class Program
    {
        private readonly uint WIDTH = 1000;
        private readonly uint HEIGHT = 1000;

        private int NodeSize { get; } = 50;

        private Board mBoard;
        private E_ClickState mState = E_ClickState.None;
        private Int2? mLatestP = null;

        private static void Main()
        {
            //Main5();
            Program program = new Program();
            program.Run();
        }

        private void Run()
        {
            RenderWindow window = new RenderWindow(new VideoMode(WIDTH, HEIGHT), "sfml");
            window.KeyPressed += KeyPressed;
            window.Closed += Closed;
            window.MouseButtonPressed += OnMouseButtonPressed;
            window.MouseMoved += OnMouseMoved;
            window.MouseButtonReleased += OnMouseButtonReleased;

            ResetBoard();

            while (window.IsOpen)
            {
                window.DispatchEvents();
                window.Clear(Color.Blue);
                window.Draw(mBoard);
                window.Display();
            }
        }

        private void ResetBoard()
        {
            mBoard = new Board((int)WIDTH, (int)HEIGHT, NodeSize);
            mBoard.SetStart(new Int2(5, 5));
            mBoard.SetGoal(new Int2(11, 5));
            mBoard.ToggleWall(new Int2(6, 5));

            //mBoard.SetStart(new Int2(2, 2));
            //mBoard.SetGoal(new Int2(7, 2));
            //mBoard.ToggleWall(new Int2(6, 2));

            mBoard.StepAll();
            mBoard.Update();
        }

        private Vector2i GetGridPosition(Window window, int nodeSize)
        {
            return Mouse.GetPosition(window) / nodeSize;
        }

        private void OnMouseButtonPressed(object sender, MouseButtonEventArgs e)
        {
            if (e.Button != Mouse.Button.Left)
            {
                return;
            }

            Window window = (Window)sender;
            Vector2i mp = GetGridPosition(window, NodeSize);
            Int2 mmp = new Int2(mp.X, mp.Y);

            mLatestP = mmp;

            if (mmp == mBoard.StartP)
            {
                mState = E_ClickState.MoveStartNode;
                return;
            }

            if (mmp == mBoard.GoalP)
            {
                mState = E_ClickState.MoveGoalNode;
                return;
            }

            if (mBoard.IsWall(mmp))
            {

                mState = E_ClickState.EraseWall;
                return;
            }

            mState = E_ClickState.MakeWall;
        }

        private void OnMouseMoved(object sender, MouseMoveEventArgs e)
        {
            if (!Mouse.IsButtonPressed(Mouse.Button.Left))
            {
                return;
            }

            if (mState == E_ClickState.None)
            {
                return;
            }

            Window window = (Window)sender;
            Vector2i mp = GetGridPosition(window, NodeSize);
            Int2 mmp = new Int2(mp.X, mp.Y);
            switch (mState)
            {
                case E_ClickState.MoveStartNode:
                {
                    if (!mBoard.IsWall(mmp))
                    {
                        mBoard.SetStart(mmp);
                    }
                }
                break;
                case E_ClickState.MoveGoalNode:
                {
                    if (!mBoard.IsWall(mmp))
                    {
                        mBoard.SetGoal(mmp);
                    }
                }
                break;
                case E_ClickState.MakeWall:
                {
                    if (mmp != mBoard.StartP && mmp != mBoard.GoalP)
                    {
                        mBoard.CreateWall(mmp);
                    }
                }
                break;
                case E_ClickState.EraseWall:
                {
                    if (mmp != mBoard.StartP && mmp != mBoard.GoalP)
                    {
                        mBoard.RemoveWall(mmp);
                    }
                }
                break;
                default:
                    break;
            }
            mBoard.Update();
        }

        private void OnMouseButtonReleased(object sender, MouseButtonEventArgs e)
        {
            if (e.Button != Mouse.Button.Left)
            {
                return;
            }

            if (mState == E_ClickState.EraseWall || mState == E_ClickState.MakeWall)
            {
                Window window = (Window)sender;
                Vector2i mp = GetGridPosition(window, NodeSize);
                Int2 mmp = new Int2(mp.X, mp.Y);
                if (mLatestP == mmp)
                {
                    if (mState == E_ClickState.EraseWall)
                    {
                        mBoard.RemoveWall(mmp);
                    }
                    else
                    {
                        mBoard.CreateWall(mmp);
                    }
                }
                mBoard.Update();
            }
            mLatestP = null;
            mState = E_ClickState.None;
        }

        private void Closed(object sender, EventArgs e)
        {
            Window window = (Window)sender;
            window.Close();
        }

        private void KeyPressed(object sender, KeyEventArgs e)
        {
            Window window = (Window)sender;
            switch (e.Code)
            {
                case Keyboard.Key.Escape:
                    window.Close();
                    break;
                case Keyboard.Key.Space:
                    FindPath();
                    break;
                case Keyboard.Key.R:
                    ResetBoard();
                    break;
                default:
                    break;
            }

        }

        private void FindPath()
        {
            Stopwatch sw = Stopwatch.StartNew();
            mBoard.StepAll();
            sw.Stop();
            Console.WriteLine($"Step Ticks : {1000.0 * sw.ElapsedTicks / Stopwatch.Frequency}");

            mBoard.FindPath();
        }

        private static bool[,] GetWalls(string[] strs)
        {
            int height = strs.Length;
            int width = strs[0].Length;
            bool[,] walls = new bool[height, width];
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    switch (strs[y][x])
                    {
                        case 'X':
                            walls[y, x] = true;
                            break;
                    }
                }
            }
            return walls;
        }

        private static void Main5()
        {
            bool[,] walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });

            JPSPlus.JPSPlusMapBaker jpspBaker = new JPSPlus.JPSPlusMapBaker(walls);
            JPSPlus.JPSPlusBakedMap bakedMap = jpspBaker.Bake();
            JPSPlus.JPSPlus jpsp = new JPSPlus.JPSPlus();
            jpsp.Init(bakedMap);

            _ = jpsp.SetStart(new Int2(0, 4));
            _ = jpsp.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jpsp.StepAll();
            sw.Stop();
            Console.WriteLine($"JPSPlus Take MS: {sw.ElapsedMilliseconds}");

            foreach (Common.AStarNode path in jpsp.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        private static void Main2()
        {
            bool[,] walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            JPS.JPS jps = new JPS.JPS(walls);
            jps.SetStart(new Int2(0, 4));
            jps.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jps.StepAll();
            sw.Stop();
            Console.WriteLine($"JPS Take MS: {sw.ElapsedMilliseconds}");

            foreach (Common.AStarNode path in jps.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        private static void Main3()
        {
            bool[,] walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            AStar.AStar astar = new AStar.AStar(walls);
            _ = astar.SetStart(new Int2(0, 4));
            _ = astar.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = astar.StepAll();
            sw.Stop();
            Console.WriteLine($"AStar Take MS: {sw.ElapsedMilliseconds}");

            foreach (Common.AStarNode path in astar.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        private static void Main4()
        {
            bool[,] walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            JPSOrthogonal.JPSOrthogonal jpso = new JPSOrthogonal.JPSOrthogonal(walls);
            _ = jpso.SetStart(new Int2(0, 4));
            _ = jpso.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jpso.StepAll();
            sw.Stop();
            Console.WriteLine($"JPSOrthogonal Take MS: {sw.ElapsedMilliseconds}");

            foreach (Common.AStarNode path in jpso.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }
    }
}
