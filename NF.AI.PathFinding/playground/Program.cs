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

    class Program
    {
        uint WIDTH = 1000;
        uint HEIGHT = 1000;

        int NodeSize { get; } = 50;
        Board mBoard;
        E_ClickState mState = E_ClickState.None;
        Int2? mLatestP = null;

        static void Main()
        {
            //Main5();
            var program = new Program();
            program.Run();
        }

        void Run()
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

        void ResetBoard()
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

        Vector2i GetGridPosition(Window window, int nodeSize)
        {
            return Mouse.GetPosition(window) / nodeSize;
        }

        private void OnMouseButtonPressed(object sender, MouseButtonEventArgs e)
        {
            if (e.Button != Mouse.Button.Left)
            {
                return;
            }

            var window = (Window)sender;
            var mp = GetGridPosition(window, NodeSize);
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

            var window = (Window)sender;
            var mp = GetGridPosition(window, NodeSize);
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
                var window = (Window)sender;
                var mp = GetGridPosition(window, NodeSize);
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
            var window = (Window)sender;
            window.Close();
        }

        private void KeyPressed(object sender, KeyEventArgs e)
        {
            var window = (Window)sender;
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

        void FindPath()
        {
            var sw = Stopwatch.StartNew();
            mBoard.StepAll();
            sw.Stop();
            Console.WriteLine($"Step Ticks : {1000.0 * sw.ElapsedTicks / Stopwatch.Frequency}");

            mBoard.FindPath();
        }

        static bool[,] GetWalls(string[] strs)
        {
            var height = strs.Length;
            var width = strs[0].Length;
            var walls = new bool[height, width];
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

        static void Main5()
        {
            var walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });

            var jpspBaker = new JPSPlus.JPSPlusMapBaker(walls);
            var bakedMap = jpspBaker.Bake();
            var jpsp = new JPSPlus.JPSPlus();
            jpsp.Init(bakedMap);

            jpsp.SetStart(new Int2(0, 4));
            jpsp.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jpsp.StepAll();
            sw.Stop();
            Console.WriteLine($"JPSPlus Take MS: {sw.ElapsedMilliseconds}");

            foreach (var path in jpsp.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        static void Main2()
        {
            var walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            var jps = new JPS.JPS(walls);
            jps.SetStart(new Int2(0, 4));
            jps.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jps.StepAll();
            sw.Stop();
            Console.WriteLine($"JPS Take MS: {sw.ElapsedMilliseconds}");

            foreach (var path in jps.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        static void Main3()
        {
            var walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            var astar = new AStar.AStar(walls);
            astar.SetStart(new Int2(0, 4));
            astar.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = astar.StepAll();
            sw.Stop();
            Console.WriteLine($"AStar Take MS: {sw.ElapsedMilliseconds}");

            foreach (var path in astar.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        static void Main4()
        {
            var walls = GetWalls(new string[] {
                 "..X...X..",
                 "......X..",
                 ".XX...XX.",
                 "..X......",
                 "..X...X..",
            });
            var jpso = new JPSOrthogonal.JPSOrthogonal(walls);
            jpso.SetStart(new Int2(0, 4));
            jpso.SetGoal(new Int2(7, 0));

            Stopwatch sw = Stopwatch.StartNew();
            bool isOk = jpso.StepAll();
            sw.Stop();
            Console.WriteLine($"JPSOrthogonal Take MS: {sw.ElapsedMilliseconds}");

            foreach (var path in jpso.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }
    }
}
