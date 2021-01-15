using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using NF.Mathematics;
using SFML.Graphics;
using SFML.System;
using SFML.Window;

namespace NF.AI.PathFinding.Playground
{
    class Program
    {
        int NodeSize { get; } = 50;
        Clock mClock = new Clock();
        Board mBoard;

        static void Main()
        {
            var program = new Program();
            program.Run();
        }

        void Run()
        {
            uint width = 1000;
            uint height = 1000;
            RenderWindow window = new RenderWindow(new VideoMode(width, height), "sfml");
            window.KeyPressed += KeyPressed;
            window.Closed += Closed;
            window.MouseButtonReleased += MouseButtonReleased;

            mBoard = new Board((int)width, (int)height, NodeSize);
            mBoard.SetStart(new Int2(0, 0));
            mBoard.SetGoal(new Int2(10, 10));
            mBoard.StepAll();
            mBoard.Update();

            while (window.IsOpen)
            {
                window.DispatchEvents();
                window.Clear(Color.Blue);
                window.Draw(mBoard);
                window.Display();
            }
        }
        Vector2i GetGridPosition(Window window, int nodeSize)
        {
            return Mouse.GetPosition(window) / nodeSize;
        }

        private void MouseButtonReleased(object sender, MouseButtonEventArgs e)
        {
            var window = (Window)sender;

            if (e.Button == Mouse.Button.Left)
            {
                var mp = GetGridPosition(window, NodeSize);
                mBoard.ToggleWall(new Int2(mp.X, mp.Y));

                var sw = Stopwatch.StartNew();
                mBoard.StepAll();
                sw.Stop();
                Console.WriteLine($"Step Ticks : {sw.ElapsedTicks}");
                
                mBoard.Update();
            }
        }

        private void Closed(object sender, EventArgs e)
        {
            var window = (Window)sender;
            window.Close();
        }

        private void KeyPressed(object sender, KeyEventArgs e)
        {
            var window = (Window)sender;
            if (e.Code == Keyboard.Key.Escape)
            {
                window.Close();
            }
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
