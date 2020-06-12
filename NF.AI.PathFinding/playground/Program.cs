using System;
using NF.Mathematics;

namespace NF.AI.PathFinding.Playground
{
    class Program
    {
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

        static void Main(string[] args)
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

            bool isOk = jpsp.StepAll();
            foreach (var path in jpsp.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }

        static void Main2(string[] args)
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
            bool isOk = jps.StepAll();
            foreach (var path in jps.GetPaths())
            {
                Console.WriteLine(path.Position);
            }
            Console.WriteLine(isOk);
            Console.WriteLine("Hello World!");
        }
    }
}
