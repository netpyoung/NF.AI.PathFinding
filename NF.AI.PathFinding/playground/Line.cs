using SFML.Graphics;
using SFML.System;

using System;

namespace NF.AI.PathFinding.Playground
{
    internal class Line : Drawable
    {
        public void Refresh(Vector2f p1, Vector2f p2)
        {
            P1 = p1;
            P2 = p2;
            SetTickness(Tickness);
            SetColor(Color);
        }

        public void SetTickness(float ticknessAmount)
        {
            Tickness = ticknessAmount;
            Vector2f direction = P2 - P1;
            Vector2f unitDirection = direction / (float)Math.Sqrt((direction.X * direction.X) + (direction.Y * direction.Y));
            Vector2f unitPerpendicular = new Vector2f(-unitDirection.Y, unitDirection.X);
            Vector2f offset = ticknessAmount / 2f * unitPerpendicular;

            mVertices[0].Position = P1 + offset;
            mVertices[1].Position = P2 + offset;
            mVertices[2].Position = P2 - offset;
            mVertices[3].Position = P1 - offset;
        }

        public void Draw(RenderTarget target, RenderStates states)
        {
            target.Draw(mVertices, 0, 4, PrimitiveType.Quads);
        }

        public float Tickness { get; private set; }
        public Vector2f P1 { get; private set; }
        public Vector2f P2 { get; private set; }
        public Color Color { get; private set; }

        private readonly Vertex[] mVertices = new Vertex[4];

        public void SetColor(Color color)
        {
            Color = color;
            for (int i = 0; i < 4; ++i)
            {
                mVertices[i].Color = color;
            }
        }
        public Line(Vector2f p1, Vector2f p2)
        {
            P1 = p1;
            P2 = p2;
            Tickness = 5;
            Color = Color.Black;
            Refresh(p1, p2);
        }
    }
}
