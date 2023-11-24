using SFML.Graphics;
using SFML.System;

namespace NF.AI.PathFinding.Playground
{
    internal class DrawableNode : Transformable, Drawable
    {
        private readonly int mNodeSize;
        private readonly RectangleShape mBack = new RectangleShape();

        public DrawableNode(int nodeSize)
        {
            mNodeSize = nodeSize;
            Init(nodeSize);
        }

        private void Init(int nodeSize)
        {
            mBack.Size = new Vector2f(nodeSize - 5, nodeSize - 5);
            mBack.Position = new Vector2f(2.5f, 2.5f);
        }

        public void Draw(RenderTarget target, RenderStates states)
        {
            states.Transform *= Transform;
            target.Draw(mBack, states);
        }

        public void SetFillColor(Color color)
        {
            mBack.FillColor = color;
        }
    }
}
