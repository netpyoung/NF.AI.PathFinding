using System;
using System.Runtime.CompilerServices;

namespace NF.Mathematics
{
    [Serializable]
    public struct Int2 : IEquatable<Int2>, IFormattable
    {
        public int X;
        public int Y;

        public Int2(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }

        [System.ComponentModel.EditorBrowsable(System.ComponentModel.EditorBrowsableState.Never)]
        public Int2 XY
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new Int2(X, Y);

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                X = value.X;
                Y = value.Y;
            }
        }

        [System.ComponentModel.EditorBrowsable(System.ComponentModel.EditorBrowsableState.Never)]
        public Int2 YX
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new Int2(Y, X);

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Y = value.X;
                X = value.Y;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Int2 operator +(Int2 a, Int2 b)
        {
            return new Int2 { X = a.X + b.X, Y = a.Y + b.Y };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Int2 operator -(Int2 a, Int2 b)
        {
            return new Int2 { X = a.X - b.X, Y = a.Y - b.Y };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Int2 operator *(Int2 a, int val)
        {
            return new Int2 { X = a.X * val, Y = a.Y * val };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Int2 operator /(Int2 a, int val)
        {
            return new Int2 { X = a.X / val, Y = a.Y / val };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Int2 a, Int2 b)
        {
            return a.X == b.X && a.Y == b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Int2 a, Int2 b)
        {
            return !(a == b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return Equals((Int2)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"Int2({X}, {Y})";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Int2 other)
        {
            return this == other;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return $"Int2({X.ToString(format, formatProvider)}, {Y.ToString(format, formatProvider)})";
        }
    }
}
