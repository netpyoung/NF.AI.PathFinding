using System;
using System.Collections.Generic;

namespace NF.Collections.Generic
{
    public class PriorityQueueBinaryHeap<T> where T : IComparable<T>
    {
        public void Push(T t)
        {
            mHeap.Add(t);

            int now = mHeap.Count - 1;
            while (now > 0)
            {
                int next = (now - 1) / 2;
                if (mHeap[now].CompareTo(mHeap[next]) < 0)
                {
                    break;
                }
                (mHeap[next], mHeap[now]) = (mHeap[now], mHeap[next]);
                now = next;
            }
        }

        public T Pop()
        {
            T ret = mHeap[0];
            int lastIndex = mHeap.Count - 1;
            mHeap[0] = mHeap[lastIndex];
            mHeap.RemoveAt(lastIndex);
            lastIndex--;

            int now = 0;
            while (true)
            {
                int left = (2 * now) + 1;
                int right = (2 * now) + 2;

                int next = now;
                if (left <= lastIndex && mHeap[next].CompareTo(mHeap[left]) < 0)
                {
                    next = left;
                }

                if (right <= lastIndex && mHeap[next].CompareTo(mHeap[right]) < 0)
                {
                    next = right;
                }

                if (next == now)
                {
                    break;
                }

                (mHeap[next], mHeap[now]) = (mHeap[now], mHeap[next]);
                now = next;
            }

            return ret;
        }

        public int Count => mHeap.Count;

        private readonly List<T> mHeap = new List<T>();

    }
}
