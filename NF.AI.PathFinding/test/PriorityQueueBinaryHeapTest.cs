using System.Collections.Generic;

using Xunit;

namespace NF.Collections.Generic.Test
{
    public class PriorityQueueBinaryHeapTest
    {
        [Fact]
        public void Test1()
        {
            PriorityQueueBinaryHeap<int> q = new PriorityQueueBinaryHeap<int>();
            q.Push(20);
            q.Push(10);
            q.Push(30);
            q.Push(90);
            q.Push(40);

            List<int> lst = new List<int>();
            while (q.Count > 0)
            {
                lst.Add(q.Pop());
            }
            Assert.Equal(lst, new List<int> { 90, 40, 30, 20, 10 });
        }
    }
}
