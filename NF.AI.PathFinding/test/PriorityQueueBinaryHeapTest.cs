using Xunit;
using System.Collections.Generic;

namespace NF.Collections.Generic.Test
{
    public class PriorityQueueBinaryHeapTest
    {
        [Fact]
        public void Test1()
        {
            var q = new PriorityQueueBinaryHeap<int>();
            q.Push(20);
            q.Push(10);
            q.Push(30);
            q.Push(90);
            q.Push(40);

            var lst = new List<int>();
            while (q.Count > 0)
            {
                lst.Add(q.Pop());
            }
            Assert.Equal(lst, new List<int> { 90, 40, 30, 20, 10 });
        }
    }
}
