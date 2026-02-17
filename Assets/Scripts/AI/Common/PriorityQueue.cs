using System;
using System.Collections.Generic;

public sealed class PriorityQueue<T>
{
    private struct Entry
    {
        public T node;
        public float key;
    }

    private readonly List<Entry> heap;

    public int Count => heap.Count;

    public PriorityQueue(int initialCapacity = 256)
    {
        heap = new List<Entry>(Math.Max(4, initialCapacity));
    }

    public void Clear()
    {
        heap.Clear();
    }

    public void Push(T node, float key)
    {
        heap.Add(new Entry { node = node, key = key });
        SiftUp(heap.Count - 1);
    }

    public T PopMin()
    {
        if (heap.Count == 0)
        {
            throw new InvalidOperationException("Priority queue is empty.");
        }

        T node = heap[0].node;
        RemoveRoot();
        return node;
    }

    public bool TryPopMin(out T node, out float key)
    {
        if (heap.Count == 0)
        {
            node = default(T);
            key = 0f;
            return false;
        }

        node = heap[0].node;
        key = heap[0].key;
        RemoveRoot();
        return true;
    }

    private void RemoveRoot()
    {
        int last = heap.Count - 1;
        heap[0] = heap[last];
        heap.RemoveAt(last);

        if (heap.Count > 0)
        {
            SiftDown(0);
        }
    }

    private void SiftUp(int index)
    {
        while (index > 0)
        {
            int parent = (index - 1) >> 1;
            if (heap[parent].key <= heap[index].key)
            {
                break;
            }

            Swap(parent, index);
            index = parent;
        }
    }

    private void SiftDown(int index)
    {
        int count = heap.Count;
        while (true)
        {
            int left = (index << 1) + 1;
            if (left >= count)
            {
                break;
            }

            int right = left + 1;
            int smallest = (right < count && heap[right].key < heap[left].key) ? right : left;
            if (heap[index].key <= heap[smallest].key)
            {
                break;
            }

            Swap(index, smallest);
            index = smallest;
        }
    }

    private void Swap(int a, int b)
    {
        Entry tmp = heap[a];
        heap[a] = heap[b];
        heap[b] = tmp;
    }
}
