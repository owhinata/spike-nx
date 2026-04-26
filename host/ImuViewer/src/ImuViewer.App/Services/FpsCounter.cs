using System.Diagnostics;

namespace ImuViewer.App.Services;

internal sealed class FpsCounter
{
    private const int WindowSize = 60;
    private readonly long[] _timestamps = new long[WindowSize];
    private int _count;
    private int _head;
    private readonly object _lock = new();

    public void Mark()
    {
        long ts = Stopwatch.GetTimestamp();
        lock (_lock)
        {
            _timestamps[_head] = ts;
            _head = (_head + 1) % WindowSize;
            if (_count < WindowSize)
            {
                _count++;
            }
        }
    }

    public double Compute()
    {
        lock (_lock)
        {
            if (_count < 2)
            {
                return 0;
            }
            int oldest = (_head - _count + WindowSize) % WindowSize;
            int newest = (_head - 1 + WindowSize) % WindowSize;
            long deltaTicks = _timestamps[newest] - _timestamps[oldest];
            if (deltaTicks <= 0)
            {
                return 0;
            }
            double seconds = deltaTicks / (double)Stopwatch.Frequency;
            return (_count - 1) / seconds;
        }
    }

    public void Reset()
    {
        lock (_lock)
        {
            _count = 0;
            _head = 0;
        }
    }
}
