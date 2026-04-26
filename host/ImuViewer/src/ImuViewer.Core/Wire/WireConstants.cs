namespace ImuViewer.Core.Wire;

public static class WireConstants
{
    public const ushort Magic = 0xB66B;
    public const byte ImuFrameType = 0x01;
    public const int HeaderSize = 18;
    public const int SampleSize = 16;
    public const int MinSampleCount = 1;
    public const int MaxSampleCount = 80;
}
