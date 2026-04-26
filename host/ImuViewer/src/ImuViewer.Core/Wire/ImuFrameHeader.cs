using System.Runtime.InteropServices;

namespace ImuViewer.Core.Wire;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public readonly struct ImuFrameHeader
{
    public ushort Magic { get; init; }
    public byte Type { get; init; }
    public byte SampleCount { get; init; }
    public ushort SampleRateHz { get; init; }
    public ushort AccelFsrG { get; init; }
    public ushort GyroFsrDps { get; init; }
    public ushort Seq { get; init; }
    public uint FirstSampleTimestampUs { get; init; }
    public ushort FrameLen { get; init; }

    public bool IsValid =>
        Magic == WireConstants.Magic &&
        Type == WireConstants.ImuFrameType &&
        SampleCount >= WireConstants.MinSampleCount &&
        SampleCount <= WireConstants.MaxSampleCount &&
        FrameLen == WireConstants.HeaderSize + SampleCount * WireConstants.SampleSize;
}
