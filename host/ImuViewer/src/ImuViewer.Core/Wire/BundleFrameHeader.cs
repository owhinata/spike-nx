namespace ImuViewer.Core.Wire;

/// <summary>
/// 16-byte fixed header following the 5-byte BUNDLE envelope.
/// </summary>
public readonly record struct BundleFrameHeader(
    ushort Seq,
    /// <summary>Absolute timestamp of the oldest IMU sample (or run-loop now() if no IMU samples). Mod 2^32 µs.</summary>
    uint TickTsUs,
    ushort ImuSectionLen,
    byte ImuSampleCount,
    byte TlvCount,
    ushort ImuSampleRateHz,
    byte ImuAccelFsrG,
    ushort ImuGyroFsrDps,
    byte Flags)
{
    public bool IsImuOn => (Flags & WireConstants.FlagImuOn) != 0;
    public bool IsSensorOn => (Flags & WireConstants.FlagSensorOn) != 0;
}
