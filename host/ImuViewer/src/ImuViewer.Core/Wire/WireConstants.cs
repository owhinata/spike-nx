namespace ImuViewer.Core.Wire;

/// <summary>
/// Wire-format constants for the BUNDLE frame (Issue #88).
/// btsensor emits one BUNDLE every 10 ms (100 Hz) carrying:
/// envelope (5 B) + bundle header (16 B) + N IMU samples (16 B each)
/// + exactly TlvCount LEGO sensor TLV entries.
/// </summary>
public static class WireConstants
{
    public const ushort Magic = 0xB66B;

    public const byte BundleFrameType = 0x02;

    public const int BundleEnvelopeSize = 5;
    public const int BundleHeaderSize = 16;
    public const int ImuSampleSize = 16;
    public const int TlvHeaderSize = 10;
    public const int MaxTlvPayload = 32;

    public const int TlvCount = 6;
    public const int MaxImuSamplesPerBundle = 8;

    /// <summary>Worst-case BUNDLE frame size with full IMU + 6 fresh TLVs.</summary>
    public const int MaxBundleFrameSize =
        BundleEnvelopeSize + BundleHeaderSize
        + ImuSampleSize * MaxImuSamplesPerBundle
        + (TlvHeaderSize + MaxTlvPayload) * TlvCount;

    /// <summary>Bundle header.flags bits.</summary>
    public const byte FlagImuOn = 0x01;
    public const byte FlagSensorOn = 0x02;
}
