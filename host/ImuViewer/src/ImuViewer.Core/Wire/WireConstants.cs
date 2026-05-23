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

    /// <summary>
    /// Phase 2.5 (#145) IMU capture frame.  Emitted by
    /// btsensor_imu_cap_mode at 104 Hz during a Tedaldi calibration
    /// session; one sample per frame (no batching) so seq gaps map
    /// one-to-one to dropped samples for the host's reject decision.
    /// </summary>
    public const byte ImuCapFrameType = 0x03;

    public const int BundleEnvelopeSize = 5;
    public const int BundleHeaderSize = 16;
    public const int ImuSampleSize = 16;
    public const int TlvHeaderSize = 10;
    public const int MaxTlvPayload = 32;

    public const int TlvCount = 6;
    public const int MaxImuSamplesPerBundle = 8;

    /// <summary>
    /// IMU_CAP payload size (22 B).  Layout — must match
    /// apps/btsensor/btsensor_wire.h on the Hub side:
    ///
    ///   +0   timestamp_us  (uint32 LE)
    ///   +4   ax,ay,az      (int16 LE × 3, raw LSB)
    ///   +10  gx,gy,gz      (int16 LE × 3, raw LSB)
    ///   +16  temp_raw      (int16 LE, T_c = 25 + raw/256)
    ///   +18  fsr_xl_idx    (uint8, lsm6dsl_fsr_xl_e)
    ///   +19  fsr_gy_idx    (uint8, lsm6dsl_fsr_gy_e)
    ///   +20  seq           (uint16 LE, monotonic)
    /// </summary>
    public const int ImuCapPayloadSize = 22;

    /// <summary>Full IMU_CAP frame size including the 5 B envelope.</summary>
    public const int ImuCapFrameSize = BundleEnvelopeSize + ImuCapPayloadSize;

    /// <summary>Worst-case BUNDLE frame size with full IMU + 6 fresh TLVs.</summary>
    public const int MaxBundleFrameSize =
        BundleEnvelopeSize + BundleHeaderSize
        + ImuSampleSize * MaxImuSamplesPerBundle
        + (TlvHeaderSize + MaxTlvPayload) * TlvCount;

    /// <summary>Bundle header.flags bits.</summary>
    public const byte FlagImuOn = 0x01;
    public const byte FlagSensorOn = 0x02;
}
