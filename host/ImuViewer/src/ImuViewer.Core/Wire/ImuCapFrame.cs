namespace ImuViewer.Core.Wire;

/// <summary>
/// One IMU_CAP frame payload (Phase 2.5, Issue #145).  Emitted by
/// btsensor_imu_cap_mode at 104 Hz during a Tedaldi calibration
/// session; one sample per frame so the host can detect drops with
/// single-sample granularity via the seq counter.
///
/// All axis fields are raw int16 LSB; the host pipeline keeps them in
/// raw units and lets imu_tk fit the calibration internally.
/// </summary>
public readonly record struct ImuCapFrame(
    uint TimestampUs,
    short Ax,
    short Ay,
    short Az,
    short Gx,
    short Gy,
    short Gz,
    short TemperatureRaw,
    byte FsrXlIdx,
    byte FsrGyIdx,
    ushort Seq);
