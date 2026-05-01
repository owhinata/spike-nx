namespace ImuViewer.Core.Wire;

/// <summary>
/// Mirrors firmware-side <c>enum legosensor_class_e</c> (boards/spike-prime-hub
/// /include/board_legosensor.h).  Order is wire-stable: the BUNDLE TLV section
/// always emits 6 entries in this order so the host can index by enum value.
/// </summary>
public enum LegoClassId : byte
{
    Color = 0,
    Ultrasonic = 1,
    Force = 2,
    MotorM = 3,
    MotorR = 4,
    MotorL = 5,
}

/// <summary>
/// Mirrors firmware-side <c>BTSENSOR_TLV_FLAG_*</c>.
/// </summary>
[Flags]
public enum LegoTlvFlags : byte
{
    None = 0,
    /// <summary>Class topic has at least one stored sample (a port has bound to it).</summary>
    Bound = 0x01,
    /// <summary>This bundle's TLV carries a payload that was published since the last bundle.</summary>
    Fresh = 0x02,
}

/// <summary>
/// Wire-stable scalar type encoding for the TLV payload.
/// </summary>
public enum LegoDataType : byte
{
    Int8 = 0,
    Int16 = 1,
    Int32 = 2,
    Float = 3,
}
