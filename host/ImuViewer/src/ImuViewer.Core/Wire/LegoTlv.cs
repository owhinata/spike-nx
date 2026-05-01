using System.Collections.Immutable;

namespace ImuViewer.Core.Wire;

/// <summary>
/// One entry of a BUNDLE frame's TLV section.  See btsensor_wire.h.
/// </summary>
public sealed record LegoTlv(
    LegoClassId ClassId,
    /// <summary>Physical port 0..5 when <see cref="LegoTlvFlags.Bound"/> is set, 0xFF otherwise.</summary>
    byte PortId,
    byte ModeId,
    LegoDataType DataType,
    byte NumValues,
    LegoTlvFlags Flags,
    /// <summary>10 ms units since last publish, 0xFF saturated.</summary>
    byte Age10ms,
    ushort Seq,
    ImmutableArray<byte> Payload)
{
    public bool IsBound => (Flags & LegoTlvFlags.Bound) != 0;
    public bool IsFresh => (Flags & LegoTlvFlags.Fresh) != 0;
}
