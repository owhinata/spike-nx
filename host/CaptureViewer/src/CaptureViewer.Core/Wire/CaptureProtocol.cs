using System.Text;

namespace CaptureViewer.Core;

/// <summary>
/// Wire-format constants and limits for the capture pipeline (Issue #122).
/// Mirrors apps/btsensor/btsensor_capture_mode.c (BT framing) and
/// apps/capture/include/capture_format.h (.cap layout).
/// </summary>
public static class CaptureProtocol
{
    /// <summary>Session-start marker on the BT byte stream.</summary>
    public static readonly byte[] BTCS = Encoding.ASCII.GetBytes("BTCS");

    /// <summary>Session-end marker (clean finish).</summary>
    public static readonly byte[] BTCE = Encoding.ASCII.GetBytes("BTCE");

    /// <summary>Session-end marker (truncated / aborted).</summary>
    public static readonly byte[] BTAB = Encoding.ASCII.GetBytes("BTAB");

    /// <summary>".cap" file-header magic — "CAPB" little-endian.</summary>
    public const uint FileMagic = 0x42504143u;

    /// <summary>Current .cap file format version.</summary>
    public const ushort FileVersion = 1;

    /// <summary>Bytes in the session-meta blob between BTCS and the payload.</summary>
    public const int SessionMetaSize = 40;

    /// <summary>Bytes in the on-disk file header (capture_file_header_s).</summary>
    public const int FileHeaderSize = 64;

    /// <summary>Bytes per field descriptor (capture_field_desc_s).</summary>
    public const int FieldDescriptorSize = 48;

    /// <summary>Maximum schema name length (bytes), excluding any null padding.</summary>
    public const int SchemaNameMax = 32;

    /// <summary>Maximum field name length (bytes).</summary>
    public const int FieldNameMax = 16;

    /// <summary>Maximum unit string length (bytes).</summary>
    public const int FieldUnitMax = 16;

    /// <summary>
    /// Hard upper bound on a session payload accepted by the host
    /// scanner.  Exists purely to reject false-positive BTCS matches
    /// found in random binary noise.  Sized at 1 MiB which is well
    /// above any legitimate capture (CONFIG_APP_CAPTURE_MAX_HEAP_BYTES
    /// defaults to 64 KiB on the firmware side).
    /// </summary>
    public const int PayloadSanityMaxBytes = 1024 * 1024;

    /// <summary>
    /// Decode the on-the-wire "BTCS magic" sequence to a 4-character
    /// string for diagnostic output (no allocation in hot paths — only
    /// callers that already failed should use this).
    /// </summary>
    public static string MarkerToString(ReadOnlySpan<byte> marker) =>
        marker.Length == 4 ? Encoding.ASCII.GetString(marker) : "<invalid>";
}
