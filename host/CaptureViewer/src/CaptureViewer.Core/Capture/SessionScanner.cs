using System.Buffers.Binary;
using System.Text;

namespace CaptureViewer.Core.Capture;

/// <summary>
/// Result of a single session-scan attempt.  See
/// <see cref="SessionScanner.TryScan"/>.
/// </summary>
/// <param name="StartIndex">Byte offset of the BTCS marker that
/// produced this session inside the input buffer.</param>
/// <param name="EndIndex">Byte offset just past the terminator (BTCE
/// or BTAB).  Callers can drop everything up to this index from their
/// rolling buffer.</param>
/// <param name="Capture">The parsed capture; payload is detached.</param>
public sealed record SessionScan(
    int StartIndex,
    int EndIndex,
    CaptureFile Capture);

/// <summary>
/// Stateless scanner that walks a byte buffer and pulls out one
/// complete BTCS+meta+payload+(BTCE|BTAB) session at a time.  The
/// design plan calls this the "host-side scanner" — it is responsible
/// for rejecting false-positive BTCS matches (e.g. a four-byte run
/// that happened to align with "BTCS" inside random binary noise).
///
/// Consumers feed bytes into a <see cref="MemoryStream"/> or rolling
/// buffer, call <see cref="TryScan"/> after each receive, and slice
/// off everything up to <see cref="SessionScan.EndIndex"/> when it
/// returns true.
/// </summary>
public static class SessionScanner
{
    /// <summary>
    /// Find the next complete capture session in <paramref name="buffer"/>.
    /// </summary>
    /// <param name="buffer">Rolling byte buffer the caller has been
    /// accumulating from BT/RFCOMM.  Read-only — the scanner does not
    /// modify it.</param>
    /// <param name="resolveSchema">Optional callback that returns
    /// <c>true</c> if the supplied <c>schema_magic</c> is one the host
    /// recognizes.  Used as part of the sanity check that rejects
    /// false-positive BTCS matches.  Pass <c>null</c> to accept all
    /// magic values (useful in tests with synthetic data).</param>
    /// <param name="result">On success, the parsed session.</param>
    /// <returns><c>true</c> if a complete session was found.</returns>
    public static bool TryScan(
        ReadOnlySpan<byte> buffer,
        Func<ushort, bool>? resolveSchema,
        out SessionScan? result)
    {
        result = null;
        var startSearch = 0;
        while (true)
        {
            var idx = IndexOf(buffer, startSearch, CaptureProtocol.BTCS);
            if (idx < 0)
                return false;

            var metaOff = idx + CaptureProtocol.BTCS.Length;
            if (metaOff + CaptureProtocol.SessionMetaSize > buffer.Length)
            {
                // Not enough bytes for the meta yet — we keep this
                // BTCS and let the caller buffer more.
                return false;
            }

            var meta = buffer.Slice(metaOff, CaptureProtocol.SessionMetaSize);
            var schemaMagic = BinaryPrimitives.ReadUInt16LittleEndian(meta[..2]);
            // skip 2 reserved bytes
            var totalBytes = (int)BinaryPrimitives.ReadUInt32LittleEndian(meta.Slice(4, 4));
            var name = CaptureFile.ReadFixedString(meta.Slice(8, CaptureProtocol.SchemaNameMax));

            if (!IsPlausible(schemaMagic, totalBytes, name, resolveSchema))
            {
                // Reject this BTCS and advance one byte.  A real
                // marker that follows will still be picked up.
                startSearch = idx + 1;
                continue;
            }

            var payloadOff = metaOff + CaptureProtocol.SessionMetaSize;
            var payloadEnd = payloadOff + totalBytes;
            if (payloadEnd + 4 > buffer.Length)
            {
                // Need more bytes; keep the BTCS, ask for more data.
                return false;
            }

            var terminator = buffer.Slice(payloadEnd, 4);
            CaptureTermination disposition;
            if (terminator.SequenceEqual(CaptureProtocol.BTCE))
            {
                disposition = CaptureTermination.Clean;
            }
            else if (terminator.SequenceEqual(CaptureProtocol.BTAB))
            {
                disposition = CaptureTermination.Aborted;
            }
            else
            {
                // BTCS+meta passed the sanity check but the terminator
                // doesn't match — most likely we mis-aligned on a
                // false BTCS hit.  Drop this candidate and rescan.
                startSearch = idx + 1;
                continue;
            }

            CaptureFile capture;
            try
            {
                capture = CaptureFile.Parse(
                    buffer.Slice(payloadOff, totalBytes), disposition);
            }
            catch (InvalidCaptureFileException)
            {
                // Internal payload was inconsistent with itself even
                // though the framing checked out — treat the same as a
                // false BTCS.
                startSearch = idx + 1;
                continue;
            }

            result = new SessionScan(idx, payloadEnd + 4, capture);
            return true;
        }
    }

    private static bool IsPlausible(
        ushort schemaMagic,
        int totalBytes,
        string name,
        Func<ushort, bool>? resolveSchema)
    {
        if (totalBytes < CaptureProtocol.FileHeaderSize)
            return false;
        if (totalBytes > CaptureProtocol.PayloadSanityMaxBytes)
            return false;

        if (string.IsNullOrEmpty(name))
            return false;
        foreach (var ch in name)
        {
            // Schema names are snake_case ASCII.  Reject anything
            // outside [A-Za-z0-9_] to filter out random bytes that
            // happened to start with a letter.
            if (!(ch == '_' || char.IsLetterOrDigit(ch)))
                return false;
        }

        if (resolveSchema is not null && !resolveSchema(schemaMagic))
            return false;

        return true;
    }

    private static int IndexOf(ReadOnlySpan<byte> haystack, int start, ReadOnlySpan<byte> needle)
    {
        if (start >= haystack.Length)
            return -1;
        var hit = haystack[start..].IndexOf(needle);
        return hit < 0 ? -1 : start + hit;
    }
}
