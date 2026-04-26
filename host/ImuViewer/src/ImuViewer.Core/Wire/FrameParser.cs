using System.Buffers.Binary;

namespace ImuViewer.Core.Wire;

/// <summary>
/// Resync-capable streaming parser for btsensor SPP frames.
/// Mirrors the Python reference parser in docs/development/pc-receive-spp.md:
/// scan for magic, validate header, drop one byte and retry on validation failure.
/// </summary>
public sealed class FrameParser
{
    private const int InitialCapacity = 4096;
    private byte[] _buffer = new byte[InitialCapacity];
    private int _length;

    public int BufferedByteCount => _length;

    public void Append(ReadOnlySpan<byte> bytes)
    {
        EnsureCapacity(_length + bytes.Length);
        bytes.CopyTo(_buffer.AsSpan(_length));
        _length += bytes.Length;
    }

    public bool TryReadFrame(out ImuFrame frame)
    {
        while (_length >= WireConstants.HeaderSize)
        {
            int magicAt = FindMagic();
            if (magicAt < 0)
            {
                // Keep at most the last byte, which might be the first byte of magic.
                int keep = Math.Min(_length, 1);
                ShiftLeft(_length - keep);
                frame = null!;
                return false;
            }

            if (magicAt > 0)
            {
                ShiftLeft(magicAt);
            }

            if (_length < WireConstants.HeaderSize)
            {
                frame = null!;
                return false;
            }

            ImuFrameHeader header = ReadHeader(_buffer.AsSpan(0, WireConstants.HeaderSize));
            if (!header.IsValid)
            {
                // Drop one byte and try resync after this magic occurrence.
                ShiftLeft(1);
                continue;
            }

            if (_length < header.FrameLen)
            {
                frame = null!;
                return false;
            }

            int sampleCount = header.SampleCount;
            ImuSample[] samples = new ImuSample[sampleCount];
            for (int i = 0; i < sampleCount; i++)
            {
                int off = WireConstants.HeaderSize + i * WireConstants.SampleSize;
                samples[i] = ReadSample(_buffer.AsSpan(off, WireConstants.SampleSize));
            }

            frame = new ImuFrame(header, [.. samples]);
            ShiftLeft(header.FrameLen);
            return true;
        }

        frame = null!;
        return false;
    }

    private int FindMagic()
    {
        ReadOnlySpan<byte> span = _buffer.AsSpan(0, _length);
        // Magic 0xB66B in little-endian byte order: 0x6B 0xB6
        for (int i = 0; i + 1 < span.Length; i++)
        {
            if (span[i] == 0x6B && span[i + 1] == 0xB6)
            {
                return i;
            }
        }
        return -1;
    }

    private static ImuFrameHeader ReadHeader(ReadOnlySpan<byte> bytes) => new()
    {
        Magic = BinaryPrimitives.ReadUInt16LittleEndian(bytes[0..2]),
        Type = bytes[2],
        SampleCount = bytes[3],
        SampleRateHz = BinaryPrimitives.ReadUInt16LittleEndian(bytes[4..6]),
        AccelFsrG = BinaryPrimitives.ReadUInt16LittleEndian(bytes[6..8]),
        GyroFsrDps = BinaryPrimitives.ReadUInt16LittleEndian(bytes[8..10]),
        Seq = BinaryPrimitives.ReadUInt16LittleEndian(bytes[10..12]),
        FirstSampleTimestampUs = BinaryPrimitives.ReadUInt32LittleEndian(bytes[12..16]),
        FrameLen = BinaryPrimitives.ReadUInt16LittleEndian(bytes[16..18]),
    };

    private static ImuSample ReadSample(ReadOnlySpan<byte> bytes) => new(
        BinaryPrimitives.ReadInt16LittleEndian(bytes[0..2]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[2..4]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[4..6]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[6..8]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[8..10]),
        BinaryPrimitives.ReadInt16LittleEndian(bytes[10..12]),
        BinaryPrimitives.ReadUInt32LittleEndian(bytes[12..16]));

    private void ShiftLeft(int count)
    {
        if (count <= 0)
        {
            return;
        }
        if (count >= _length)
        {
            _length = 0;
            return;
        }
        Buffer.BlockCopy(_buffer, count, _buffer, 0, _length - count);
        _length -= count;
    }

    private void EnsureCapacity(int required)
    {
        if (_buffer.Length >= required)
        {
            return;
        }
        int newSize = _buffer.Length;
        while (newSize < required)
        {
            newSize *= 2;
        }
        Array.Resize(ref _buffer, newSize);
    }
}
