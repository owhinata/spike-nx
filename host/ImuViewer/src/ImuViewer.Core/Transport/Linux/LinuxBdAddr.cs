namespace ImuViewer.Core.Transport.Linux;

internal static class LinuxBdAddr
{
    /// <summary>
    /// Encode "F8:2E:0C:A0:3E:64" as a 6-byte bdaddr_t (little-endian, the
    /// human-readable LSB is byte[0]).
    /// </summary>
    public static byte[] Encode(string bdAddr)
    {
        ArgumentException.ThrowIfNullOrEmpty(bdAddr);
        string[] parts = bdAddr.Split(':');
        if (parts.Length != 6)
        {
            throw new ArgumentException($"invalid BD_ADDR: {bdAddr}", nameof(bdAddr));
        }
        byte[] bytes = new byte[6];
        for (int i = 0; i < 6; i++)
        {
            if (!byte.TryParse(parts[i], System.Globalization.NumberStyles.HexNumber,
                               System.Globalization.CultureInfo.InvariantCulture, out byte b))
            {
                throw new ArgumentException($"invalid BD_ADDR octet '{parts[i]}'", nameof(bdAddr));
            }
            bytes[5 - i] = b;
        }
        return bytes;
    }
}
