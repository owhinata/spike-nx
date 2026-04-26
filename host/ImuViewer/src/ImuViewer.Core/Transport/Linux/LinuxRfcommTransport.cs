using System.Runtime.InteropServices;

namespace ImuViewer.Core.Transport.Linux;

public sealed class LinuxRfcommTransport : IBluetoothTransport
{
    public static bool IsSupported =>
        RuntimeInformation.IsOSPlatform(OSPlatform.Linux);

    public Task<Stream> ConnectAsync(string bdAddr, int channel, CancellationToken ct)
    {
        if (!IsSupported)
        {
            throw new PlatformNotSupportedException(
                "LinuxRfcommTransport requires Linux + BlueZ.");
        }
        ct.ThrowIfCancellationRequested();
        return Task.Run<Stream>(() =>
        {
            LinuxRfcommSocket sock = LinuxRfcommSocket.Connect(bdAddr, channel);
            return new LinuxRfcommStream(sock);
        }, ct);
    }

    public ValueTask DisposeAsync() => ValueTask.CompletedTask;
}
