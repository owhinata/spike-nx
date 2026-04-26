namespace ImuViewer.Core.Transport.Stub;

public sealed class WindowsTransportStub : IBluetoothTransport
{
    public Task<Stream> ConnectAsync(string bdAddr, int channel, CancellationToken ct) =>
        throw new PlatformNotSupportedException(
            "Windows RFCOMM (WinRT / 32feet.NET) is tracked by a future issue; only Linux is implemented in the PoC.");

    public ValueTask DisposeAsync() => ValueTask.CompletedTask;
}
