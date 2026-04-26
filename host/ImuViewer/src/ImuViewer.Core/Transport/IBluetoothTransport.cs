namespace ImuViewer.Core.Transport;

public interface IBluetoothTransport : IAsyncDisposable
{
    Task<Stream> ConnectAsync(string bdAddr, int channel, CancellationToken ct);
}
