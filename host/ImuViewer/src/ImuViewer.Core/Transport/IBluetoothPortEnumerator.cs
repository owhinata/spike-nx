namespace ImuViewer.Core.Transport;

public interface IBluetoothPortEnumerator
{
    Task<IReadOnlyList<BluetoothPort>> GetPairedPortsAsync(CancellationToken ct);
}
