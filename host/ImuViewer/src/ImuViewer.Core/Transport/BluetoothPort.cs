namespace ImuViewer.Core.Transport;

public sealed record BluetoothPort(string BdAddr, string Name)
{
    /// <summary>
    /// Human-friendly label for ComboBox display: "Name (BdAddr)" when
    /// the port has a discoverable name, just BdAddr otherwise.
    /// Keeps both fields visible while letting a single TextBlock
    /// bind + truncate via TextTrimming.
    /// </summary>
    public string DisplayName =>
        string.IsNullOrWhiteSpace(Name) ? BdAddr : $"{Name} ({BdAddr})";
}
