using System.Diagnostics;
using System.Text.RegularExpressions;

namespace ImuViewer.Core.Transport.Linux;

public sealed partial class LinuxPortEnumerator : IBluetoothPortEnumerator
{
    [GeneratedRegex(@"^Device\s+([0-9A-Fa-f:]{17})\s+(.*)$", RegexOptions.Multiline)]
    private static partial Regex DeviceRegex();

    public async Task<IReadOnlyList<BluetoothPort>> GetPairedPortsAsync(CancellationToken ct)
    {
        ProcessStartInfo psi = new("bluetoothctl")
        {
            ArgumentList = { "devices", "Paired" },
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true,
        };

        using Process? p = Process.Start(psi)
            ?? throw new InvalidOperationException("failed to start bluetoothctl");
        string stdout = await p.StandardOutput.ReadToEndAsync(ct).ConfigureAwait(false);
        await p.WaitForExitAsync(ct).ConfigureAwait(false);
        if (p.ExitCode != 0)
        {
            // Some bluetoothctl versions print to stderr and still exit 0; fall through.
            return [];
        }

        List<BluetoothPort> ports = new();
        foreach (Match m in DeviceRegex().Matches(stdout))
        {
            ports.Add(new BluetoothPort(m.Groups[1].Value, m.Groups[2].Value.Trim()));
        }
        return ports;
    }
}
