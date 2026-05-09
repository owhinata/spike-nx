using CaptureViewer.Core.Capture;

namespace CaptureViewer.Core.Live;

/// <summary>
/// Persists received capture sessions to disk under a deterministic
/// filename pattern: <c>{schemaName}_{startTsUs}.cap</c>.  The
/// filename collision case (same schema + same start_ts_us — happens
/// only if the firmware reboot did not advance CLOCK_BOOTTIME) is
/// resolved by appending <c>_NN</c>.
/// </summary>
public sealed class CaptureFileWriter
{
    private readonly string _outputDirectory;

    public CaptureFileWriter(string outputDirectory)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(outputDirectory);
        _outputDirectory = outputDirectory;
        Directory.CreateDirectory(_outputDirectory);
    }

    /// <summary>
    /// Writes the capture's payload to disk and returns the absolute
    /// path that was written.  Note: <see cref="CaptureFile.Payload"/>
    /// is the *full* on-disk byte stream (header + field descriptors +
    /// records), so the file is byte-identical to what the firmware
    /// emitted.
    /// </summary>
    public string Save(CaptureFile capture)
    {
        ArgumentNullException.ThrowIfNull(capture);

        var basePath = Path.Combine(
            _outputDirectory,
            $"{capture.SchemaName}_{capture.StartTimestampUs}.cap");
        var path = basePath;
        for (var i = 1; File.Exists(path); i++)
        {
            path = Path.Combine(
                _outputDirectory,
                $"{capture.SchemaName}_{capture.StartTimestampUs}_{i:D2}.cap");
        }

        File.WriteAllBytes(path, capture.Payload.Span);
        return path;
    }
}
