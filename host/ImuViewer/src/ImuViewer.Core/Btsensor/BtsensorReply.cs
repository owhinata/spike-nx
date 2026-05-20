namespace ImuViewer.Core.Btsensor;

public abstract record BtsensorReply
{
    public sealed record Ok(string? Payload = null) : BtsensorReply
    {
        public override string ToString() =>
            Payload is null ? "OK" : $"OK {Payload}";
    }

    public sealed record Err(string Reason) : BtsensorReply
    {
        public override string ToString() => $"ERR {Reason}";
    }

    public sealed record Unknown(string Line) : BtsensorReply
    {
        public override string ToString() => $"<unrecognised: {Line}>";
    }

    public static BtsensorReply Parse(string line)
    {
        ArgumentNullException.ThrowIfNull(line);
        string trimmed = line.TrimEnd('\r', '\n');
        if (trimmed == "OK")
        {
            return new Ok();
        }
        // Issue #139: "OK <payload>" form added for GET ODR/ACCEL_FSR/GYRO_FSR.
        // The payload is left as a string so callers can int.Parse with
        // their own range / format checks.
        if (trimmed.StartsWith("OK ", StringComparison.Ordinal))
        {
            return new Ok(trimmed[3..].Trim());
        }
        if (trimmed.StartsWith("ERR ", StringComparison.Ordinal))
        {
            return new Err(trimmed[4..].Trim());
        }
        if (trimmed == "ERR")
        {
            return new Err(string.Empty);
        }
        return new Unknown(trimmed);
    }

    public bool IsOk => this is Ok;
}
