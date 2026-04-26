namespace ImuViewer.Core.Btsensor;

public abstract record BtsensorReply
{
    public sealed record Ok : BtsensorReply
    {
        public override string ToString() => "OK";
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
