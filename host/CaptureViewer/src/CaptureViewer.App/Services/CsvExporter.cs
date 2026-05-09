using System.Globalization;
using System.Text;

using CaptureViewer.Core;
using CaptureViewer.Core.Capture;

namespace CaptureViewer.App.Services;

/// <summary>
/// Writes a <see cref="CaptureFile"/> out as a CSV with one column
/// per declared field.  Numeric fields are scaled per
/// <c>scale_log10</c>; the header carries the unit string in
/// parentheses where present.
/// </summary>
public static class CsvExporter
{
    public static void Save(CaptureFile capture, string path)
    {
        ArgumentNullException.ThrowIfNull(capture);
        ArgumentException.ThrowIfNullOrWhiteSpace(path);

        var sb = new StringBuilder();
        var ci = CultureInfo.InvariantCulture;

        // Locate ts_us so we can synthesize a friendlier `t_seconds`
        // column up front.  Plot tools (gnuplot / Excel / pandas) are
        // happier with seconds than with microseconds-as-int.
        FieldDescriptor? tsField = null;
        foreach (var f in capture.Fields)
        {
            if (f.Name == "ts_us") { tsField = f; break; }
        }

        // Header row
        var headers = new List<string>(capture.Fields.Count + 1);
        if (tsField is not null) headers.Add("t_seconds");
        foreach (var f in capture.Fields)
        {
            headers.Add(string.IsNullOrEmpty(f.Unit)
                ? f.Name
                : $"{f.Name} ({f.Unit})");
        }
        sb.AppendLine(string.Join(",", headers));

        // Rows
        for (var i = 0; i < capture.RecordCount; i++)
        {
            var rec = capture.Records(i).Span;
            var first = true;
            if (tsField is not null)
            {
                var tsec = FieldReader.ReadRawDouble(rec, tsField) / 1_000_000.0;
                sb.Append(tsec.ToString("G", ci));
                first = false;
            }
            foreach (var f in capture.Fields)
            {
                if (!first) sb.Append(',');
                first = false;
                sb.Append(FieldReader.ReadScaledDouble(rec, f).ToString("G", ci));
            }
            sb.AppendLine();
        }

        File.WriteAllText(path, sb.ToString());
    }
}
