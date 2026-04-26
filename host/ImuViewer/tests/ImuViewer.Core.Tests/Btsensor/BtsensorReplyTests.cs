using FluentAssertions;
using ImuViewer.Core.Btsensor;
using Xunit;

namespace ImuViewer.Core.Tests.Btsensor;

public class BtsensorReplyTests
{
    [Fact]
    public void Parse_OK()
    {
        BtsensorReply r = BtsensorReply.Parse("OK");
        r.Should().BeOfType<BtsensorReply.Ok>();
        r.IsOk.Should().BeTrue();
    }

    [Fact]
    public void Parse_OK_with_trailing_crlf()
    {
        BtsensorReply.Parse("OK\r\n").Should().BeOfType<BtsensorReply.Ok>();
        BtsensorReply.Parse("OK\n").Should().BeOfType<BtsensorReply.Ok>();
    }

    [Fact]
    public void Parse_ERR_with_reason()
    {
        BtsensorReply r = BtsensorReply.Parse("ERR busy");
        r.Should().BeOfType<BtsensorReply.Err>()
            .Which.Reason.Should().Be("busy");
    }

    [Fact]
    public void Parse_ERR_with_multi_word_reason()
    {
        BtsensorReply r = BtsensorReply.Parse("ERR invalid 9999");
        r.Should().BeOfType<BtsensorReply.Err>()
            .Which.Reason.Should().Be("invalid 9999");
    }

    [Fact]
    public void Parse_ERR_no_reason()
    {
        BtsensorReply.Parse("ERR").Should().BeOfType<BtsensorReply.Err>()
            .Which.Reason.Should().Be(string.Empty);
    }

    [Fact]
    public void Parse_unknown_line_falls_back_to_Unknown()
    {
        BtsensorReply r = BtsensorReply.Parse("garbage line");
        r.Should().BeOfType<BtsensorReply.Unknown>()
            .Which.Line.Should().Be("garbage line");
        r.IsOk.Should().BeFalse();
    }
}
