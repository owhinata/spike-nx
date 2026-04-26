using System.Text;
using FluentAssertions;
using ImuViewer.Core.Btsensor;
using Xunit;

namespace ImuViewer.Core.Tests.Btsensor;

public class BtsensorCommandClientTests
{
    [Fact]
    public async Task SendAsync_writes_command_and_resolves_with_OK()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);
        session.Start();
        using BtsensorCommandClient client = new(session);

        Task<BtsensorReply> sendTask = client.SendAsync("IMU ON", CancellationToken.None);

        await WaitForWritten(pipe, "IMU ON\n");
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("OK\n"));

        BtsensorReply reply = await sendTask.WaitAsync(TimeSpan.FromSeconds(2));
        reply.Should().BeOfType<BtsensorReply.Ok>();

        await session.DisposeAsync();
    }

    [Fact]
    public async Task SendAsync_resolves_with_ERR_when_hub_returns_busy()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);
        session.Start();
        using BtsensorCommandClient client = new(session);

        Task<BtsensorReply> sendTask = client.SendAsync("SET BATCH 13", CancellationToken.None);
        await WaitForWritten(pipe, "SET BATCH 13\n");
        await pipe.InjectAsync(Encoding.ASCII.GetBytes("ERR busy\n"));

        BtsensorReply reply = await sendTask.WaitAsync(TimeSpan.FromSeconds(2));
        reply.Should().BeOfType<BtsensorReply.Err>().Which.Reason.Should().Be("busy");

        await session.DisposeAsync();
    }

    [Fact]
    public async Task Two_serial_commands_resolve_in_FIFO_order()
    {
        FakeDuplexStream pipe = new();
        BtsensorSession session = new(pipe);
        session.Start();
        using BtsensorCommandClient client = new(session);

        Task<BtsensorReply> first = client.SendAsync("IMU OFF", CancellationToken.None);
        await WaitForWritten(pipe, "IMU OFF\n");
        Task<BtsensorReply> second = client.SendAsync("SET BATCH 13", CancellationToken.None);
        await WaitForWritten(pipe, "IMU OFF\nSET BATCH 13\n");

        await pipe.InjectAsync(Encoding.ASCII.GetBytes("OK\nERR invalid 200\n"));

        BtsensorReply r1 = await first.WaitAsync(TimeSpan.FromSeconds(2));
        BtsensorReply r2 = await second.WaitAsync(TimeSpan.FromSeconds(2));
        r1.Should().BeOfType<BtsensorReply.Ok>();
        r2.Should().BeOfType<BtsensorReply.Err>().Which.Reason.Should().Be("invalid 200");

        await session.DisposeAsync();
    }

    private static async Task WaitForWritten(FakeDuplexStream pipe, string expected, int timeoutMs = 2000)
    {
        byte[] target = Encoding.ASCII.GetBytes(expected);
        int waited = 0;
        while (waited < timeoutMs)
        {
            byte[] written = pipe.GetWritten();
            if (written.Length >= target.Length && written.AsSpan(0, target.Length).SequenceEqual(target))
            {
                return;
            }
            await Task.Delay(10);
            waited += 10;
        }
        throw new TimeoutException($"did not see expected outgoing bytes: {expected}");
    }
}
