using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace ImuViewer.Core.Transport.Linux;

/// <summary>
/// Minimal P/Invoke wrapper around <c>socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM)</c>.
/// </summary>
/// <remarks>
/// .NET's <c>System.Net.Sockets.Socket</c> does not expose AF_BLUETOOTH on Linux,
/// so the BTPROTO_RFCOMM RFCOMM channel has to be opened with libc directly. The
/// Python reference (<c>tests/test_bt_spp.py::test_bt_pc_pair_and_stream</c>) uses
/// the same path via <c>socket.AF_BLUETOOTH</c>.
/// </remarks>
internal sealed class LinuxRfcommSocket : IDisposable
{
    private const int AF_BLUETOOTH = 31;
    private const int SOCK_STREAM = 1;
    private const int BTPROTO_RFCOMM = 3;

    private const int SHUT_RDWR = 2;

    private const int EINTR = 4;

    private readonly SafeFileHandle _handle;

    private LinuxRfcommSocket(SafeFileHandle handle)
    {
        _handle = handle;
    }

    public SafeFileHandle Handle => _handle;

    public static LinuxRfcommSocket Connect(string bdAddr, int channel)
    {
        if (channel is < 1 or > 30)
        {
            throw new ArgumentOutOfRangeException(nameof(channel), "RFCOMM channel must be 1..30");
        }

        int fd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (fd < 0)
        {
            throw new IOException(
                $"socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM) failed: errno={Marshal.GetLastPInvokeError()}");
        }

        SafeFileHandle safe = new(new IntPtr(fd), ownsHandle: true);

        // sockaddr_rc { sa_family_t rc_family; bdaddr_t rc_bdaddr (6 LE); uint8_t rc_channel; }
        // Linux sizeof = 10 (padded to 2-byte alignment of sa_family_t).
        Span<byte> addr = stackalloc byte[10];
        addr[0] = (byte)(AF_BLUETOOTH & 0xFF);
        addr[1] = (byte)((AF_BLUETOOTH >> 8) & 0xFF);
        byte[] bd = LinuxBdAddr.Encode(bdAddr);
        bd.CopyTo(addr.Slice(2, 6));
        addr[8] = (byte)channel;
        addr[9] = 0;

        int rc;
        unsafe
        {
            fixed (byte* p = addr)
            {
                do
                {
                    rc = connect(fd, (IntPtr)p, (uint)addr.Length);
                } while (rc < 0 && Marshal.GetLastPInvokeError() == EINTR);
            }
        }
        if (rc < 0)
        {
            int err = Marshal.GetLastPInvokeError();
            safe.Dispose();
            throw new IOException(
                $"connect(RFCOMM, {bdAddr}, ch={channel}) failed: errno={err}");
        }

        return new LinuxRfcommSocket(safe);
    }

    public unsafe int Read(Span<byte> buffer)
    {
        int fd = (int)_handle.DangerousGetHandle();
        long n;
        do
        {
            fixed (byte* p = buffer)
            {
                n = recv(fd, (IntPtr)p, (UIntPtr)buffer.Length, 0);
            }
        } while (n < 0 && Marshal.GetLastPInvokeError() == EINTR);
        if (n < 0)
        {
            throw new IOException($"recv failed: errno={Marshal.GetLastPInvokeError()}");
        }
        return (int)n;
    }

    public unsafe int Write(ReadOnlySpan<byte> buffer)
    {
        int fd = (int)_handle.DangerousGetHandle();
        int total = 0;
        while (total < buffer.Length)
        {
            long n;
            ReadOnlySpan<byte> slice = buffer[total..];
            fixed (byte* p = slice)
            {
                n = send(fd, (IntPtr)p, (UIntPtr)slice.Length, 0);
            }
            if (n < 0)
            {
                int err = Marshal.GetLastPInvokeError();
                if (err == EINTR)
                {
                    continue;
                }
                throw new IOException($"send failed: errno={err}");
            }
            total += (int)n;
        }
        return total;
    }

    public void Dispose()
    {
        // shutdown() wakes any thread parked in recv()/send() on this fd —
        // close(fd) alone is not sufficient on Linux RFCOMM sockets and
        // leaves the reader thread spinning inside the kernel until the
        // peer closes the link itself. SHUT_RDWR forces both directions
        // to abort, recv() returns 0 (orderly shutdown), and the
        // BtsensorSession reader breaks out via its read <= 0 guard.
        // We swallow shutdown errors because the fd may already be in a
        // half-disposed state (e.g. concurrent Dispose call); close()
        // always runs via SafeFileHandle.Dispose() to release the fd.
        if (!_handle.IsInvalid && !_handle.IsClosed)
        {
            int fd = (int)_handle.DangerousGetHandle();
            shutdown(fd, SHUT_RDWR);
        }
        _handle.Dispose();
    }

    [DllImport("libc", SetLastError = true)]
    private static extern int socket(int domain, int type, int protocol);

    [DllImport("libc", SetLastError = true)]
    private static extern int connect(int fd, IntPtr addr, uint addrlen);

    [DllImport("libc", SetLastError = true)]
    private static extern int shutdown(int fd, int how);

    [DllImport("libc", SetLastError = true, EntryPoint = "recv")]
    private static extern long recv(int fd, IntPtr buf, UIntPtr len, int flags);

    [DllImport("libc", SetLastError = true, EntryPoint = "send")]
    private static extern long send(int fd, IntPtr buf, UIntPtr len, int flags);
}
