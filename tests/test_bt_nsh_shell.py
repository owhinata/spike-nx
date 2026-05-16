"""Category H-9 (Issue #108): BT-side NSH shell over SPP.

End-to-end test for the ``MODE SHELL`` / ``MODE TELEMETRY`` exchange
documented in docs/{ja,en}/development/bt-nsh-shell.md.  Marked
@pytest.mark.interactive because it requires a paired Linux host with
CAP_NET_RAW (same setup as test_bt_spp.py::test_bt_pc_pair_and_stream).

The flow exercised here:

  1. ``btsensor start`` → ``btsensor bt on``
  2. PC opens BTPROTO_RFCOMM channel 1
  3. PC: ``MODE SHELL\\n`` → expect ``OK\\n``
  4. PC: ``ls /dev\\n`` → expect ``btnsh_in`` and ``btnsh_out`` lines
  5. PC: ``exit\\n`` → expect ``READY\\n``
  6. PC: ``IMU ON\\n`` → expect ``OK\\n`` and BUNDLE bytes flowing
"""

import re
import socket
import time

import pytest

from test_bt_spp import _btsensor_fresh_start


def _read_until(sock, predicate, timeout=5.0):
    """Read from sock until ``predicate(buf)`` returns True or timeout.

    Returns the accumulated bytes.  Used to scan for line markers and
    NSH output without imposing a strict line-by-line parser (the
    NSH child emits raw bytes — no CRLF normalisation).
    """
    deadline = time.time() + timeout
    buf = b""
    sock.settimeout(0.5)
    while time.time() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            chunk = b""
        if chunk:
            buf += chunk
            if predicate(buf):
                return buf
    return buf


@pytest.mark.interactive
def test_bt_nsh_shell_roundtrip(p):
    """H-9: MODE SHELL → ls /dev → exit → READY → telemetry resumes."""
    bdaddr = _btsensor_fresh_start(p)

    try:
        out = p.sendCommand("btsensor bt on", timeout=5)
        assert "OK" in out, f"btsensor bt on failed: {out!r}"

        p.waitUser(
            f"Hub is advertising as 'SPIKE-BT-Sensor' at\n"
            f"  BD_ADDR = {bdaddr}\n"
            f"with telemetry pumps OFF (default).\n\n"
            f"Pair + trust if not already done (see pc-receive-spp.md)\n"
            f"then press ENTER."
        )

        try:
            sock = socket.socket(socket.AF_BLUETOOTH,
                                 socket.SOCK_STREAM,
                                 socket.BTPROTO_RFCOMM)
        except OSError as exc:
            pytest.skip(f"AF_BLUETOOTH socket unavailable: {exc}")

        sock.settimeout(5.0)
        try:
            sock.connect((bdaddr, 1))
        except OSError as exc:
            sock.close()
            pytest.fail(f"RFCOMM connect failed: {exc}")

        try:
            # Step 1 — enter shell mode.
            sock.sendall(b"MODE SHELL\n")
            buf = _read_until(sock, lambda b: b"OK\n" in b, timeout=3.0)
            assert b"OK\n" in buf, (
                f"Expected OK\\n after MODE SHELL, got: {buf!r}"
            )

            # Brief pause so the post-drain callback can flip mode and
            # the NSH child can issue its first prompt before we send
            # stdin.  Per protocol, the peer is allowed to send
            # immediately after OK\n, but on slow boards waiting an
            # extra 100 ms makes the test deterministic.
            time.sleep(0.2)

            # Step 2 — ask NSH for /dev contents and check the FIFO
            # nodes appear.  Be liberal about delays — readline_fd is
            # a tight loop but the BTstack scheduler can interleave.
            sock.sendall(b"ls /dev\n")
            buf = _read_until(
                sock,
                lambda b: b"btnsh_in" in b and b"btnsh_out" in b,
                timeout=5.0,
            )
            assert b"btnsh_in" in buf, (
                f"Expected /dev/btnsh_in line in NSH output: {buf!r}"
            )
            assert b"btnsh_out" in buf, (
                f"Expected /dev/btnsh_out line in NSH output: {buf!r}"
            )

            # Step 3 — exit the shell.  Drain the post-exit READY\n
            # handshake.
            sock.sendall(b"exit\n")
            buf = _read_until(sock, lambda b: b"READY\n" in b, timeout=5.0)
            assert b"READY\n" in buf, (
                f"Expected READY\\n after exit, got: {buf!r}"
            )

            # Step 4 — telemetry mode resumes; IMU ON should yield OK
            # and BUNDLE frames (magic 0x6B 0xB6) starting flowing.
            sock.sendall(b"IMU ON\n")
            buf = _read_until(sock, lambda b: b"OK\n" in b, timeout=3.0)
            assert b"OK\n" in buf, f"IMU ON did not return OK: {buf!r}"

            buf = _read_until(
                sock, lambda b: b"\x6b\xb6" in b, timeout=3.0
            )
            assert b"\x6b\xb6" in buf, (
                f"No BUNDLE magic seen after IMU ON: {buf!r}"
            )
        finally:
            try:
                sock.close()
            except OSError:
                pass

    finally:
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


def test_bt_nsh_shell_fifos_present(p):
    """H-9-aux (automatable): /dev/btnsh_in and /dev/btnsh_out exist
    after `btsensor start`.

    rcS auto-launches btsensor at boot (Issue #111), so after the
    reboot the daemon is already running and the FIFOs should already
    be present — `btsensor start` then returns "already running",
    which is the expected post-boot state.  We accept either outcome
    and verify the post-condition that matters: the FIFO nodes exist.
    """
    p.reboot(timeout=15)

    out = p.sendCommand("btsensor start", timeout=5)
    assert ("started (pid" in out) or ("already running" in out), (
        f"btsensor start neither started nor reported already running: {out!r}"
    )

    # Wait briefly for shell_init() to run (it happens early in the
    # daemon entry, before HCI bring-up).
    time.sleep(1.0)

    out = p.sendCommand("ls /dev", timeout=5)
    assert "btnsh_in" in out, f"/dev/btnsh_in not present: {out!r}"
    assert "btnsh_out" in out, f"/dev/btnsh_out not present: {out!r}"
