"""Category G: W25Q256 + LittleFS + Zmodem tests.

G.1-G.4 are automated regression tests (run with ``-m "not slow"``).
G.I1-G.I2 are interactive Zmodem transfer tests that drive the lrzsz
``sz``/``rz`` binaries via subprocess; they require the lrzsz tools
on the PC (`brew install lrzsz` on macOS) and use the same physical
serial port that pytest is already connected to, so they temporarily
release/reopen the port around each transfer.
"""

import os
import re
import shutil
import subprocess
import time

import pytest


# ---------------------------------------------------------------------------
# G.1 LittleFS regression (automated)
# ---------------------------------------------------------------------------


def test_flash_mount(p):
    """G-1: /mnt/flash is mounted as LittleFS at boot."""
    out = p.sendCommand("mount")
    assert "/mnt/flash" in out
    assert "littlefs" in out


def test_flash_mtdblock_visible(p):
    """G-2: /dev/mtdblock0 is registered, full chip raw is not exposed."""
    out = p.sendCommand("ls /dev/mtdblock0")
    assert "mtdblock0" in out

    # /dev/mtd0 (full chip raw) must NOT be exposed — LEGO bootloader
    # area protection.  ls returns an error string for missing entries.
    out = p.sendCommand("ls /dev/mtd0")
    assert "No such file" in out or "ENOENT" in out or "mtd0" not in out


def test_flash_write_read(p):
    """G-3: echo -> file, read back via cat, contents match."""
    p.sendCommand("rm -f /mnt/flash/regress.txt")
    payload = "regression-{:x}".format(int(time.time()))
    p.sendCommand(
        f'echo "{payload}" > /mnt/flash/regress.txt', timeout=120
    )
    out = p.sendCommand("cat /mnt/flash/regress.txt")
    assert payload in out


def test_flash_persist_across_reboot(p):
    """G-4: file persists across NSH `reboot` (LittleFS durability)."""
    p.sendCommand("rm -f /mnt/flash/persist.txt")
    payload = "persist-{:x}".format(int(time.time()))
    p.sendCommand(
        f'echo "{payload}" > /mnt/flash/persist.txt', timeout=120
    )

    p.reboot(timeout=30)

    out = p.sendCommand("cat /mnt/flash/persist.txt")
    assert payload in out


# ---------------------------------------------------------------------------
# G.I Zmodem file transfer (interactive — operator drives picocom manually)
#
# Automating the transfer over the same serial port that pytest holds open
# proved unreliable: subprocess `sz`/`rz` either failed to negotiate
# (timeouts) or got into ZNAK retry loops, sometimes leaving the Hub's USB
# CDC stack in an unresponsive state requiring power cycle.  picocom-driven
# manual transfer is rock solid (see CLAUDE.md / docs).  These tests only
# automate file preparation and post-transfer verification (size + md5).
# ---------------------------------------------------------------------------

_TEST_SIZE = 1024 * 1024  # 1 MB
_TEST_FILE = "/tmp/spike_zmtest_1mb.bin"
_TEST_BASENAME = os.path.basename(_TEST_FILE)


def _ensure_lrzsz():
    if shutil.which("sz") is None or shutil.which("rz") is None:
        pytest.skip("lrzsz not installed (brew install lrzsz)")


def _create_testfile():
    """Always regenerate the test file with fresh random data; return md5."""
    subprocess.run(
        [
            "dd",
            "if=/dev/urandom",
            f"of={_TEST_FILE}",
            "bs=1024",
            f"count={_TEST_SIZE // 1024}",
        ],
        check=True,
        capture_output=True,
    )
    md5 = subprocess.run(
        ["md5sum", _TEST_FILE],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.split()[0]
    return md5


def _hub_md5(p, hub_path):
    """Run `md5 -f <path>` on the Hub and return the 32-char hex digest."""
    # NSH's md5 prints the 32-char hex digest with NO trailing newline.
    # Append a bare `; echo` so the sendCommand marker is on its own line.
    out = p.sendCommand(f"md5 -f {hub_path} ; echo", timeout=120).strip()
    m = re.search(r"\b([0-9a-f]{32})\b", out)
    assert m, f"could not parse md5 from output: {out!r}"
    return m.group(1)


def _release_port(p):
    """Close pytest's serial connection so picocom can use the port."""
    if p.proc is not None:
        try:
            p.proc.close()
        except Exception:
            pass
        p.proc = None
    if p.ser is not None:
        # Drop reference; do not call close() — it was already closed by
        # proc.close() (fdspawn owns the fd).  Setting these avoids
        # double-close errors when reconnect() runs.
        p.ser.fd = None
        p.ser.is_open = False
        p.ser = None


@pytest.mark.interactive
def test_flash_zmodem_send(p):
    """G-I1: PC -> Hub Zmodem send (1 MB).

    Operator runs picocom + sz manually; pytest verifies file size and
    on-device md5 against the freshly-generated source file.
    """
    _ensure_lrzsz()
    md5_orig = _create_testfile()

    p.sendCommand(f"rm -f /mnt/flash/{_TEST_BASENAME}")

    device = p.device
    _release_port(p)

    print()
    print("=" * 70)
    print("G-I1: PC -> Hub Zmodem SEND")
    print("=" * 70)
    print(f"Local file: {_TEST_FILE} (md5={md5_orig})")
    print()
    print("Run in another terminal:")
    print(
        f"  picocom --send-cmd 'sz -vv -L 256' {device}"
    )
    print()
    print("Then in the picocom session:")
    print(f"  nsh> rz")
    print(f"  (press C-a C-s, enter '{_TEST_FILE}', press Enter)")
    print(f"  (wait for 'Transfer complete', then C-a C-x to exit)")
    print("=" * 70)
    p.waitUser("Press Enter once the transfer has completed and picocom is closed.")

    p.reconnect(timeout=30)

    out = p.sendCommand(f"ls -l /mnt/flash/{_TEST_BASENAME}")
    m = re.search(r"\b(\d{4,})\s+/mnt/flash/", out)
    assert m, f"Could not parse size from: {out!r}"
    assert int(m.group(1)) == _TEST_SIZE, (
        f"Expected {_TEST_SIZE} bytes, got {m.group(1)}"
    )

    md5_hub = _hub_md5(p, f"/mnt/flash/{_TEST_BASENAME}")
    assert md5_orig == md5_hub, (
        f"on-device md5 mismatch: PC={md5_orig} Hub={md5_hub}"
    )

    print(f"--- G-I1 OK: {_TEST_SIZE} bytes, md5 matches ({md5_hub}) ---")


@pytest.mark.interactive
def test_flash_zmodem_recv(p):
    """G-I2: Hub -> PC Zmodem recv (1 MB) + md5 round-trip check.

    Operator runs picocom + rz manually in the current working directory.
    pytest verifies md5 against the original, then deletes the received
    file (and the Hub copy) on success.  Depends on test_flash_zmodem_send
    having uploaded the file.
    """
    _ensure_lrzsz()

    out = p.sendCommand(f"ls /mnt/flash/{_TEST_BASENAME}")
    if _TEST_BASENAME not in out:
        pytest.skip(
            f"upload {_TEST_BASENAME} first via test_flash_zmodem_send"
        )

    # Reuse whatever is currently on the Hub: ground truth is the on-device
    # md5, not whatever may be sitting in /tmp on this run.  This makes the
    # recv test work standalone after a previous send.
    md5_orig = _hub_md5(p, f"/mnt/flash/{_TEST_BASENAME}")

    cwd = os.getcwd()
    received = os.path.join(cwd, _TEST_BASENAME)
    # Pre-clean any stale local copy so we know rz wrote a fresh file.
    if os.path.exists(received):
        os.remove(received)

    device = p.device
    _release_port(p)

    print()
    print("=" * 70)
    print("G-I2: Hub -> PC Zmodem RECV")
    print("=" * 70)
    print(f"Receive into: {cwd}")
    print(f"Expected md5: {md5_orig}")
    print()
    print("Run in another terminal (the cd step matters — rz writes to "
          "picocom's cwd):")
    print(f"  cd {cwd}")
    print(
        f"  picocom --receive-cmd 'rz -vv -y' {device}"
    )
    print()
    print("Then in the picocom session:")
    print(f"  nsh> sz /mnt/flash/{_TEST_BASENAME}")
    print(f"  (press C-a C-r, then Enter with no arguments)")
    print(f"  (wait for 'Transfer complete', then C-a C-x to exit)")
    print("=" * 70)
    p.waitUser("Press Enter once the transfer has completed and picocom is closed.")

    p.reconnect(timeout=30)

    try:
        assert os.path.exists(received), f"file not received at {received}"
        md5_recv = subprocess.run(
            ["md5sum", received],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.split()[0]
        assert md5_orig == md5_recv, (
            f"round-trip md5 mismatch: orig={md5_orig} recv={md5_recv}"
        )

        # Verification passed — clean up so cwd / Hub aren't littered.
        os.remove(received)
        p.sendCommand(f"rm /mnt/flash/{_TEST_BASENAME}")

        print(f"--- G-I2 OK: md5 matches ({md5_recv}); cleaned up ---")
    except Exception:
        # Leave the received file in place on failure for debugging.
        print(f"FAILED — received file kept at {received} for inspection")
        raise
