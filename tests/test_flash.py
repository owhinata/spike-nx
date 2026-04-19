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


# ---------------------------------------------------------------------------
# G.S Concurrent-load stress (interactive)
#
# Verifies that heavy W25Q256 SPI2/DMA1 traffic does not disrupt other
# active peripherals: the IMU I2C2 daemon (833 Hz reads via HPWORK) and
# Sound DAC1 + DMA1_S5 playback running concurrently with the flash dd.
# Operator confirms audible cleanliness; dmesg check + IMU status check
# automate the rest.  Required by Codex review of the Issue #46 plan.
# ---------------------------------------------------------------------------


_FORBIDDEN_LOG_KEYWORDS = (
    "ERROR",
    "FAULT",
    "underrun",
    "overflow",
    "ASSERT",
    "panic",
)


@pytest.mark.interactive
def test_flash_stress_concurrent(p):
    """G-S1: Sound + IMU + Flash dd concurrent stress.

    Drives the IMU daemon (continuous I2C2 + EXTI4 + HPWORK), a 4-second
    Sound DMA1_S5 playback, and a 64 KB Flash write+read in parallel,
    then checks dmesg / IMU status / audible cleanliness for regressions.
    """
    # 1. Start IMU daemon (puts I2C2 + EXTI + HPWORK under continuous load)
    p.sendCommand("imu start", timeout=10)
    time.sleep(1)
    status = p.sendCommand("imu status")
    assert re.search(r"running:\s*yes", status), (
        f"IMU daemon failed to start: {status!r}"
    )

    try:
        # 2. Capture dmesg baseline (we will diff after the run)
        dmesg_before = p.sendCommand("dmesg", timeout=10)

        # 3. Drop the volume so the tone is audible-but-quiet, then kick
        # off a 4-second 440 Hz tone in the background.  NSH `&` runs the
        # command as a daemon so we can keep issuing other commands while
        # DAC1+DMA1_S5 are active.  The original volume is restored in
        # the finally block.
        original_volume = p.sendCommand("sound volume")
        m_vol = re.search(r"volume:\s*(\d+)", original_volume)
        baseline_vol = int(m_vol.group(1)) if m_vol else 100
        p.sendCommand("sound volume 15", timeout=5)
        p.sendCommand("sound beep 440 4000 &", timeout=5)
        time.sleep(0.3)  # let the playback actually start

        # 4. Heavy flash I/O while sound is playing.  64 KB on LittleFS
        # exercises 16 × 4 KB sector erases + 256 page programs (≈3-5 s),
        # which should fully overlap the 4 s tone.
        p.sendCommand("rm -f /mnt/flash/stress.bin", timeout=10)
        p.sendCommand(
            "dd if=/dev/zero of=/mnt/flash/stress.bin bs=1024 count=64",
            timeout=120,
        )
        p.sendCommand(
            "dd if=/mnt/flash/stress.bin of=/dev/null bs=1024 count=64",
            timeout=60,
        )
        # 5. Wait for the tone (and any tail-end PCM DMA work) to finish
        time.sleep(2)

        # 6. NSH dd is silent on success; verify the file is the right size
        ls = p.sendCommand("ls -l /mnt/flash/stress.bin")
        m = re.search(r"\b(\d{4,})\s+/mnt/flash/stress\.bin", ls)
        assert m, f"could not parse size from: {ls!r}"
        assert int(m.group(1)) == 65536, (
            f"expected 65536 bytes, got {m.group(1)}: {ls!r}"
        )

        # 7. IMU daemon must still be alive and reading
        status = p.sendCommand("imu status")
        assert re.search(r"running:\s*yes", status), (
            f"IMU daemon died during stress: {status!r}"
        )

        # 8. dmesg diff must not contain known regressions
        dmesg_after = p.sendCommand("dmesg", timeout=10)
        new_lines = dmesg_after[len(dmesg_before):]
        for kw in _FORBIDDEN_LOG_KEYWORDS:
            assert kw not in new_lines, (
                f"dmesg gained {kw!r} during stress:\n{new_lines}"
            )

        # 9. Operator confirms audible cleanliness (the only check that
        # cannot be automated — DAC underruns may not surface as syslog).
        p.waitUser(
            "Confirm: was the 4-second 440 Hz tone clean — "
            "no clicks, pops, dropouts, or pitch wobble?"
        )
    finally:
        p.sendCommand("imu stop", timeout=5)
        p.sendCommand("rm -f /mnt/flash/stress.bin", timeout=10)
        # restore baseline volume so subsequent runs are not silently quiet
        try:
            p.sendCommand(f"sound volume {baseline_vol}", timeout=5)
        except Exception:
            pass

    print("--- G-S1 OK: concurrent load survived without regressions ---")
