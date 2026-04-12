"""Category F: Sound driver smoke tests.

Minimal smoke coverage for /dev/tone0, /dev/pcm0, and the apps/sound NSH
builtin. The goal is to prove the driver is alive, the NSH command plumbing
works, and that one short note / one short beep return within the expected
duration window — nothing more. Audible verification (scales, arpeggios,
volume sweeps) lives in the ``interactive`` tests at the bottom of this file.
"""

import re
import time

import pytest


# ---------------------------------------------------------------------------
# F.0 Registration and usage (silent)
# ---------------------------------------------------------------------------


def test_sound_devices_present(p):
    """F-1: /dev/tone0 and /dev/pcm0 are registered at boot."""
    output = p.sendCommand("ls /dev")
    assert "tone0" in output, f"/dev/tone0 missing: {output}"
    assert "pcm0" in output, f"/dev/pcm0 missing: {output}"


def test_sound_dmesg_banner(p):
    """F-2: bringup syslog contains the three sound init lines."""
    output = p.sendCommand("dmesg")
    assert "sound: initialized" in output, f"sound banner missing: {output}"
    assert "tone: /dev/tone0 registered" in output
    assert "pcm: /dev/pcm0 registered" in output


def test_sound_usage(p):
    """F-3: `sound` with no args prints usage."""
    output = p.sendCommand("sound")
    assert "Usage" in output
    assert "beep" in output
    assert "notes" in output


# ---------------------------------------------------------------------------
# F.1 Minimal timing smoke (one short tone + one short beep)
# ---------------------------------------------------------------------------


def test_tone_single_note(p):
    """F-4: /dev/tone0 single quarter note at 120 BPM returns in ~500 ms."""
    t0 = time.monotonic()
    p.sendCommand('echo "C4/4" > /dev/tone0', timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    # 120 BPM quarter = 500 ms nominal; allow release gap + sendCommand
    # sentinel round-trip overhead.
    assert 400 <= elapsed <= 1300, f"unexpected duration: {elapsed:.0f} ms"


def test_sound_beep_default(p):
    """F-5: `sound beep` defaults to 500 Hz / 200 ms via /dev/pcm0."""
    t0 = time.monotonic()
    p.sendCommand("sound beep", timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    assert 150 <= elapsed <= 800, f"unexpected duration: {elapsed:.0f} ms"


# ---------------------------------------------------------------------------
# F.2 Silent IOCTL / error-path smoke
# ---------------------------------------------------------------------------


def test_sound_volume_roundtrip(p):
    """F-6: volume SET then GET round-trips through TONEIOC_VOLUME_*."""
    original = p.sendCommand("sound volume")
    match = re.search(r"volume:\s*(\d+)", original)
    assert match, f"volume output unparseable: {original}"
    baseline = int(match.group(1))

    try:
        p.sendCommand("sound volume 30")
        probe = p.sendCommand("sound volume")
        assert "volume: 30" in probe, f"volume not updated: {probe}"

        p.sendCommand("sound volume 75")
        probe = p.sendCommand("sound volume")
        assert "volume: 75" in probe, f"volume not updated: {probe}"
    finally:
        p.sendCommand(f"sound volume {baseline}")


def test_sound_off(p):
    """F-7: `sound off` returns cleanly even when nothing is playing."""
    output = p.sendCommand("sound off")
    assert "failed" not in output.lower()


def test_pcm_short_write_rejected(p):
    """F-8: writing fewer bytes than the header does not crash the board."""
    p.sendCommand('echo "abcd" > /dev/pcm0', timeout=5)
    output = p.sendCommand("echo alive")
    assert "alive" in output


def test_pcm_bad_magic_rejected(p):
    """F-9: a 20-byte garbage header is rejected without panic."""
    p.sendCommand("echo 'XXXXXXXXXXXXXXXXXXXX' > /dev/pcm0", timeout=5)
    output = p.sendCommand("echo alive")
    assert "alive" in output


# ---------------------------------------------------------------------------
# F.3 Audible verification (interactive)
#
# Plays distinctive patterns on each code path; operator confirms audibly.
# ---------------------------------------------------------------------------


@pytest.mark.interactive
def test_sound_audible_tone_dev(p):
    """F-I1: audible verification for /dev/tone0 (kernel tune parser)."""
    p.sendCommand('echo "C4/4 E4/4 G4/4 C5/2" > /dev/tone0', timeout=10)
    p.waitUser("Confirm: did you hear an ascending C/E/G/C arpeggio from /dev/tone0?")


@pytest.mark.interactive
def test_sound_audible_pcm_dev(p):
    """F-I2: audible verification for /dev/pcm0 via `sound beep`."""
    p.sendCommand("sound beep 440 400", timeout=5)
    p.sendCommand("sound beep 880 400", timeout=5)
    p.waitUser("Confirm: did you hear two distinct tones (440 Hz then 880 Hz) via /dev/pcm0?")


@pytest.mark.interactive
def test_sound_audible_notes_app(p):
    """F-I3: audible verification for apps/sound `sound notes`."""
    p.sendCommand('sound notes "T240 C4/4 E4/4 G4/4 C5/4 G4/4 E4/4 C4/2"', timeout=10)
    p.waitUser("Confirm: did you hear an up-and-down C arpeggio from `sound notes`?")


@pytest.mark.interactive
def test_sound_audible_volume(p):
    """F-I4: audible verification that volume changes take effect."""
    original = p.sendCommand("sound volume")
    match = re.search(r"volume:\s*(\d+)", original)
    baseline = int(match.group(1)) if match else 100
    try:
        p.sendCommand("sound volume 100")
        p.sendCommand("sound beep 500 400", timeout=5)
        p.sendCommand("sound volume 20")
        p.sendCommand("sound beep 500 400", timeout=5)
        p.waitUser("Confirm: did the second 500 Hz tone sound noticeably quieter than the first?")
    finally:
        p.sendCommand(f"sound volume {baseline}")
