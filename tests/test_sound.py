"""Category C: Sound driver tests.

Exercises /dev/tone0 (tune string parser), /dev/pcm0 (raw PCM write ABI),
and the apps/sound NSH builtin that wraps both.  Most tests check that
the command returns within the expected duration window (the driver uses
synchronous nxsig_usleep playback), so timing confirms both that audio
actually started and that the hardware pipeline is not hung.
"""

import re
import time

import pytest


# ---------------------------------------------------------------------------
# C.0 Registration
# ---------------------------------------------------------------------------


def test_sound_devices_present(p):
    """C-1: /dev/tone0 and /dev/pcm0 are registered at boot."""
    output = p.sendCommand("ls /dev")
    assert "tone0" in output, f"/dev/tone0 missing: {output}"
    assert "pcm0" in output, f"/dev/pcm0 missing: {output}"


def test_sound_dmesg_banner(p):
    """C-2: bringup syslog contains the three sound init lines."""
    output = p.sendCommand("dmesg")
    assert "sound: initialized" in output, f"sound banner missing: {output}"
    assert "tone: /dev/tone0 registered" in output
    assert "pcm: /dev/pcm0 registered" in output


# ---------------------------------------------------------------------------
# C.1 /dev/tone0 tune string parser (echo path)
# ---------------------------------------------------------------------------


def test_tone_single_note(p):
    """C-3: single quarter note at 120 BPM takes about 500 ms."""
    t0 = time.monotonic()
    p.sendCommand('echo "C4/4" > /dev/tone0', timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    # 120 BPM quarter = 500 ms note; allow release gap + serial round-trip.
    assert 400 <= elapsed <= 900, f"unexpected duration: {elapsed:.0f} ms"


def test_tone_major_scale(p):
    """C-4: C major scale (7 quarters + 1 half) takes about 4.5 s."""
    t0 = time.monotonic()
    p.sendCommand(
        'echo "C4/4 D4/4 E4/4 F4/4 G4/4 A4/4 B4/4 C5/2" > /dev/tone0',
        timeout=15,
    )
    elapsed = (time.monotonic() - t0) * 1000
    # 7 * 500 + 1000 = 4500 ms nominal; allow slack for release gaps.
    assert 4200 <= elapsed <= 5500, f"unexpected duration: {elapsed:.0f} ms"


def test_tone_rest_and_accidental(p):
    """C-5: rest + accidental + dotted note parses without error."""
    t0 = time.monotonic()
    p.sendCommand('echo "C4/4 R/4 D#5/8." > /dev/tone0', timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    # 500 (C4) + 500 (rest) + 375 (dotted 8th) ~= 1375 ms
    assert 1200 <= elapsed <= 2000, f"unexpected duration: {elapsed:.0f} ms"


def test_tone_tempo_directive(p):
    """C-6: T240 halves the note duration vs the default 120 BPM."""
    t0 = time.monotonic()
    p.sendCommand('echo "T240 C4/4 D4/4" > /dev/tone0', timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    # 240 BPM quarter = 250 ms; two quarters ~= 500 ms + release gaps.
    assert 400 <= elapsed <= 1000, f"unexpected duration: {elapsed:.0f} ms"


# ---------------------------------------------------------------------------
# C.2 apps/sound NSH builtin
# ---------------------------------------------------------------------------


def test_sound_usage(p):
    """C-7: `sound` with no args prints usage."""
    output = p.sendCommand("sound")
    assert "Usage" in output
    assert "beep" in output
    assert "notes" in output


def test_sound_beep_default(p):
    """C-8: `sound beep` defaults to 500 Hz / 200 ms via /dev/pcm0."""
    t0 = time.monotonic()
    p.sendCommand("sound beep", timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    assert 150 <= elapsed <= 800, f"unexpected duration: {elapsed:.0f} ms"


def test_sound_beep_custom(p):
    """C-9: `sound beep 800 300` runs for about 300 ms."""
    t0 = time.monotonic()
    p.sendCommand("sound beep 800 300", timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    assert 250 <= elapsed <= 900, f"unexpected duration: {elapsed:.0f} ms"


def test_sound_selftest(p):
    """C-10: `sound selftest` returns successfully within ~500 ms."""
    t0 = time.monotonic()
    output = p.sendCommand("sound selftest", timeout=5)
    elapsed = (time.monotonic() - t0) * 1000
    assert "500 Hz 200 ms" in output
    assert 150 <= elapsed <= 800, f"unexpected duration: {elapsed:.0f} ms"


def test_sound_notes_via_app(p):
    """C-11: `sound notes` forwards the tune string to /dev/tone0."""
    t0 = time.monotonic()
    p.sendCommand('sound notes "C4/4 E4/4 G4/2"', timeout=10)
    elapsed = (time.monotonic() - t0) * 1000
    # 500 + 500 + 1000 = 2000 ms nominal
    assert 1800 <= elapsed <= 3000, f"unexpected duration: {elapsed:.0f} ms"


def test_sound_volume_roundtrip(p):
    """C-12: volume SET then GET round-trips through TONEIOC_VOLUME_*."""
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
    """C-13: `sound off` returns cleanly even when nothing is playing."""
    output = p.sendCommand("sound off")
    # apps/sound prints nothing on success
    assert "failed" not in output.lower()


# ---------------------------------------------------------------------------
# C.3 /dev/pcm0 error paths (raw ABI sanity)
# ---------------------------------------------------------------------------


def test_pcm_short_write_rejected(p):
    """C-14: writing fewer bytes than the header does not crash the board.

    The kernel driver must reject the write with -EINVAL; we cannot observe
    the errno directly from NSH redirection, but confirming that the next
    command still runs proves no panic / hang occurred.
    """
    p.sendCommand('echo "abcd" > /dev/pcm0', timeout=5)
    # Survivor check: a follow-up command must still reach the prompt.
    output = p.sendCommand("echo alive")
    assert "alive" in output


def test_pcm_bad_magic_rejected(p):
    """C-15: a 20-byte garbage header is rejected without panic."""
    # 20 printable bytes = short enough to fit in one echo argv, magic
    # value is wrong so the driver returns -EINVAL.
    p.sendCommand("echo 'XXXXXXXXXXXXXXXXXXXX' > /dev/pcm0", timeout=5)
    output = p.sendCommand("echo alive")
    assert "alive" in output


# ---------------------------------------------------------------------------
# C.4 Audible verification (interactive)
#
# The non-interactive tests above prove the driver schedules playback with
# the expected duration, but they cannot verify that audio actually reaches
# the speaker.  The amplifier enable pin, the DAC output path, and the
# speaker itself all fall outside the timing assertions.  The tests in this
# section play a distinctive pattern on each code path and ask the operator
# to confirm audibly.
# ---------------------------------------------------------------------------


@pytest.mark.interactive
def test_sound_audible_tone_dev(p):
    """C-16: audible verification for /dev/tone0 (kernel tune parser)."""
    p.sendCommand('echo "C4/4 E4/4 G4/4 C5/2" > /dev/tone0', timeout=10)
    p.waitUser("Confirm: did you hear an ascending C/E/G/C arpeggio from /dev/tone0?")


@pytest.mark.interactive
def test_sound_audible_pcm_dev(p):
    """C-17: audible verification for /dev/pcm0 via `sound beep`."""
    p.sendCommand("sound beep 440 400", timeout=5)
    p.sendCommand("sound beep 880 400", timeout=5)
    p.waitUser("Confirm: did you hear two distinct tones (440 Hz then 880 Hz) via /dev/pcm0?")


@pytest.mark.interactive
def test_sound_audible_notes_app(p):
    """C-18: audible verification for apps/sound `sound notes`."""
    p.sendCommand('sound notes "T240 C4/4 E4/4 G4/4 C5/4 G4/4 E4/4 C4/2"', timeout=10)
    p.waitUser("Confirm: did you hear an up-and-down C arpeggio from `sound notes`?")


@pytest.mark.interactive
def test_sound_audible_volume(p):
    """C-19: audible verification that volume changes take effect."""
    # Save, play loud, play soft, restore.
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
