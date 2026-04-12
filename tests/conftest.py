import os
import re
import sys
import time
import uuid

import pexpect
import pexpect.fdpexpect
import pexpect.spawnbase
import pytest
import serial

# ---------------------------------------------------------------------------
# ANSI escape code removal (split-safe monkey-patch)
# ---------------------------------------------------------------------------
_ANSI_RE = re.compile(rb"(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]")
_ANSI_PARTIAL_RE = re.compile(rb"(\x9B|\x1B\[)[0-?]*[ -\/]*$")
_original_read_nonblocking = pexpect.spawnbase.SpawnBase.read_nonblocking


def _clean_read_nonblocking(self, size=1, timeout=None):
    """Strip ANSI escapes from serial data, buffering partial sequences.

    The upstream NuttX CI monkey-patch strips per-read, which corrupts
    data when a multi-byte escape (e.g. ``\\x1b[K``) is split across
    USB CDC packets.  This version accumulates a residual buffer on the
    spawn instance so partial trailing escapes are held until the next
    read completes them.
    """
    buf = getattr(self, "_ansi_buf", b"")
    buf += _original_read_nonblocking(self, size, timeout)

    # Strip all complete ANSI sequences.
    buf = _ANSI_RE.sub(b"", buf)

    # If the tail looks like the start of an incomplete sequence,
    # hold it back for the next call.
    m = _ANSI_PARTIAL_RE.search(buf)
    if m:
        self._ansi_buf = buf[m.start():]
        return buf[:m.start()]

    self._ansi_buf = b""
    return buf


pexpect.spawnbase.SpawnBase.read_nonblocking = _clean_read_nonblocking

# ---------------------------------------------------------------------------
# NSH prompt pattern
# ---------------------------------------------------------------------------
PROMPT = "nsh> "


# ---------------------------------------------------------------------------
# NuttxSerial — lightweight wrapper around pexpect + pyserial
# ---------------------------------------------------------------------------
class NuttxSerial:
    def __init__(self, device, log_path, show_serial=False):
        self.device = device
        self.log_path = log_path
        self.show_serial = show_serial
        self.ser = None
        self.proc = None
        self.log_file = None
        self._open()

    # -- connection management ----------------------------------------------

    def _open(self):
        os.makedirs(self.log_path, exist_ok=True)
        log_name = os.path.join(
            self.log_path,
            time.strftime("%Y%m%d_%H%M%S") + ".log",
        )
        self.log_file = open(log_name, "ab+")
        self.ser = serial.Serial(port=self.device, baudrate=115200)
        self.proc = pexpect.fdpexpect.fdspawn(
            self.ser, "wb", maxread=20000, logfile=self.log_file
        )
        if self.show_serial:
            # Tee bytes read from the serial port to stdout (live view).
            self.proc.logfile_read = sys.stdout.buffer

    def close(self):
        if self.proc:
            # fdspawn.close() closes the underlying fd, so mark serial
            # as closed first to avoid double-close OSError.
            if self.ser:
                self.ser.fd = None
                self.ser.is_open = False
                self.ser = None
            self.proc.close()
            self.proc = None
        elif self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def reboot(self, timeout=15):
        """Issue NSH ``reboot`` and reconnect after the board resets."""
        try:
            self.proc.sendline("reboot")
        except Exception:
            pass  # the board may already be mid-reset
        for attempt in range(2):
            try:
                self.reconnect(timeout=timeout)
                return
            except (TimeoutError, Exception):
                if attempt == 1:
                    raise

    def reconnect(self, timeout=15):
        """Close and reopen the serial port (after USB replug / reset)."""
        # fdspawn owns the underlying fd; mark serial closed first to
        # avoid double-close OSError.
        if self.proc:
            if self.ser:
                self.ser.fd = None
                self.ser.is_open = False
                self.ser = None
            try:
                self.proc.close()
            except Exception:
                pass
            self.proc = None
        elif self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        # Wait for the device to reappear
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                self.ser = serial.Serial(port=self.device, baudrate=115200)
                break
            except serial.SerialException:
                time.sleep(0.5)
        else:
            raise TimeoutError(
                f"Device {self.device} did not reappear within {timeout}s"
            )
        self.proc = pexpect.fdpexpect.fdspawn(
            self.ser, "wb", maxread=20000, logfile=self.log_file
        )
        if self.show_serial:
            self.proc.logfile_read = sys.stdout.buffer
        # Wait for NSH prompt after reset (use raw expect, not sendCommand)
        for attempt in range(3):
            try:
                self.proc.send("\r\n")
                time.sleep(0.1)
                self.proc.send("\r\n")
                self.proc.expect(PROMPT, timeout=5)
                break
            except Exception:
                if attempt == 2:
                    raise

    # -- command helpers ----------------------------------------------------

    def clean_buffer(self):
        """Drain any pending data from the serial buffer."""
        while True:
            try:
                self.proc.read_nonblocking(size=self.proc.maxread, timeout=0.1)
            except Exception:
                break
        self.proc.before = b""
        self.proc.after = b""

    def sendCommand(self, cmd, timeout=10):
        """Send a command and return its output.

        Synchronization strategy:
        1. A per-call PRE marker (``echo MKPRE<nonce>``) is sent to
           establish a clean baseline past any stale buffer bytes.
           The unique nonce prevents false matches on stale data.
        2. A 10 ms pause lets USB CDC flush trailing bytes.
        3. The real command is sent, and we match ``\\r\\nnsh> `` —
           a line-anchored prompt — to capture its output.
        """
        nonce = uuid.uuid4().hex[:16]
        pre = f"MKPRE{nonce}"
        prompt_re = re.escape(PROMPT)

        self.proc.sendline(f"echo {pre}")
        self.proc.expect(f"{pre}\r\n{prompt_re}", timeout=timeout)

        # Brief pause so USB CDC can flush any trailing bytes before
        # Phase 2 starts (see issue #34).
        time.sleep(0.01)

        self.proc.sendline(cmd)
        self.proc.expect(f"\r\n{prompt_re}", timeout=timeout)
        raw = self.proc.before.decode(errors="ignore")

        if "\r\n" in raw:
            return raw.split("\r\n", 1)[1]
        return ""

    def waitUser(self, msg):
        """Pause and wait for user confirmation (interactive tests)."""
        input(f"\n>>> {msg}\n    Press Enter to continue...")


# ---------------------------------------------------------------------------
# pytest CLI options
# ---------------------------------------------------------------------------
def pytest_addoption(parser):
    parser.addoption(
        "-D",
        action="store",
        default=os.environ.get("NUTTX_DEVICE", "/dev/tty.usbmodem01"),
        help="serial device (env: NUTTX_DEVICE, default: /dev/tty.usbmodem01)",
    )
    parser.addoption(
        "--show-serial",
        "-S",
        action="store_true",
        default=False,
        help="tee NSH serial I/O to stdout (still logged to tests/logs/)",
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def p(pytestconfig):
    """Session-scoped NuttX serial connection."""
    device = pytestconfig.getoption("-D")
    show_serial = pytestconfig.getoption("--show-serial")
    log_path = os.path.join(os.path.dirname(__file__), "logs")
    nuttx = NuttxSerial(device, log_path, show_serial=show_serial)
    # Ensure we have an NSH prompt (retry a few times)
    for attempt in range(3):
        try:
            nuttx.clean_buffer()
            nuttx.proc.send("\r\n")
            time.sleep(0.1)
            nuttx.proc.send("\r\n")
            nuttx.proc.expect(PROMPT, timeout=5)
            break
        except Exception:
            if attempt == 2:
                raise
    yield nuttx
    nuttx.close()


# ---------------------------------------------------------------------------
# Collection hook: move interactive tests to the end
# ---------------------------------------------------------------------------
def pytest_collection_modifyitems(items):
    interactive = []
    normal = []
    for item in items:
        if "interactive" in item.keywords:
            interactive.append(item)
        else:
            normal.append(item)
    items[:] = normal + interactive
