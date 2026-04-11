import os
import re
import time
import uuid

import pexpect
import pexpect.fdpexpect
import pexpect.spawnbase
import pytest
import serial

# ---------------------------------------------------------------------------
# ANSI escape code removal (monkey-patch pexpect, same as NuttX CI upstream)
# ---------------------------------------------------------------------------
_original_read_nonblocking = pexpect.spawnbase.SpawnBase.read_nonblocking


def _clean_read_nonblocking(self, size=1, timeout=None):
    return re.sub(
        r"(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]",
        "",
        _original_read_nonblocking(self, size, timeout).decode(errors="ignore"),
    ).encode()


pexpect.spawnbase.SpawnBase.read_nonblocking = _clean_read_nonblocking

# ---------------------------------------------------------------------------
# NSH prompt pattern
# ---------------------------------------------------------------------------
PROMPT = "nsh> "


# ---------------------------------------------------------------------------
# NuttxSerial — lightweight wrapper around pexpect + pyserial
# ---------------------------------------------------------------------------
class NuttxSerial:
    def __init__(self, device, log_path):
        self.device = device
        self.log_path = log_path
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
           establish a clean baseline past any stale buffer bytes. The
           nonce makes the match unique; we expect the marker's output
           line immediately followed by the next NSH prompt.
        2. The real command is sent, and we match ``\\r\\nnsh> `` —
           a line-anchored prompt — to capture its output. Anchoring on
           ``\\r\\n`` before the prompt avoids false matches on a bare
           ``nsh> `` substring that might appear inside command output.

        Only one ``sendline`` is issued per phase, so NSH never has to
        process queued input behind a slow/heavy command.
        """
        nonce = uuid.uuid4().hex[:16]
        pre = f"MKPRE{nonce}"
        prompt_re = re.escape(PROMPT)

        # Phase 1: baseline sync. Matching ``<pre>\r\nnsh> `` as one
        # pattern consumes through the prompt after the PRE output.
        self.proc.sendline(f"echo {pre}")
        self.proc.expect(f"{pre}\r\n{prompt_re}", timeout=timeout)

        # Phase 2: send the real command; match line-anchored prompt.
        self.proc.sendline(cmd)
        self.proc.expect(f"\r\n{prompt_re}", timeout=timeout)
        raw = self.proc.before.decode(errors="ignore")

        # ``raw`` is "<cmd_echo>\r\n[<output>]" — strip the cmd echo
        # line. For no-output commands ``raw`` is just ``<cmd_echo>``
        # with no trailing ``\r\n``, so we return the empty string.
        if "\r\n" in raw:
            return raw.split("\r\n", 1)[1]
        return ""

    def waitUser(self, msg):
        """Pause and wait for user confirmation (interactive tests)."""
        input(f"\n>>> {msg}\n    Press Enter to continue...")

    # -- memory helpers -----------------------------------------------------

    def getFree(self):
        """Run ``free`` and return parsed memory info as a dict.

        Returns dict with keys: ``total``, ``used``, ``free``, ``largest``.
        Values are in bytes.
        """
        output = self.sendCommand("free")
        result = {}
        for line in output.splitlines():
            if "Umem" in line:
                parts = line.split()
                # free output: Umem: total used free largest
                if len(parts) >= 5:
                    result["total"] = int(parts[1])
                    result["used"] = int(parts[2])
                    result["free"] = int(parts[3])
                    result["largest"] = int(parts[4])
        return result


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


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="session")
def p(pytestconfig):
    """Session-scoped NuttX serial connection."""
    device = pytestconfig.getoption("-D")
    log_path = os.path.join(os.path.dirname(__file__), "logs")
    nuttx = NuttxSerial(device, log_path)
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


@pytest.fixture(autouse=True)
def check_memory_leak(request, p):
    """Record memory before/after each test and warn on leaks."""
    if request.node.get_closest_marker("no_memcheck"):
        yield
        return
    before = p.getFree()
    yield
    after = p.getFree()
    if before and after:
        leak = before["free"] - after["free"]
        if leak > 1024:
            import warnings

            warnings.warn(
                f"Possible memory leak: {leak} bytes "
                f"(free: {before['free']} -> {after['free']})"
            )


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
