import os
import re
import time

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
        if self.proc:
            self.proc.close()
            self.proc = None
        if self.ser and self.ser.is_open:
            self.ser.close()
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
        # Wait for NSH prompt after reset
        self.sendCommand("", PROMPT, timeout=10)

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

    def sendCommand(self, cmd, expected=PROMPT, timeout=10):
        """Send a command and wait for the expected pattern.

        Returns the full output text between the command and the matched
        pattern (i.e. ``self.proc.before``).  Raises on timeout/EOF.
        """
        self.clean_buffer()
        self.proc.sendline(cmd)
        if cmd:
            # Wait for the command echo (first word only, as long
            # commands may be wrapped/garbled by the terminal)
            first_word = cmd.split()[0]
            self.proc.expect(re.escape(first_word), timeout=timeout)
        self.proc.expect(expected, timeout=timeout)
        return self.proc.before.decode(errors="ignore")

    def waitUser(self, msg):
        """Pause and wait for user confirmation (interactive tests)."""
        input(f"\n>>> {msg}\n    Press Enter to continue...")

    # -- memory helpers -----------------------------------------------------

    def getFree(self):
        """Run ``free`` and return parsed memory info as a dict.

        Returns dict with keys: ``total``, ``used``, ``free``, ``largest``.
        Values are in bytes.
        """
        output = self.sendCommand("free", PROMPT)
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
        default="/dev/tty.usbmodem01",
        help="serial device (default: /dev/tty.usbmodem01)",
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
def check_memory_leak(p):
    """Record memory before/after each test and warn on leaks."""
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
