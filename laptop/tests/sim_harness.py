"""Test harness: runs the REAL Mega firmware logic (test/firmware_sim.cpp, host build) behind a pyserial-like object and a virtual clock."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import robot_link as rl          # noqa: E402
import robot_protocol as rp      # noqa: E402

SIM = os.environ.get("RS26_SIM", "/tmp/rs26_sim")
TEST_KEY_HEX = "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"      # synthetic TEST key, not a credential of the robot
TEST_KEY = bytes.fromhex(TEST_KEY_HEX)
OTHER_KEY = bytes.fromhex("ffeeddccbbaa99887766554433221100ffeeddccbbaa99887766554433221100")


def sim_available():
    return os.path.isfile(SIM) and os.access(SIM, os.X_OK)


class FirmwareSim:
    def __init__(self):
        self.p = subprocess.Popen([SIM], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.tx = bytearray()
        self.usb_out = bytearray()
        self.all_tx = bytearray()          # everything that ever left Serial1 (for secret-leak checks)
        self.all_usb = bytearray()
        self.run(300)

    def _cmd(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        state = None
        tag = None
        while True:
            r = self.p.stdout.readline()
            if not r:
                raise RuntimeError("simulator exited")
            r = r.strip()
            if r == "OK":
                return state if state is not None else tag
            if r.startswith("TX "):
                b = bytes.fromhex(r[3:])
                self.tx += b
                self.all_tx += b
            elif r.startswith("UO "):
                b = bytes.fromhex(r[3:])
                self.usb_out += b
                self.all_usb += b
            elif r.startswith("STATE "):
                state = dict(kv.split("=", 1) for kv in r[6:].split())
            elif r.startswith("TAGOUT "):
                tag = r[7:]
            elif r.startswith("ERR"):
                raise RuntimeError(r)

    def rx(self, data):
        self._cmd("RX " + (data if isinstance(data, bytes) else data.encode("latin-1")).hex())

    def usb(self, text):
        self._cmd("USB " + text.encode("latin-1").hex())

    def run(self, ms):
        self._cmd("RUN %d" % ms)

    def reboot(self, enforce=True):
        self._cmd("REBOOT" if enforce else "REBOOT open")

    def state(self):
        return self._cmd("STATE")

    def ball(self, deg, strength):
        self._cmd("BALL %s %s" % (deg, strength))

    def tag(self, label, a, b):
        return self._cmd("TAG %s %s %s" % (label, a.hex(), b.hex()))

    def provision(self, key_hex=TEST_KEY_HEX):
        self.usb(":authkey " + key_hex + "\n")
        self.run(100)

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=5)
        except Exception:
            self.p.kill()


class SimSerial:
    """pyserial-like view of the simulator's Serial1 (the HC-05 side)."""

    def __init__(self, sim):
        self.sim = sim
        self.closed = False
        self.written = bytearray()

    @property
    def in_waiting(self):
        return len(self.sim.tx)

    def read(self, n=1):
        b = bytes(self.sim.tx[:n])
        del self.sim.tx[:n]
        return b

    def write(self, data):
        if self.closed:
            raise OSError("port closed")
        self.written += data
        self.sim.rx(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Env:
    """Robot simulation + virtual clock + RobotLink wired together."""

    def __init__(self, key=TEST_KEY, provision=True, enforce=True, rng=None, auto_auth=True):
        self.sim = FirmwareSim()
        if not enforce:
            self.sim.reboot(False)
        if provision:
            self.sim.provision()
        self.clock = Clock()
        self.ser = SimSerial(self.sim)
        counter = [0]

        def det_rng(n):
            counter[0] += 1
            return bytes(((counter[0] * 37 + i * 11) & 0xFF) for i in range(n))
        self.link = rl.RobotLink(self.ser, key, clock=self.clock, rng=rng or det_rng, auto_auth=auto_auth)

    def step(self, ms=20):
        self.clock.t += ms / 1000.0
        self.sim.run(ms)
        self.link.pump()

    def advance(self, ms, step=20):
        for _ in range(int(ms / step)):
            self.step(step)

    def silence(self, ms, step=20):
        """Time passes with the laptop sending and reading nothing (a dead link)."""
        for _ in range(int(ms / step)):
            self.clock.t += step / 1000.0
            self.sim.run(step)
            self.sim.tx.clear()

    def login(self, max_ms=3000):
        for _ in range(int(max_ms / 20)):
            self.step(20)
            if self.link.authed and self.sim.state()["authed"] == "1":
                return True
        return False

    def close(self):
        self.sim.close()
