"""robot_protocol.py - the laptop side of the robot's Bluetooth protocol, with NO input/output (pure logic, fully testable).

Wire format: text lines ending in "\\n" over the HC-05's serial link (Windows "Outgoing" Bluetooth COM port). See docs/BLUETOOTH.md.

Authentication (mutual challenge-response, HMAC-SHA-256 with a shared 32-byte key; the key never crosses the air):
    laptop -> Mega   AUTH
    Mega -> laptop   AUTH CH <sn: 32 hex>         16-byte challenge, unique, single use, valid 10 s
    laptop -> Mega   AUTH RESP <cn: 32 hex> <tag: 32 hex>      cn = 16 fresh random bytes from os.urandom,  tag = HMAC(key, "E2M" | sn | cn)[:16]
    Mega -> laptop   AUTH OK <tag2: 32 hex>       tag2 = HMAC(key, "M2E" | cn | sn)[:16]  - the laptop checks it, so a fake robot is detected
    laptop -> Mega   AUTH OUT                     end the session
HMAC-SHA-256 here is Python's standard library (hmac, hashlib): an implementation independent of the firmware's.
"""
import hashlib
import hmac
import json
import os
import re
from collections import namedtuple

NONCE_LEN = 16
TAG_LEN = 16
KEY_LEN = 32
PING_INTERVAL_S = 0.5            # the robot ends a session after 2.5 s without a valid command
MAX_LINE = 600                   # longer lines from the robot are discarded
FRAME_KINDS = "crid"


class KeyFormatError(ValueError):
    pass


def parse_key(text):
    """64 hexadecimal digits -> 32 bytes. Rejects anything else (and a key made of one repeated byte)."""
    t = (text or "").strip()
    if not re.fullmatch(r"[0-9a-fA-F]{64}", t):
        raise KeyFormatError("the key must be exactly 64 hexadecimal digits")
    key = bytes.fromhex(t)
    if len(set(key)) == 1:
        raise KeyFormatError("the key must not be one repeated byte")
    return key


def proof(key, label, first, second):
    """HMAC-SHA-256(key, label | first | second) truncated to 16 bytes. label is b'E2M' (laptop proof) or b'M2E' (robot proof)."""
    return hmac.new(key, label + first + second, hashlib.sha256).digest()[:TAG_LEN]


def fingerprint(key):
    """First 4 bytes of HMAC(key, "FPR"): lets you compare keys on the laptop and the Mega without showing them."""
    return hmac.new(key, b"FPR", hashlib.sha256).digest()[:4].hex()


# ------------------------------------------------------------------------------------------------ handshake
IDLE, WAIT_CH, WAIT_OK, AUTHED = "idle", "waiting for challenge", "waiting for robot proof", "authenticated"


class Handshake:
    """Client side of the handshake. Feed it every line from the robot; it tells you what to send and what happened.

    start(now)          -> line to send ("AUTH")
    on_line(line, now)  -> (consumed, line_to_send_or_None, event_or_None)
    tick(now)           -> line_to_send_or_None   (timeouts)
    """

    def __init__(self, key, rng=os.urandom):
        self.key = key
        self.rng = rng
        self.state = IDLE
        self.sn = self.cn = b""
        self.since = 0.0
        self.retry_at = 0.0
        self.failures = 0
        self.robot_open = False
        self.last_error = ""

    @property
    def authed(self):
        return self.state == AUTHED

    def reset(self, now=0.0, retry_in=0.0):
        self.state = IDLE
        self.sn = self.cn = b""
        self.retry_at = now + retry_in

    def can_start(self, now):
        return self.state == IDLE and now >= self.retry_at

    def start(self, now):
        self.state = WAIT_CH
        self.since = now
        return "AUTH"

    def _fail(self, now, why, retry_in):
        self.failures += 1
        self.last_error = why
        self.reset(now, retry_in)
        return why

    def on_line(self, line, now):
        if line.startswith("AUTH CH "):
            if self.state != WAIT_CH:
                return True, None, None                                   # a stale or unsolicited challenge: ignored
            hexsn = line[8:].strip()
            if not re.fullmatch(r"[0-9a-f]{32}", hexsn):
                return True, None, self._fail(now, "malformed challenge from the robot", 3.0)
            self.sn = bytes.fromhex(hexsn)
            self.cn = self.rng(NONCE_LEN)
            tag = proof(self.key, b"E2M", self.sn, self.cn)
            self.state = WAIT_OK
            self.since = now
            return True, "AUTH RESP %s %s" % (self.cn.hex(), tag.hex()), None
        if line.startswith("AUTH OK "):
            if self.state != WAIT_OK:
                return True, None, None
            hextag = line[8:].strip()
            want = proof(self.key, b"M2E", self.cn, self.sn)
            if re.fullmatch(r"[0-9a-f]{32}", hextag) and hmac.compare_digest(bytes.fromhex(hextag), want):
                self.state = AUTHED
                self.failures = 0
                self.robot_open = False
                return True, None, "authenticated"
            return True, None, self._fail(now, "the robot did NOT prove it knows the key (wrong key or not your robot)", 10.0)
        if line == "AUTH OPEN":
            self.robot_open = True
            self.reset(now, 5.0)
            return True, None, "the robot is NOT enforcing authentication (':auth off' on its USB console); it is not trusted"
        if line == "AUTH OUT":
            self.reset(now)
            return True, None, None
        if line.startswith("AUTH"):
            return True, None, None
        if line == "ERR:AUTH_LOCKOUT":
            return True, None, self._fail(now, "the robot is throttling authentication attempts; waiting", 8.0)
        if line == "ERR:AUTH_BUSY":
            self.reset(now, 0.5)
            return True, None, None
        if line == "ERR:NO_KEY":
            return True, None, self._fail(now, "the robot has no key stored (provision it over USB)", 15.0)
        if line in ("ERR:AUTH_FAIL", "ERR:AUTH_EXPIRED", "ERR:AUTH_NOCHAL", "ERR:AUTH_SYNTAX"):
            return True, None, self._fail(now, "authentication refused by the robot (%s)" % line[4:], 3.0)
        if line == "ERR:AUTH" or line == "PONG:AUTH":
            if self.state == AUTHED:
                self.reset(now, 0.2)
                return True, None, "session lost"                          # the robot no longer has our session
            return True, None, None                                        # stale refusal during a handshake
        return False, None, None

    def tick(self, now):
        if self.state in (WAIT_CH, WAIT_OK) and now - self.since > 3.0:
            self._fail(now, "no answer from the robot during authentication", 2.0)
        return None


# ------------------------------------------------------------------------------------------------ commands the laptop may send
SINGLE_KEYS = set("MUQWSADZCRP?")
WORDS = {"defend", "attack", "manual", "stop", "idle", "calibrate"}


def normalize_command(text):
    """-> (line_to_send, None) or (None, error message). Only commands the Mega firmware supports are allowed through."""
    t = " ".join((text or "").split())
    if not t:
        return None, "empty command"
    if len(t) > 24:
        return None, "command too long (the robot accepts at most 24 characters)"
    low = t.lower()
    if low in ("x", "!"):
        return "X", None
    if len(t) == 1 and t.upper() in SINGLE_KEYS:
        return t.upper(), None
    if low in WORDS:
        return low, None
    if re.fullmatch(r"gyro (cal|stop|drift( [0-9]{1,3})?)", low):
        return low, None
    if re.fullmatch(r"tele (on|off|now [crid])", low):
        return low, None
    return None, "unsupported command '%s' (type 'help')" % t


# ------------------------------------------------------------------------------------------------ telemetry
Frame = namedtuple("Frame", "kind seq ms data")


def parse_frame(line):
    """One telemetry line -> Frame, or None if it is not a well-formed version-1 frame."""
    if not line or len(line) > 400 or line[0] != "{" or line[-1] != "}":
        return None
    try:
        d = json.loads(line)
    except ValueError:
        return None
    if not isinstance(d, dict) or d.get("v") != 1 or d.get("k") not in list(FRAME_KINDS):
        return None
    n, ms = d.get("n"), d.get("ms")
    if not isinstance(n, int) or isinstance(n, bool) or not 0 <= n <= 65535 or not isinstance(ms, int) or isinstance(ms, bool) or ms < 0:
        return None
    return Frame(d["k"], n, ms, d)


class FrameStore:
    """Latest frame of each kind with its arrival time, plus counters for bad and lost frames."""

    def __init__(self):
        self.latest = {}
        self.arrived = {}
        self.bad = 0
        self.lost = 0
        self.count = 0
        self._last_seq = None

    def add(self, line, now):
        f = parse_frame(line)
        if f is None:
            self.bad += 1
            return None
        if self._last_seq is not None:
            gap = (f.seq - self._last_seq - 1) & 0xFFFF
            if 0 < gap < 1000:
                self.lost += gap
        self._last_seq = f.seq
        self.latest[f.kind] = f
        self.arrived[f.kind] = now
        self.count += 1
        return f

    def get(self, kind):
        f = self.latest.get(kind)
        return f.data if f else {}

    def age(self, kind, now):
        t = self.arrived.get(kind)
        return None if t is None else now - t


VIEWS = ("all", "ir", "imu", "motors", "ball", "safety", "raw")
VIEW_ALIASES = {"back": "all", "sensors": "ir", "gyro": "imu", "motor": "motors", "status": "safety"}
EXTRA_KIND = {"all": "r", "ir": "r", "imu": "i", "motors": "d"}


def _f(v, fmt="%.1f"):
    return "n/a" if v is None else fmt % v


def _arr(d, key, fmt="%4.0f", n=6):
    a = d.get(key)
    if not isinstance(a, list):
        return " ".join(["n/a"] * n)
    return " ".join(fmt % x if isinstance(x, (int, float)) else " n/a" for x in a[:n])


def _stale(store, kind, now):
    a = store.age(kind, now)
    return "  [no data yet]" if a is None else ("  [STALE %.0f s old]" % a if a > 2.5 else "")


def render_view(store, view, now, authed):
    """Text for one view. Every value the robot did not provide is shown as n/a; old sections are flagged."""
    c, r, i, d = store.get("c"), store.get("r"), store.get("i"), store.get("d")
    head = "== %s | session: %s | frames %d, bad %d, lost %d ==" % (view.upper(), "authenticated" if authed else "NOT AUTHENTICATED", store.count, store.bad, store.lost)
    out = [head]
    if view in ("all", "safety"):
        out.append("mode %s / %s | safety %s | fault '%s' | e-stop %s | robot hears remote: %s | last key '%s'%s" % (
            c.get("mode", "n/a"), c.get("sub", "n/a"), c.get("safety", "n/a"), c.get("fault", ""), "LATCHED" if c.get("estop") else "clear", "yes" if c.get("bt") else "no", c.get("cmd", "-"), _stale(store, "c", now)))
        if view == "safety":
            out.append("fault mask %s | loop avg %s us, max %s us | robot tx drops %s%s" % (d.get("faults", "n/a"), d.get("loop_avg_us", "n/a"), d.get("loop_max_us", "n/a"), d.get("tx_drops", "n/a"), _stale(store, "d", now)))
    if view in ("all", "ball"):
        out.append("ball %s | bearing %s deg (0 = ahead, + = right) | confidence %s%s%s%s" % ("SEEN" if c.get("ball") else "not seen", _f(c.get("bearing_deg")), c.get("conf", "n/a"),
                                                                                           " | AMBIGUOUS" if c.get("ambiguous") else "", " | centered" if c.get("centered") else "", _stale(store, "c", now)))
    if view in ("all", "ir"):
        out.append("receivers      A0   A1   A2   A3   A4   A5%s" % _stale(store, "r", now))
        for key, label in (("raw_level", "level"), ("swing", "swing"), ("filtered", "filtered"), ("noise_floor", "noise floor"), ("threshold", "threshold")):
            if view == "ir" or key == "swing":
                out.append("%-12s %s" % (label, _arr(r, key)))
        out.append("obstacle modules: D34 %s, D35 %s | underside D40: %s (%s) | shooting position: %s" % (
            "DETECTING" if (c.get("obst") or [0, 0])[0] else "clear", "DETECTING" if (c.get("obst") or [0, 0, 0])[1] else "clear",
            "BALL" if c.get("under") else "clear", "trusted" if c.get("under_ok") else "IGNORED", "YES" if c.get("shot_pos") else "no"))
        if view == "ir":
            out.append("bearing map: %s | noise floor calibrated: %s" % (r.get("map", "n/a"), "yes" if r.get("cal") else "NO"))
    if view in ("all", "imu"):
        out.append("imu %s | calibration %s | accel g %s %s %s | gyro dps %s %s %s%s" % ("ok" if i.get("imu") else "NOT delivering data", i.get("imu_cal", "n/a"),
                   _f(i.get("ax_g"), "%.3f"), _f(i.get("ay_g"), "%.3f"), _f(i.get("az_g"), "%.3f"), _f(i.get("gx_dps"), "%.2f"), _f(i.get("gy_dps"), "%.2f"), _f(i.get("gz_dps"), "%.2f"), _stale(store, "i", now)))
        out.append("roll %s pitch %s yaw %s deg (relative ESTIMATES, yaw from the gyro only and drifts) | gyro bias %s dps | yaw uncertainty estimate %s deg | drift test %s%s" % (
            _f(i.get("roll_est_deg")), _f(i.get("pitch_est_deg")), _f(i.get("yaw_rel_est_deg")), _f(i.get("gyro_bias_dps"), "%.3f"), _f(i.get("yaw_unc_est_deg"), "%.2f"),
            i.get("drift", "n/a"), "" if i.get("drift_deg_min") is None else " = %.2f deg/min" % i["drift_deg_min"]))
    if view in ("all", "motors"):
        pwm = c.get("motor_command_pwm")
        pwm = tuple(pwm) if isinstance(pwm, list) and len(pwm) == 4 else ("n/a",) * 4
        out.append("motor COMMAND PWM  FL %s FR %s BL %s BR %s  (commands only: no encoder or current sensor exists)%s" % (pwm + (_stale(store, "c", now),)))
        if view == "motors":
            out.append("commanded direction %s (-1 backward, 0 stopped, +1 forward) | moving %s ms, rotating %s ms | watchdog trips %s | stale commands cleared %s%s" % (
                d.get("wheel_dir", "n/a"), d.get("moving_ms", "n/a"), d.get("turning_ms", "n/a"), d.get("wd_trips", "n/a"), d.get("stale_cmds", "n/a"), _stale(store, "d", now)))
    if view == "raw":
        for k in FRAME_KINDS:
            f = store.latest.get(k)
            out.append("[%s] %s" % (k, json.dumps(f.data, separators=(",", ":")) if f else "(none received)"))
    return "\n".join(out)
