"""robot_link.py - serial I/O and session handling on top of robot_protocol.py.

RobotLink works with any object that has pyserial's in_waiting / read(n) / write(bytes) / close(), so it is tested with fakes and with the firmware simulation.
All timing uses the injected `clock`, so tests run in virtual time. Nothing here prints or logs the key; events are short human-readable strings.
"""
import os
import threading
import time

import robot_protocol as rp


class LinkDown(Exception):
    """The serial port failed (Bluetooth lost, port closed, device unplugged)."""


class NotAuthenticated(Exception):
    """A command needing an authenticated session was attempted without one."""


class RobotLink:
    SILENT_AFTER_S = 4.0          # an authenticated session with nothing at all from the robot for this long is treated as lost

    def __init__(self, ser, key, clock=time.monotonic, rng=os.urandom, auto_auth=True):
        self.ser = ser
        self.clock = clock
        self.hs = rp.Handshake(key, rng)
        self.store = rp.FrameStore()
        self.link_up = True
        self.want_session = auto_auth
        self.events = []
        self.replies = []
        self.unauth_dropped = 0
        self.garbage = 0
        self._buf = bytearray()
        self._overflow = False
        self._last_ping = 0.0
        self._last_rx = clock()
        self._last_req = 0.0
        self._req_n = 0
        self._lock = threading.RLock()

    # ---------------------------------------------------------------- state
    @property
    def authed(self):
        return self.link_up and self.hs.authed

    def drain_events(self):
        with self._lock:
            e, self.events = self.events, []
        return e

    def _event(self, text):
        self.events.append(text)

    def _down(self, why):
        if self.link_up:
            self.link_up = False
            self.hs.reset(self.clock(), 0.0)
            self._event("serial link lost: %s" % why)

    # ---------------------------------------------------------------- output
    def _write(self, line):
        with self._lock:
            if not self.link_up:
                raise LinkDown("serial link is down")
            try:
                self.ser.write((line + "\n").encode("ascii"))
            except (OSError, ValueError, AttributeError) as exc:
                self._down(type(exc).__name__)
                raise LinkDown("write failed")

    def send_command(self, text):
        """Validates and sends one command. 'X' (emergency stop) is always allowed; everything else needs an authenticated session."""
        line, err = rp.normalize_command(text)
        if err:
            raise ValueError(err)
        if line == "X":
            self.emergency_stop()
            return line
        if not self.authed:
            raise NotAuthenticated("not authenticated: the robot would refuse '%s'" % line)
        self._write(line)
        return line

    def emergency_stop(self):
        """Best effort, never raises. Works without a session (the robot always accepts X)."""
        try:
            self._write("X")
            return True
        except LinkDown:
            return False

    def connect(self):
        """Human-initiated: start (or restart) the login handshake."""
        with self._lock:
            self.want_session = True
            if not self.hs.authed:
                self.hs.reset(self.clock(), 0.0)

    def logout(self):
        if self.authed:
            try:
                self._write("AUTH OUT")
            except LinkDown:
                pass
        self.hs.reset(self.clock(), 0.0)
        self.want_session = False

    def close(self, stop=True):
        """Normal shutdown: stop the robot (without latching the emergency stop), end the session, close the port."""
        with self._lock:
            try:
                if stop and self.authed:
                    self._write("stop")
                self.logout()
            except LinkDown:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.link_up = False

    # ---------------------------------------------------------------- input
    def _feed(self, data, now):
        for b in data:
            if b == 10:                                           # \n
                if self._overflow:
                    self._overflow = False
                    self.garbage += 1
                else:
                    raw = bytes(self._buf).rstrip(b"\r")
                    if raw:
                        self._line(raw, now)
                self._buf.clear()
            elif len(self._buf) < rp.MAX_LINE:
                self._buf.append(b)
            else:
                self._overflow = True

    def _line(self, raw, now):
        if any(c < 32 or c > 126 for c in raw):
            self.garbage += 1                                     # binary junk (a half-open Bluetooth port can produce it): ignored
            return
        line = raw.decode("ascii")
        self._last_rx = now
        if line.startswith("{"):
            if self.authed:
                self.store.add(line, now)
            else:
                self.unauth_dropped += 1                          # telemetry is only trusted from an authenticated session
            return
        consumed, send, event = self.hs.on_line(line, now)
        if send:
            self._write(send)
        if event:
            self._event(event)
        if consumed:
            return
        if line.startswith("PONG:"):
            return
        self.replies.append(line)
        del self.replies[:-50]
        self._event("robot: " + line)

    def pump(self):
        """Call every ~20 ms (the app runs it on a thread). Reads, handles, pings, re-authenticates."""
        with self._lock:
            if not self.link_up:
                return
            now = self.clock()
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n) if n else b""
            except (OSError, ValueError, AttributeError, TypeError) as exc:
                self._down(type(exc).__name__)
                return
            if data:
                try:
                    self._feed(data, now)
                except LinkDown:
                    return
            was = self.hs.authed
            self.hs.tick(now)
            if was and not self.hs.authed:
                self._event("session ended")
            if self.hs.authed and now - self._last_rx > self.SILENT_AFTER_S:
                self.hs.reset(now, 0.0)
                self._event("the robot stopped answering: session considered lost")
            try:
                if self.want_session and self.hs.can_start(now):
                    self._write(self.hs.start(now))
                if self.hs.authed and now - self._last_ping >= rp.PING_INTERVAL_S:
                    self._last_ping = now
                    self._write("P")
            except LinkDown:
                return

    def request_frames(self, view):
        """Ask the robot for fresh telemetry frames for a view (core plus the kind that view needs), at most every 0.5 s."""
        now = self.clock()
        if not self.authed or now - self._last_req < 0.5:
            return
        self._last_req = now
        if view == "raw":
            kind = "crid"[self._req_n % 4]
        else:
            kind = "c" if self._req_n % 2 == 0 else rp.EXTRA_KIND.get(view, "c")
        self._req_n += 1
        try:
            self._write("tele now " + kind)
        except LinkDown:
            pass
