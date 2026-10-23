"""Serial-level behaviour of the client with fake ports: disconnects, junk, oversized lines, command filtering, human-initiated login, key storage, views."""
import os
import stat
import sys
import tempfile
import unittest

from sim_harness import Clock, Env, TEST_KEY, TEST_KEY_HEX, sim_available
import keystore
import laptop_remote as app_mod
import robot_link as rl
import robot_protocol as rp


class FakeSerial:
    def __init__(self):
        self.rxbuf = bytearray()
        self.written = bytearray()
        self.fail_write = False
        self.fail_read = False
        self.closed = False

    @property
    def in_waiting(self):
        if self.fail_read:
            raise OSError("device disconnected")
        return len(self.rxbuf)

    def read(self, n=1):
        b = bytes(self.rxbuf[:n]); del self.rxbuf[:n]; return b

    def write(self, data):
        if self.fail_write:
            raise OSError("write failed")
        self.written += data
        return len(data)

    def close(self):
        self.closed = True


def make(auto=False):
    ser, clock = FakeSerial(), Clock()
    link = rl.RobotLink(ser, TEST_KEY, clock=clock, rng=lambda n: bytes(range(n)), auto_auth=auto)
    return ser, clock, link


class CommandFiltering(unittest.TestCase):
    def test_allowed_and_refused(self):
        ok = {"w": "W", "M": "M", "x": "X", "!": "X", "Defend": "defend", "gyro drift 30": "gyro drift 30", "tele now r": "tele now r", "calibrate": "calibrate", "R": "R", "?": "?"}
        for text, want in ok.items():
            self.assertEqual(rp.normalize_command(text), (want, None), text)
        for bad in ("", "set maxauto 255", ":authkey " + TEST_KEY_HEX, "digitalwrite 22 1", "AUTH", "AUTH RESP x y", "gyro drift 30000", "tele now q", "w" * 30, "launch", "esp32connect pass"):
            line, err = rp.normalize_command(bad)
            self.assertIsNone(line, bad)
            self.assertTrue(err, bad)

    def test_commands_need_a_session_but_emergency_stop_does_not(self):
        ser, clock, link = make()
        with self.assertRaises(rl.NotAuthenticated):
            link.send_command("manual")
        with self.assertRaises(ValueError):
            link.send_command("set maxauto 255")
        link.send_command("x")
        self.assertEqual(bytes(ser.written), b"X\n")


class HumanInitiatedLogin(unittest.TestCase):
    def test_nothing_is_sent_until_the_human_connects(self):
        ser, clock, link = make(auto=False)
        for _ in range(100):
            clock.t += 0.05
            link.pump()
        self.assertEqual(bytes(ser.written), b"")
        app = app_mod.App(link, out=lambda s: None)
        app.handle("connect")
        clock.t += 0.05
        link.pump()
        self.assertEqual(bytes(ser.written), b"AUTH\n")
        app.handle("disconnect")
        self.assertFalse(link.want_session)


class RobustInput(unittest.TestCase):
    def test_junk_oversize_and_telemetry_before_login(self):
        ser, clock, link = make()
        ser.rxbuf += bytes([0, 1, 2, 255, 254]) + b"\n" + b"A" * 5000 + b"\n" + b"{\"v\":1,\"k\":\"c\",\"n\":1,\"ms\":5}\n" + b"\xff\xfe garbage\r\n" + b"OK:M\n"
        link.pump()
        self.assertGreaterEqual(link.garbage, 3)
        self.assertEqual(link.store.count, 0)                    # telemetry from a peer that is not authenticated is not trusted
        self.assertEqual(link.unauth_dropped, 1)
        self.assertIn("OK:M", link.replies)

    def test_write_failure_marks_the_link_down_and_never_raises_from_emergency_stop(self):
        ser, clock, link = make()
        ser.fail_write = True
        self.assertFalse(link.emergency_stop())
        self.assertFalse(link.link_up)
        self.assertTrue(any("serial link lost" in e for e in link.drain_events()))
        with self.assertRaises((rl.LinkDown, rl.NotAuthenticated)):
            link.send_command("manual")
        link.pump()                                              # a dead link is a no-op, not a crash

    def test_read_failure(self):
        ser, clock, link = make()
        ser.fail_read = True
        link.pump()
        self.assertFalse(link.link_up)

    def test_close_sends_stop_and_logout_only_when_logged_in(self):
        ser, clock, link = make()
        link.hs.state = rp.AUTHED
        link.close(stop=True)
        self.assertEqual(bytes(ser.written), b"stop\nAUTH OUT\n")
        self.assertTrue(ser.closed)

    def test_handshake_timeout_and_malformed_challenge(self):
        hs = rp.Handshake(TEST_KEY, rng=lambda n: bytes(n))
        hs.start(0.0)
        hs.tick(5.0)
        self.assertEqual(hs.state, rp.IDLE)
        self.assertGreater(hs.retry_at, 5.0)
        hs.start(10.0)
        _, send, ev = hs.on_line("AUTH CH nothex", 10.1)
        self.assertIsNone(send)
        self.assertIn("malformed", ev)
        hs.start(20.0)
        for stale in ("AUTH OK " + "00" * 16, "AUTH CH " + "11" * 16 + "22"):
            _, send, ev = hs.on_line(stale, 20.1)
            self.assertFalse(hs.authed)


class Views(unittest.TestCase):
    def test_render_with_missing_data(self):
        store = rp.FrameStore()
        for v in rp.VIEWS:
            text = rp.render_view(store, v, 5.0, False)
            self.assertIn("NOT AUTHENTICATED", text)
        self.assertIn("n/a", rp.render_view(store, "imu", 5.0, True))
        self.assertIn("no data yet", rp.render_view(store, "ir", 5.0, True))

    def test_frame_validation_and_loss_counting(self):
        store = rp.FrameStore()
        good = '{"v":1,"k":"c","n":10,"ms":5,"mode":"IDLE","bearing_deg":null}'
        self.assertIsNotNone(store.add(good, 1.0))
        for bad in ("", "{", "{}", '{"v":2,"k":"c","n":1,"ms":5}', '{"v":1,"k":"z","n":1,"ms":5}', '{"v":1,"k":"c","n":-1,"ms":5}', '{"v":1,"k":"c","n":"1","ms":5}', "x" * 500, "[1,2]"):
            self.assertIsNone(store.add(bad, 2.0))
        store.add('{"v":1,"k":"c","n":12,"ms":6}', 3.0)
        self.assertEqual((store.bad, store.lost, store.count), (9, 1, 2))
        self.assertIn("bearing null shown as n/a", "bearing null shown as " + ("n/a" if rp._f(None) == "n/a" else "?"))


class KeyStorage(unittest.TestCase):
    def test_save_load_permissions_and_errors_never_show_the_key(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "sub", "robot.key")
            k = keystore.generate()
            keystore.save(path, k)
            self.assertEqual(keystore.load(path), k)
            if os.name != "nt":
                self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)
            self.assertNotEqual(keystore.generate(), keystore.generate())
            with open(path, "w") as f:
                f.write("not a key " + TEST_KEY_HEX[:10])
            with self.assertRaises(rp.KeyFormatError) as cm:
                keystore.load(path)
            self.assertNotIn(TEST_KEY_HEX[:10], str(cm.exception))
            with self.assertRaises(FileNotFoundError):
                keystore.load(os.path.join(d, "missing.key"))


@unittest.skipUnless(sim_available(), "firmware simulator not built")
class AppAgainstFirmware(unittest.TestCase):
    def test_connect_command_logs_in_and_views_show_robot_data(self):
        e = Env(auto_auth=False)
        try:
            out = []
            app = app_mod.App(e.link, out=out.append)
            e.advance(500)
            self.assertFalse(e.link.authed)
            app.handle("manual")
            self.assertTrue(any("not authenticated" in o for o in out))
            app.handle("connect")
            self.assertTrue(e.login(5000))
            app.handle("ir")
            for _ in range(60):
                e.link.request_frames("ir"); e.step(100)
            text = rp.render_view(e.link.store, "ir", e.clock(), True)
            self.assertIn("A0", text); self.assertIn("threshold", text); self.assertNotIn("[no data yet]", text.split("\n")[1])
            app.handle("x")
            e.advance(100)
            self.assertEqual(e.sim.state()["estop"], "1")
            self.assertTrue(any("EMERGENCY STOP sent" in o for o in out))
            app.handle("disconnect")
            e.advance(100)
            self.assertEqual(e.sim.state()["authed"], "0")
            self.assertFalse(app.handle("quit"))
        finally:
            e.close()


if __name__ == "__main__":
    unittest.main()
