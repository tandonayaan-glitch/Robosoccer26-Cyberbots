"""The Python client against the real firmware logic: valid and invalid authentication, tampering, replay, expiry, uniqueness, sessions, lockout,
unauthorized commands, emergency stop and latch, provisioning, rotation, and secret leakage. HOST SIMULATION: no radio, no HC-05, no robot."""
import re
import unittest

from sim_harness import Env, FirmwareSim, OTHER_KEY, TEST_KEY, TEST_KEY_HEX, sim_available
import robot_protocol as rp

skip = unittest.skipUnless(sim_available(), "firmware simulator not built (test/run_tests.sh builds it)")


def lines(buf):
    return [l for l in bytes(buf).decode("latin-1").split("\n") if l]


def raw_session(sim):
    """A bare peer (no client): returns helpers to speak the line protocol directly."""
    class P:
        def send(self, text, wait=60):
            sim.rx(text + "\n")
            sim.run(wait)

        def take(self):
            out = lines(sim.tx)
            sim.tx.clear()
            return out
    return P()


def resp_line(key, ch_line, cn, corrupt=False):
    sn = bytes.fromhex(ch_line[8:])
    tag = bytearray(rp.proof(key, b"E2M", sn, cn))
    if corrupt:
        tag[3] ^= 1
    return "AUTH RESP %s %s" % (cn.hex(), bytes(tag).hex())


@skip
class ValidAndInvalid(unittest.TestCase):
    def setUp(self):
        self.env = None

    def tearDown(self):
        if self.env:
            self.env.close()

    def test_valid_authentication_and_commands(self):
        self.env = e = Env()
        self.assertFalse(e.sim.state()["authed"] == "1")
        self.assertTrue(e.login())
        self.assertTrue(e.link.authed and e.link.hs.authed)
        e.advance(200)
        e.link.send_command("manual"); e.advance(200)
        self.assertEqual(e.sim.state()["manual"], "1")
        e.link.send_command("defend"); e.advance(300)
        st = e.sim.state()
        self.assertEqual((st["defend"], st["mode"]), ("1", "DEFEND"))
        e.advance(3000)
        self.assertGreater(e.link.store.count, 0)                      # telemetry flows once authenticated
        self.assertIn("DEFEND", e.link.store.get("c").get("mode", ""))

    def test_wrong_key_never_authenticates_and_is_throttled(self):
        self.env = e = Env(key=OTHER_KEY)
        e.advance(20000)
        self.assertFalse(e.link.authed)
        st = e.sim.state()
        self.assertEqual(st["authed"], "0")
        self.assertGreaterEqual(int(st["fails"]), 1)
        self.assertTrue(any("refused" in ev or "throttling" in ev for ev in e.link.drain_events()))
        # a client with the right key gets in once any lockout is over
        e.link.hs.key = TEST_KEY
        e.link.hs.reset(e.clock(), 0.0)
        e.advance(70000)
        self.assertTrue(e.link.authed)

    def test_unauthenticated_peer_cannot_control_or_read(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        for cmd in ("M", "U", "W", "R", "Q", "manual", "defend", "attack", "calibrate", "stop", "motors", "gyro cal", "tele now r", "set maxauto 255", "AUTHX"):
            p.send(cmd, 400)
        got = p.take()
        self.assertIn("ERR:AUTH", got)
        st = e.sim.state()
        self.assertEqual((st["manual"], st["defend"], st["auto"], st["wheels"]), ("0", "0", "0", "0,0,0,0"))
        e.sim.run(3000)
        self.assertFalse(any(l.startswith("{") for l in lines(e.sim.tx)), "no telemetry before authentication")
        p.send("P")
        self.assertIn("PONG:AUTH", p.take())

    def test_malformed_and_tampered_proofs(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        cn = bytes(range(50, 66))
        bad_builders = [
            lambda ch: resp_line(TEST_KEY, ch, cn, corrupt=True),                       # one flipped bit in the tag
            lambda ch: resp_line(OTHER_KEY, ch, cn),                                    # wrong key
            lambda ch: "AUTH RESP zz",
            lambda ch: "AUTH RESP",
            lambda ch: "AUTH RESP " + cn.hex(),                                         # no tag
            lambda ch: "AUTH RESP %s %s" % (cn.hex(), "00" * 15),                       # tag too short
            lambda ch: "AUTH RESP %s %s" % (cn.hex(), "00" * 17),                       # tag too long
            lambda ch: "AUTH RESP %s %s" % ("g" * 32, "00" * 16),                       # not hex
            lambda ch: "AUTH RESP %s  %s" % (cn.hex(), "00" * 16),                      # extra space
            lambda ch: "AUTH BOGUS",
        ]
        for i, build in enumerate(bad_builders):
            e.sim.run(5000 if i in (3, 6, 7) else 400)                                 # (lets any lockout lapse)
            p.send("AUTH")
            ch = [l for l in p.take() if l.startswith("AUTH CH ")]
            if not ch:
                e.sim.run(70000); p.send("AUTH"); ch = [l for l in p.take() if l.startswith("AUTH CH ")]
            self.assertTrue(ch, "no challenge for case %d" % i)
            p.send(build(ch[-1]))
            self.assertEqual(e.sim.state()["authed"], "0", "case %d must not authenticate" % i)
            p.take()
        # and the correct proof still works afterwards (after the lockout lapses)
        e.sim.run(70000)
        p.send("AUTH")
        ch = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
        p.send(resp_line(TEST_KEY, ch, cn))
        out = p.take()
        self.assertTrue(any(l.startswith("AUTH OK ") for l in out))
        self.assertEqual(e.sim.state()["authed"], "1")
        ok = [l for l in out if l.startswith("AUTH OK ")][0]
        self.assertEqual(ok[8:], rp.proof(TEST_KEY, b"M2E", cn, bytes.fromhex(ch[8:])).hex())     # the robot's own proof is correct too

    def test_lockout_after_three_failures(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        for _ in range(3):
            e.sim.run(400); p.send("AUTH")
            ch = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
            p.send(resp_line(OTHER_KEY, ch, bytes(16)[:0] + bytes(range(16))))
            p.take()
        self.assertEqual(e.sim.state()["fails"], "3")
        e.sim.run(400); p.send("AUTH")
        self.assertIn("ERR:AUTH_LOCKOUT", p.take())
        e.sim.run(4500); p.send("AUTH")
        self.assertTrue(any(l.startswith("AUTH CH ") for l in p.take()))

    def test_replay_is_rejected(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        cn = bytes(range(70, 86))
        p.send("AUTH")
        ch1 = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
        recorded = resp_line(TEST_KEY, ch1, cn)
        p.send(recorded)
        self.assertTrue(any(l.startswith("AUTH OK ") for l in p.take()))
        p.send(recorded)                                                                # the same proof again, session open
        self.assertIn("ERR:AUTH_NOCHAL", p.take())
        p.send("AUTH OUT"); p.take()
        p.send(recorded)                                                                # replay after the session ended
        self.assertIn("ERR:AUTH_NOCHAL", p.take())
        self.assertEqual(e.sim.state()["authed"], "0")
        e.sim.run(400); p.send("AUTH")
        ch2 = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
        self.assertNotEqual(ch1, ch2)
        p.send(recorded)                                                                # old proof against a NEW challenge
        self.assertIn("ERR:AUTH_FAIL", p.take())
        self.assertEqual(e.sim.state()["authed"], "0")

    def test_replayed_robot_proof_is_rejected_by_the_client(self):
        """A fake robot that replays an old 'AUTH OK' cannot make the client believe it is talking to the real robot."""
        hs1 = rp.Handshake(TEST_KEY, rng=lambda n: bytes(range(n)))
        hs1.start(0.0)
        _, resp, _ = hs1.on_line("AUTH CH " + "11" * 16, 0.1)
        old_ok = "AUTH OK " + rp.proof(TEST_KEY, b"M2E", hs1.cn, hs1.sn).hex()
        _, _, ev = hs1.on_line(old_ok, 0.2)
        self.assertEqual(ev, "authenticated")
        hs2 = rp.Handshake(TEST_KEY, rng=lambda n: bytes(range(10, 10 + n)))
        hs2.start(10.0)
        hs2.on_line("AUTH CH " + "22" * 16, 10.1)
        _, _, ev = hs2.on_line(old_ok, 10.2)
        self.assertFalse(hs2.authed)
        self.assertIn("did NOT prove", ev)

    def test_challenge_expiry(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        p.send("AUTH")
        ch = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
        for _ in range(22):                                                             # 11 s with pings so the link stays alive
            p.send("P", 500)
        p.take()
        p.send(resp_line(TEST_KEY, ch, bytes(range(90, 106))))
        self.assertIn("ERR:AUTH_EXPIRED", p.take())
        self.assertEqual(e.sim.state()["authed"], "0")
        # just in time works
        e.sim.run(400); p.send("AUTH")
        ch = [l for l in p.take() if l.startswith("AUTH CH ")][-1]
        for _ in range(16):
            p.send("P", 500)
        p.take()
        p.send(resp_line(TEST_KEY, ch, bytes(range(90, 106))))
        self.assertTrue(any(l.startswith("AUTH OK ") for l in p.take()))

    def test_challenges_are_unique_including_across_reboots(self):
        self.env = e = Env(auto_auth=False)
        seen = set()
        p = raw_session(e.sim)
        for boot in range(4):
            for _ in range(12):
                e.sim.run(400); p.send("AUTH")
                for l in p.take():
                    if l.startswith("AUTH CH "):
                        self.assertNotIn(l, seen)
                        seen.add(l)
            e.sim.reboot(True)
            e.sim.tx.clear()
        self.assertEqual(len(seen), 48)
        self.assertGreaterEqual(int(e.sim.state()["bootctr"]), 4)

    def test_session_timeout_and_reconnect(self):
        self.env = e = Env()
        self.assertTrue(e.login())
        e.link.send_command("manual"); e.advance(200)
        e.link.send_command("W")
        e.silence(3000)                                                                # the link dies: nothing in either direction
        st = e.sim.state()
        self.assertEqual(st["authed"], "0")
        self.assertEqual(st["wheels"], "0,0,0,0")                                       # no motor command stays active
        e.advance(5000)                                                                 # the laptop comes back: it must authenticate again by itself
        self.assertTrue(e.link.authed)
        self.assertEqual(e.sim.state()["authed"], "1")
        # a new AUTH from anybody ends the old session at once
        p = raw_session(e.sim)
        e.sim.tx.clear()
        p.send("AUTH")
        self.assertEqual(e.sim.state()["authed"], "0")

    def test_interrupted_handshake_leaves_nothing_running(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        p.send("AUTH"); p.take()
        e.sim.run(15000)
        st = e.sim.state()
        self.assertEqual((st["authed"], st["wheels"], st["manual"]), ("0", "0,0,0,0", "0"))

    def test_emergency_stop_unauthenticated_and_latch_untouched(self):
        self.env = e = Env(auto_auth=False)
        p = raw_session(e.sim)
        p.send("X", 100)
        st = e.sim.state()
        self.assertEqual((st["estop"], st["wheels"], st["authed"]), ("1", "0,0,0,0", "0"))
        e.link.want_session = True                                                      # now authenticate: the latch must survive it
        self.assertTrue(e.login(8000))
        self.assertEqual(e.sim.state()["estop"], "1")
        p.take()
        e.link.send_command("manual"); e.advance(200)
        self.assertEqual(e.sim.state()["manual"], "0")
        self.assertIn("ERR:ESTOP", [l for l in lines(e.sim.tx)] + e.link.replies)
        e.link.send_command("R"); e.advance(200)                                       # only an authenticated R clears it
        st = e.sim.state()
        self.assertEqual((st["estop"], st["manual"], st["auto"]), ("0", "0", "0"))

    def test_emergency_stop_from_a_running_session(self):
        self.env = e = Env()
        self.assertTrue(e.login())
        e.link.send_command("manual"); e.advance(200)
        for _ in range(5):
            e.link.send_command("W"); e.advance(100)
        self.assertNotEqual(e.sim.state()["wheels"], "0,0,0,0")
        self.assertTrue(e.link.emergency_stop())
        e.advance(40)
        st = e.sim.state()
        self.assertEqual((st["estop"], st["wheels"]), ("1", "0,0,0,0"))

    def test_robot_without_key_fails_closed(self):
        self.env = e = Env(provision=False)
        e.advance(6000)
        self.assertFalse(e.link.authed)
        self.assertTrue(any("no key" in ev for ev in e.link.drain_events()))
        p = raw_session(e.sim)
        e.sim.tx.clear()
        p.send("manual"); p.send("M")
        self.assertIn("ERR:NO_KEY", p.take())
        self.assertEqual(e.sim.state()["manual"], "0")
        p.send("X", 50)
        self.assertEqual(e.sim.state()["estop"], "1")

    def test_robot_with_enforcement_off_is_not_trusted(self):
        self.env = e = Env(enforce=False, provision=True)
        e.advance(3000)
        self.assertFalse(e.link.authed)
        self.assertTrue(e.link.hs.robot_open)
        self.assertTrue(any("NOT enforcing" in ev for ev in e.link.drain_events()))


@skip
class Provisioning(unittest.TestCase):
    def test_provision_rotate_clear_and_no_leak(self):
        sim = FirmwareSim()
        try:
            # first key
            sim.provision(TEST_KEY_HEX)
            out = sim.usb_out.decode("latin-1")
            self.assertIn("fingerprint " + rp.fingerprint(TEST_KEY), out)
            self.assertNotIn(TEST_KEY_HEX, out)
            sim.usb_out.clear()
            sim.usb(":authkey\n"); sim.run(100)
            self.assertIn("fingerprint " + rp.fingerprint(TEST_KEY), sim.usb_out.decode("latin-1"))
            # bad keys are refused and do not replace the stored one
            for bad in ("1234", "zz" + TEST_KEY_HEX[2:], "00" * 32, TEST_KEY_HEX + "00", TEST_KEY_HEX[:-1]):
                sim.usb_out.clear()
                sim.usb(":authkey " + bad + "\n"); sim.run(100)
                self.assertIn("key rejected", sim.usb_out.decode("latin-1"))
            sim.usb_out.clear(); sim.usb(":authkey\n"); sim.run(100)
            self.assertIn(rp.fingerprint(TEST_KEY), sim.usb_out.decode("latin-1"))
            # survives a reboot
            sim.reboot(True)
            self.assertEqual(sim.state()["key"], "1")
            # rotation: the old key stops working, the new one works
            new_key = bytes(range(200, 232))
            sim.provision(new_key.hex())
            self.assertEqual(sim.state()["authed"], "0")
            for key, expect in ((TEST_KEY, False), (new_key, True)):
                env_ser = __import__("sim_harness").SimSerial(sim)
                clock = __import__("sim_harness").Clock()
                link = __import__("robot_link").RobotLink(env_ser, key, clock=clock)
                for _ in range(300):
                    clock.t += 0.02; sim.run(20); link.pump()
                self.assertEqual(link.authed, expect)
                link.close(stop=False)
            # clear: nobody can log in; X still works
            sim.usb_out.clear()
            sim.usb(":authkey clear\n"); sim.run(100)
            self.assertIn("ERASED", sim.usb_out.decode("latin-1"))
            self.assertEqual(sim.state()["key"], "0")
            sim.rx("AUTH\n"); sim.run(100)
            self.assertIn("ERR:NO_KEY", sim.tx.decode("latin-1"))
            # nothing secret anywhere the robot ever wrote
            for secret in (TEST_KEY, new_key):
                for fmt in (secret.hex(), secret.hex().upper()):
                    self.assertNotIn(fmt, bytes(sim.all_tx).decode("latin-1"))
                    self.assertNotIn(fmt, bytes(sim.all_usb).decode("latin-1"))
        finally:
            sim.close()

    def test_tools_provision_against_the_firmware(self):
        import importlib.util, os
        spec = importlib.util.spec_from_file_location("provision", os.path.join(os.path.dirname(__file__), "..", "..", "tools", "provision.py"))
        prov = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(prov)
        sim = FirmwareSim()
        try:
            class Usb:                                                               # the Mega's USB console as a pyserial-like object
                def __init__(self):
                    self.sent = bytearray()
                def write(self, data):
                    self.sent += data
                    sim.usb(data.decode("ascii"))
                    sim.run(150)
                @property
                def in_waiting(self):
                    return len(sim.usb_out)
                def read(self, n):
                    b = bytes(sim.usb_out[:n]); del sim.usb_out[:n]; return b
            ser = Usb()
            key = bytes(range(1, 33))
            t = [0.0]
            ok, msg = prov.provision_mega(ser, key, sleep=lambda s: None, clock=lambda: (t.__setitem__(0, t[0] + 0.05) or t[0]))
            self.assertTrue(ok, msg)
            self.assertNotIn(key.hex(), msg)
            ok, msg = prov.provision_mega(ser, key, sleep=lambda s: None, clock=lambda: (t.__setitem__(0, t[0] + 0.05) or t[0]))
            self.assertTrue(ok)
            ok, msg = prov.clear_mega(ser, sleep=lambda s: None, clock=lambda: (t.__setitem__(0, t[0] + 0.05) or t[0]))
            self.assertTrue(ok, msg)
        finally:
            sim.close()


@skip
class NoSecretLeakage(unittest.TestCase):
    def test_full_session_leaks_nothing(self):
        e = Env()
        try:
            self.assertTrue(e.login())
            e.link.send_command("manual"); e.advance(300)
            e.link.send_command("defend"); e.advance(3000)
            for view in ("ir", "imu", "motors", "all"):
                rp.render_view(e.link.store, view, e.clock(), True)
            events = "\n".join(e.link.drain_events())
            for secret in (TEST_KEY_HEX, TEST_KEY_HEX.upper()):
                self.assertNotIn(secret, bytes(e.sim.all_tx).decode("latin-1"))
                self.assertNotIn(secret, bytes(e.sim.all_usb).decode("latin-1"))
                self.assertNotIn(secret, bytes(e.ser.written).decode("latin-1"))
                self.assertNotIn(secret, events)
            # the key bytes themselves never appear in what the laptop wrote either
            self.assertNotIn(TEST_KEY, bytes(e.ser.written))
        finally:
            e.close()


if __name__ == "__main__":
    unittest.main()
