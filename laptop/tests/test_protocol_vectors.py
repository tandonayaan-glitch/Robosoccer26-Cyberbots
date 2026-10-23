"""Known-answer and cross-implementation tests: Python's standard hmac against published vectors AND against the firmware's own C++ HMAC."""
import hashlib
import hmac
import os
import random
import unittest

from sim_harness import FirmwareSim, TEST_KEY, TEST_KEY_HEX, sim_available
import robot_protocol as rp


class StandardVectors(unittest.TestCase):
    """RFC 4231 HMAC-SHA-256 test cases (published values) - checks the reference implementation used by the client."""

    def test_rfc4231(self):
        cases = [
            (b"\x0b" * 20, b"Hi There", "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"),
            (b"Jefe", b"what do ya want for nothing?", "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"),
            (b"\xaa" * 20, b"\xdd" * 50, "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe"),
            (b"\xaa" * 131, b"Test Using Larger Than Block-Size Key - Hash Key First", "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"),
        ]
        for key, data, want in cases:
            self.assertEqual(hmac.new(key, data, hashlib.sha256).hexdigest(), want)

    def test_protocol_known_answers(self):
        """Fixed key/challenges -> fixed tags, written down independently of this code base."""
        sn, cn = bytes(range(16, 32)), bytes(range(100, 116))
        self.assertEqual(rp.proof(TEST_KEY, b"E2M", sn, cn).hex(), "d54502d4b71c0790adc761e607ec55e1")
        self.assertEqual(rp.proof(TEST_KEY, b"M2E", cn, sn).hex(), "86309f2d6e8b732daa735aaf71dc4b7e")
        self.assertEqual(rp.fingerprint(TEST_KEY), "17a56f78")
        self.assertNotEqual(rp.proof(TEST_KEY, b"E2M", sn, cn), rp.proof(TEST_KEY, b"M2E", sn, cn))     # direction labels prevent reflection

    def test_key_parsing(self):
        self.assertEqual(rp.parse_key(TEST_KEY_HEX.upper()), TEST_KEY)
        self.assertEqual(rp.parse_key("  " + TEST_KEY_HEX + "\n"), TEST_KEY)
        for bad in ("", "12", TEST_KEY_HEX[:-1], TEST_KEY_HEX + "0", "zz" + TEST_KEY_HEX[2:], "00" * 32, "ab" * 32, None):
            with self.assertRaises(rp.KeyFormatError):
                rp.parse_key(bad)


@unittest.skipUnless(sim_available(), "firmware simulator not built (test/run_tests.sh builds it)")
class FirmwareCrossCheck(unittest.TestCase):
    def test_firmware_hmac_equals_python_hmac(self):
        """200 random (key, a, b) triples: the firmware's C++ SHA-256/HMAC must give exactly the tags Python's hmac gives."""
        sim = FirmwareSim()
        rnd = random.Random(20260426)
        try:
            for i in range(200):
                key = bytes(rnd.getrandbits(8) for _ in range(32))
                if len(set(key)) == 1:
                    continue
                sim.provision(key.hex())
                a, b = bytes(rnd.getrandbits(8) for _ in range(16)), bytes(rnd.getrandbits(8) for _ in range(16))
                for label in (b"E2M", b"M2E"):
                    self.assertEqual(sim.tag(label.decode(), a, b), rp.proof(key, label, a, b).hex(), "mismatch for case %d" % i)
            self.assertIn("fingerprint " + rp.fingerprint(key), sim.usb_out.decode("latin-1"))
        finally:
            sim.close()


if __name__ == "__main__":
    unittest.main()
