#!/usr/bin/env python3
"""Provision the Bluetooth key of the VVS Cyberbots robot - one command, Windows or Linux/macOS.

  python tools\\provision.py --mega-port COM3        (COM3 = the Mega's USB port, see Arduino IDE > Tools > Port)

  1. makes a random 32-byte key with Python's 'secrets' (the operating system's secure random generator)
  2. stores it on this laptop in a private file (default %LOCALAPPDATA%\\Robosoccer26\\robot.key)
  3. sends   :authkey <key>   to the Mega over its USB cable and checks that the Mega reports the same fingerprint
Nothing is flashed to the Mega and no key is written into any source file. The key is not printed unless you ask (--show-key). It never goes over Bluetooth.

  --reuse            keep the key already in the key file and (re)send it to the Mega (e.g. a replaced Mega)
  --rotate           make a NEW key (the old one stops working on the robot at once); every other laptop must import the new one
  --clear            erase the key on the Mega: nobody can control the robot over Bluetooth until a key is provisioned again
  --show-key         print the key once (to type into another laptop's  laptop_remote.py --import-key ), then clear the screen yourself
  --key-file PATH    use another key file          --print-mega-command   show the line to type into the Mega's USB Serial Monitor instead
  --selftest         check the fingerprint code against the firmware's test vector

SAFETY: opening the Mega's USB port resets it, and after a reset it starts its automatic attack after 1.5 s. Lift the wheels or disconnect the motor battery first.
The USB cable to the Mega is trusted: whoever can plug into it can set or erase the key. The key sits unencrypted in the Mega's EEPROM (see docs/BLUETOOTH.md).
"""
import argparse
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "laptop"))
import keystore                                   # noqa: E402
import robot_protocol as rp                       # noqa: E402


def mega_exchange(ser, line, expect, wait=3.0, sleep=time.sleep, clock=time.monotonic):
    """Types one line on the Mega's USB console and returns the text it printed until `expect` appears (or the wait is over). `ser` is pyserial-like."""
    ser.write((line + "\n").encode("ascii"))
    end = clock() + wait
    out = ""
    while clock() < end:
        n = ser.in_waiting
        out += (ser.read(n) if n else b"").decode("utf-8", "replace")
        if expect in out:
            break
        sleep(0.05)
    return out


def provision_mega(ser, key, sleep=time.sleep, clock=time.monotonic):
    """Stores `key` in the Mega. Returns (ok, message). The check is the fingerprint the Mega reports; the key is never part of the message."""
    ser.write(b"s\n")                                     # 'stop': cancels the automatic attack start if the board has just rebooted
    sleep(0.3)
    out = mega_exchange(ser, ":authkey " + key.hex(), "fingerprint", 3.0, sleep, clock)
    m = re.search(r"fingerprint ([0-9a-f]{8})", out)
    if m and m.group(1) == rp.fingerprint(key):
        return True, "the Mega stored the key; its fingerprint %s matches" % m.group(1)
    if "key rejected" in out:
        return False, "the Mega rejected the key"
    return False, "the Mega did not confirm the key (is this the Mega's USB port, with the current firmware flashed?)"


def clear_mega(ser, sleep=time.sleep, clock=time.monotonic):
    ser.write(b"s\n")
    sleep(0.3)
    out = mega_exchange(ser, ":authkey clear", "ERASED", 3.0, sleep, clock)
    return ("ERASED" in out), ("key erased on the Mega" if "ERASED" in out else "no confirmation from the Mega")


def open_mega(port, wait_boot):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing:  python -m pip install -r laptop\\requirements.txt")
    ser = serial.Serial(port, 115200, timeout=0)
    print("opened %s; the Mega resets when its USB port opens - waiting %.0f s for it to finish booting" % (port, wait_boot))
    time.sleep(wait_boot)
    ser.reset_input_buffer()
    return ser


def main(argv=None):
    ap = argparse.ArgumentParser(description="Provision the VVS Cyberbots Bluetooth key (see the module docstring).")
    ap.add_argument("--mega-port"); ap.add_argument("--key-file", default=keystore.default_path())
    ap.add_argument("--reuse", action="store_true"); ap.add_argument("--rotate", action="store_true"); ap.add_argument("--clear", action="store_true")
    ap.add_argument("--show-key", action="store_true"); ap.add_argument("--print-mega-command", action="store_true")
    ap.add_argument("--boot-wait", type=float, default=12.0, help="seconds to wait after opening the port (the Mega reboots and calibrates the gyro)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)

    if a.selftest:
        k = bytes.fromhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f")
        assert rp.fingerprint(k) == "17a56f78", rp.fingerprint(k)
        assert rp.proof(k, b"E2M", bytes(range(16, 32)), bytes(range(100, 116))).hex() == "d54502d4b71c0790adc761e607ec55e1"
        print("selftest OK (fingerprint and protocol tag match the firmware test vectors)")
        return 0

    if a.clear:
        if not a.mega_port:
            sys.exit("--clear needs --mega-port")
        ok, msg = clear_mega(open_mega(a.mega_port, a.boot_wait))
        print(msg)
        return 0 if ok else 1

    have = keystore.exists(a.key_file)
    if a.reuse:
        if not have:
            sys.exit("--reuse: there is no key file at %s" % a.key_file)
        key = keystore.load(a.key_file)
        print("using the key in %s (fingerprint %s)" % (a.key_file, rp.fingerprint(key)))
    else:
        if have and not a.rotate:
            sys.exit("%s already holds a key. Use --reuse to send it to a Mega, or --rotate to replace it with a new one (every other laptop then needs the new key)." % a.key_file)
        key = keystore.generate()
        keystore.save(a.key_file, key)
        print("new key stored in %s (private file; never commit or e-mail it). Fingerprint %s." % (a.key_file, rp.fingerprint(key)))
    if a.show_key:
        print("KEY (shown once): " + key.hex())
    if a.print_mega_command:
        print("type this into the Mega's USB Serial Monitor (line ending Newline):\n  :authkey " + key.hex())
    if a.mega_port:
        print("SAFETY: wheels lifted or motor battery disconnected? The Mega resets and may start its automatic attack.")
        ok, msg = provision_mega(open_mega(a.mega_port, a.boot_wait), key)
        print(msg)
        if not ok:
            return 1
        print("Nothing to flash. Next: python laptop\\laptop_remote.py --port <the HC-05's Bluetooth COM port>")
    else:
        print("Next: rerun with --mega-port <the Mega's USB port> (or use --print-mega-command) to store the key in the robot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
