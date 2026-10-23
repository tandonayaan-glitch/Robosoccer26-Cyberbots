#!/usr/bin/env python3
"""laptop_remote.py - control the VVS Cyberbots robot directly from a (Windows) laptop through the HC-05's Bluetooth serial port.

    python laptop_remote.py --list-ports             show the serial ports (the HC-05 is the "Standard Serial over Bluetooth link ... Outgoing" one)
    python laptop_remote.py --import-key             store the robot's key on this laptop (hidden input; first time on a new laptop)
    python laptop_remote.py --port COM6              connect, authenticate, and open the command prompt (use YOUR port)

No ESP32 is involved. Pairing Windows with the HC-05 only opens a Bluetooth link; the robot ignores every command until this program has proved that it knows
the shared key (docs/BLUETOOTH.md). The key is read from a private file, never printed, never logged and never sent over the air.
Emergency stop: type  x  (or press X in drive mode, or Ctrl+C). It works even when not authenticated.
"""
import argparse
import getpass
import os
import sys
import threading
import time

import keystore
import robot_link as rl
import robot_protocol as rp

HELP = """\
connection (YOU start and end the login)
  connect              log in to the robot (proves the shared key; nothing else works before this except x)
  disconnect           end the session (the robot forgets it at once); the port stays open
  link                 state of this laptop's connection (serial port, session, frames)
  quit                 stop the robot, end the session, close the port (normal exit)
robot modes (after authentication)
  defend               defending mode: hold heading, strafe to get in front of the ball
  attack               autonomous attack mode
  manual               manual mode (then use w a s d z c or 'drive')
  stop                 stop the motors and leave attack / defend / manual (does not clear the emergency stop)
  calibrate            gyro bias + receiver noise floor (robot still, ball removed)
manual driving (after 'manual'; each command moves the robot for 0.4 s)
  w s a d z c          forward, backward, turn left, turn right, strafe left, strafe right
  drive                hold-to-drive with the keyboard (Windows): w a s d z c move, SPACE stops, q leaves, x = EMERGENCY STOP
  r                    clear the emergency-stop latch (leaves the robot idle; needs authentication)
views of the robot's sensors (type 'back' to return to the full view)
  ir                   receivers A0-A5, obstacle modules D34/D35, underside sensor D40
  imu                  accelerometer, gyro, roll/pitch/yaw estimates
  motors               motor COMMAND PWM and directions (commands, not measured speed)
  ball | safety | raw  ball bearing / mode, faults, e-stop latch / raw telemetry lines
  back | all           the normal full view          off   stop printing views
other robot commands
  gyro drift 30 | gyro cal | gyro stop | tele on | tele off | tele now r
EMERGENCY STOP
  x   (or  !   or  estop)   works at any time, even when not authenticated. The robot then needs 'r' after authentication.
"""


class App:
    """Command interpreter (separate from the terminal so it can be tested)."""

    def __init__(self, link, out=print):
        self.link = link
        self.out = out
        self.view = None            # None = do not print views

    def link_state(self):
        l = self.link
        s = rl_state(l)
        return "serial %s | session: %s | frames %d (bad %d, lost %d) | robot says: %s" % ("up" if l.link_up else "DOWN", s, l.store.count, l.store.bad, l.store.lost, (l.replies[-1] if l.replies else "-"))

    def handle(self, text):
        """Returns False when the program should exit."""
        t = " ".join(text.split())
        if not t:
            return True
        low = t.lower()
        if low in ("quit", "exit"):
            return False
        if low in ("help", "?help", "h"):
            self.out(HELP)
            return True
        if low in ("x", "!", "estop", "emergency", "emergency stop"):
            ok = self.link.emergency_stop()
            self.out("EMERGENCY STOP sent." if ok else "EMERGENCY STOP COULD NOT BE SENT: the serial link is down. Switch the robot's motor power off.")
            return True
        if low == "connect":
            self.link.connect()
            self.out("logging in... (watch the messages; 'link' shows the state)")
            return True
        if low == "disconnect":
            self.link.logout()
            self.out("session ended. Type 'connect' to log in again.")
            return True
        if low == "link":
            self.out(self.link_state())
            return True
        if low == "off":
            self.view = None
            self.out("views off")
            return True
        v = rp.VIEW_ALIASES.get(low, low)
        if v in rp.VIEWS:
            self.view = v
            self.out("view: %s   ('back' = full view, 'off' = stop printing)" % v)
            return True
        if low == "drive":
            self.out("drive mode needs the interactive Windows terminal; use w a s d z c instead")
            return True
        if low in ("w", "s", "a", "d", "z", "c", "r") and not self.link.authed:
            self.out("not authenticated yet: the robot would refuse this. Type 'link' to see why.")
            return True
        try:
            line = self.link.send_command(t)
            self.out("sent: " + line)
        except ValueError as exc:
            self.out(str(exc))
        except rl.NotAuthenticated as exc:
            self.out(str(exc) + ". Type 'link' to see the session state.")
        except rl.LinkDown:
            self.out("the serial link is down; reconnect Bluetooth and restart this program")
        return True


def rl_state(link):
    if not link.link_up:
        return "serial link DOWN"
    return {rp.AUTHED: "AUTHENTICATED"}.get(link.hs.state, link.hs.state + (" (retry in %.0f s)" % max(0.0, link.hs.retry_at - link.clock()) if link.hs.state == rp.IDLE and link.hs.retry_at > link.clock() else ""))


def drive_available():
    try:
        import msvcrt  # noqa: F401
        return sys.stdin.isatty()
    except ImportError:
        return False


def drive_mode(link, out=print):
    """Hold-to-drive with the keyboard (Windows console). Each key press sends one 0.4 s lease; releasing the key stops the robot by itself."""
    import msvcrt
    keymap = {"w": "W", "s": "S", "a": "A", "d": "D", "z": "Z", "c": "C"}
    out("DRIVE MODE: hold w/s forward/back, a/d turn, z/c strafe | SPACE stop | q leave | x EMERGENCY STOP")
    try:
        link.send_command("manual")
    except (ValueError, rl.NotAuthenticated, rl.LinkDown) as exc:
        out("cannot start: %s" % exc)
        return
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getwch().lower()
            if ch == "x" or ch == "\x1b":
                link.emergency_stop()
                out("EMERGENCY STOP sent.")
                return
            if ch == "q":
                try:
                    link.send_command("Q")
                except Exception:
                    pass
                return
            if ch == " ":
                try:
                    link.send_command("Q")
                except Exception:
                    pass
            elif ch in keymap:
                try:
                    link.send_command(keymap[ch])
                except (rl.NotAuthenticated, rl.LinkDown):
                    out("session lost: robot stopped by its timeout")
                    return
        time.sleep(0.01)


def background(link, app, stop):
    last_view = 0.0
    while not stop.is_set():
        link.pump()
        for e in link.drain_events():
            print("[%s]" % e)
        now = time.monotonic()
        if app.view and link.authed:
            link.request_frames(app.view)
            if now - last_view >= 1.0:
                last_view = now
                print(rp.render_view(link.store, app.view, now, link.authed))
        stop.wait(0.02)


def open_port(name, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing. In PowerShell:  python -m pip install -r requirements.txt")
    try:
        return serial.Serial(name, baud, timeout=0, write_timeout=2)
    except (OSError, ValueError) as exc:
        sys.exit("cannot open %s: %s\nCheck the port with  python laptop_remote.py --list-ports  and that the HC-05 is paired and powered; Windows must not have another program open on it." % (name, exc))


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        sys.exit("pyserial is missing. In PowerShell:  python -m pip install -r requirements.txt")
    ports = sorted(lp.comports())
    if not ports:
        print("no serial ports found")
    for p in ports:
        print("%-8s %s" % (p.device, p.description))
    print("\nThe HC-05 appears as 'Standard Serial over Bluetooth link' with the OUTGOING direction after Windows has paired it (Settings > Bluetooth > More Bluetooth options > COM Ports).")


def import_key(path):
    print("Paste the robot's key (64 hexadecimal digits). Nothing is shown while you type.")
    text = getpass.getpass("key: ")
    try:
        key = rp.parse_key(text)
    except rp.KeyFormatError as exc:
        sys.exit("not saved: %s" % exc)
    keystore.save(path, key)
    print("saved to %s (fingerprint %s; the Mega prints the same value for ':authkey')" % (path, rp.fingerprint(key)))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Control the VVS Cyberbots robot from a laptop over the HC-05's Bluetooth serial port.")
    ap.add_argument("--port", help="the HC-05's OUTGOING Bluetooth serial port, e.g. COM6 (see --list-ports)")
    ap.add_argument("--baud", type=int, default=9600, help="serial speed of the Bluetooth port (default 9600; the HC-05 side is set on the Mega, normally nothing to change)")
    ap.add_argument("--key-file", default=keystore.default_path(), help="where the robot key is stored (default: %(default)s)")
    ap.add_argument("--auth-timeout", type=float, default=20.0, help="with --connect: seconds to wait for the login before continuing (default 20)")
    ap.add_argument("--connect", action="store_true", help="log in immediately at start-up (otherwise you type 'connect')")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--import-key", action="store_true", help="store the robot key on this laptop (hidden input)")
    a = ap.parse_args(argv)

    if a.list_ports:
        return list_ports()
    if a.import_key:
        return import_key(a.key_file)
    if not a.port:
        ap.error("--port is required (for example --port COM6); run with --list-ports to see the ports")
    try:
        key = keystore.load(a.key_file)
    except FileNotFoundError:
        sys.exit("no key stored on this laptop (%s).\nFirst laptop: python tools\\provision.py --mega-port <USB port of the Mega>.   Other laptops: python laptop\\laptop_remote.py --import-key" % a.key_file)
    except rp.KeyFormatError as exc:
        sys.exit(str(exc))

    link = rl.RobotLink(open_port(a.port, a.baud), key, auto_auth=a.connect)
    app = App(link)
    stop = threading.Event()
    th = threading.Thread(target=background, args=(link, app, stop), daemon=True)
    th.start()
    print("Opened %s (key fingerprint %s). Type 'connect' to log in, 'help' for the commands. EMERGENCY STOP: type x at any time." % (a.port, rp.fingerprint(key)))
    if a.connect:
        t0 = time.monotonic()
        while not link.authed and time.monotonic() - t0 < a.auth_timeout and link.link_up:
            time.sleep(0.1)
        print("AUTHENTICATED." if link.authed else "Not authenticated yet (%s); the program keeps trying." % rl_state(link))
    clean = False
    try:
        while True:
            try:
                text = input("robot> ")
            except EOFError:
                break
            if text.strip().lower() == "drive" and drive_available():
                drive_mode(link)
                continue
            if not app.handle(text):
                clean = True
                break
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        stop.set()
        th.join(timeout=1.0)
        if clean:
            link.close(stop=True)
            print("Robot stopped, session ended, port closed.")
        else:
            link.emergency_stop()                  # unexpected exit: ask for an emergency stop (the robot latches it; 'r' after the next login)
            link.close(stop=False)
            print("Unexpected exit: EMERGENCY STOP requested. After the next login type 'r' to clear it.")


if __name__ == "__main__":
    main()
