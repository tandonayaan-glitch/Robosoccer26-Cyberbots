#!/usr/bin/env python3
"""Download the robot's event records (kept in the Mega's EEPROM) into a text file on the PC.

    python3 tools/get_records.py                  # find the Mega, send ':log', save records.txt
    python3 tools/get_records.py COM5             # (Windows)  or  /dev/ttyACM0  (Linux)  /dev/cu.usbmodem1101  (macOS)
    python3 tools/get_records.py --follow         # keep running and save EVERYTHING the robot prints (with PC time stamps) until Ctrl-C
    python3 tools/get_records.py --clear          # erase the records on the robot
    python3 tools/get_records.py --out my.txt --baud 115200

Needs pyserial:  pip install pyserial.   Close the Arduino IDE's Serial Monitor first (only one program can use the port).
Opening the port normally RESETS an Arduino Mega (that is how uploading works): the robot restarts, the records are NOT lost, and a new BOOT record is added.
The robot never moves after a reset (autonomy is off until you send 'a'). Nothing here uploads firmware.
"""
import argparse
import re
import sys
import time

PREFIX = re.compile(r"^\[[A-Z]+\] \d+\.\d{3} ")


def clean(line):
    """Drop the '[LOG] 12.345 ' prefix the firmware puts on its own lines."""
    return PREFIX.sub("", line.rstrip("\r\n"))


def extract_records(lines):
    """Return the lines between '#RECORDS' and '#END-RECORDS' (both included), or None if the end marker was not seen."""
    out, active = [], False
    for raw in lines:
        s = clean(raw)
        if s.startswith("#RECORDS"):
            out, active = [s], True
        elif active:
            out.append(s)
            if s.startswith("#END-RECORDS"):
                return out
    return None


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        text = " ".join(filter(None, [p.description, p.manufacturer, p.product])).lower()
        if "arduino" in text or "mega" in text or "2560" in text or "wch" in text or "ch340" in text:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    names = ", ".join(p.device for p in ports) or "none found"
    sys.exit("Could not pick the Mega's serial port automatically (ports: %s). Give it as the first argument." % names)


def open_port(port, baud, reset):
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    if not reset:
        ser.dtr = False  # best effort: avoids the auto-reset on some systems
    ser.open()
    if reset:
        time.sleep(2.5)  # the Mega restarts when the port opens: wait for the bootloader and the boot report
    return ser


def read_lines(ser, seconds):
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                yield line.decode("ascii", "replace")
        else:
            yield None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="records.txt")
    ap.add_argument("--follow", action="store_true", help="save everything the robot prints, with PC time stamps, until Ctrl-C")
    ap.add_argument("--clear", action="store_true", help="erase the records on the robot")
    ap.add_argument("--no-reset", action="store_true", help="try not to restart the robot when opening the port")
    ap.add_argument("--timeout", type=float, default=25.0, help="seconds to wait for the dump")
    a = ap.parse_args()
    port = a.port or find_port()
    print("using %s at %d baud" % (port, a.baud))
    ser = open_port(port, a.baud, not a.no_reset)
    ser.reset_input_buffer()

    if a.clear:
        ser.write(b":log clear\n")
        time.sleep(0.5)
        print("sent ':log clear'")
        return

    if a.follow:
        print("saving everything to %s (Ctrl-C to stop)" % a.out)
        with open(a.out, "a", encoding="utf-8") as f:
            f.write("# session started %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            try:
                while True:
                    for line in read_lines(ser, 3600):
                        if line is None:
                            continue
                        s = "%s  %s" % (time.strftime("%H:%M:%S"), line.rstrip("\r"))
                        print(s)
                        f.write(s + "\n")
                        f.flush()
            except KeyboardInterrupt:
                print("\nstopped")
        return

    ser.write(b":log\n")
    lines = []
    for line in read_lines(ser, a.timeout):
        if line is not None:
            lines.append(line)
            if "#END-RECORDS" in line:
                break
    recs = extract_records(lines)
    if recs is None:
        sys.exit("No complete record dump received within %.0f s. Is this the robot's port, at the right baud rate, with the Serial Monitor closed?" % a.timeout)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("# Robosoccer26 Cyberbots event records, downloaded %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        f.write("# t_ms counts from the most recent BOOT line above it\n")
        f.write("\n".join(recs) + "\n")
    print("saved %d lines to %s" % (len(recs), a.out))


if __name__ == "__main__":
    main()
