"""keystore.py - where the laptop keeps the robot's shared key.

The key is 32 random bytes written as 64 hex digits in a small file in the user's profile (Windows: %LOCALAPPDATA%\\Robosoccer26\\robot.key). It is never printed,
logged, put on a command line or sent over Bluetooth. Whoever can read the file can control the robot: keep it private, do not commit or e-mail it.
Moving to another laptop = copy the file there (USB stick) or run   python laptop_remote.py --import-key   on the new laptop and paste the 64 digits (hidden input).
"""
import os
import secrets
import subprocess
import sys

from robot_protocol import KeyFormatError, parse_key


def default_path():
    if os.name == "nt":
        base = os.environ.get("LOCALAPPDATA") or os.path.join(os.path.expanduser("~"), "AppData", "Local")
        return os.path.join(base, "Robosoccer26", "robot.key")
    return os.path.join(os.path.expanduser("~"), ".robosoccer26", "robot.key")


def generate():
    """A new random key from the operating system's secure generator."""
    while True:
        k = secrets.token_bytes(32)
        if len(set(k)) > 1:
            return k


def save(path, key):
    """Write the key (as hex) readable only by the current user as far as the platform allows."""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="ascii") as f:
        f.write(key.hex() + "\n")
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    if os.name == "nt":                                   # best effort: drop inherited permissions and keep only the current user
        try:
            user = os.environ.get("USERNAME")
            if user:
                subprocess.run(["icacls", path, "/inheritance:r", "/grant:r", user + ":F"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        except OSError:
            pass


def load(path):
    """-> 32 bytes. Raises FileNotFoundError or KeyFormatError (the message never contains the file's content)."""
    with open(path, "r", encoding="ascii", errors="replace") as f:
        text = f.read()
    try:
        return parse_key(text)
    except KeyFormatError:
        raise KeyFormatError("%s does not contain a valid 64-digit key" % path)


def exists(path):
    return os.path.isfile(path)
