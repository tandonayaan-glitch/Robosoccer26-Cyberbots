# Prompt for Codex: set up and verify the RoboSoccer laptop remote, then stop when I must act

Paste everything below the line into Codex, with the repository checked out.

---

You are working in my RoboCup Junior 2026 project, repository `tandonayaan-glitch/Robosoccer26-Cyberbots`, branch `claude/sweet-knuth-hpyd7i`.

**Architecture (the only one): Windows laptop -> Bluetooth serial COM port -> HC-05 -> Arduino Mega 2560 (`Serial1`).** There is no ESP32. The Mega firmware is in `Robosoccer26_Cyberbots/`, the Python client in `laptop/`, the provisioning tool in `tools/provision.py`, the beginner guide in `Connect.md`, details in `docs/BLUETOOTH.md`.

## Rules (read first)

* Work only on `claude/sweet-knuth-hpyd7i`. Never push or merge to `Robocup-2026-Ayaan` or `main`. Never force-push or reset; if a push is rejected, `git fetch` and merge, and keep other people's commits.
* Never put a real key, passcode or credential into any file in the repository, a commit, a log, a command line you print, or your replies. The robot key lives only in a private key file outside the repository (default `%LOCALAPPDATA%\Robosoccer26\robot.key`, or `~/.robosoccer26/robot.key` on Linux/macOS). Use throw-away keys in temporary directories for any test.
* Do not change motor pins (right L298N: FR D22/D23/ENA D5, BR D24/D25/ENB D6; left L298N: FL D26/D27/ENA D9, BL D28/D29/ENB D10), sensor pins, or the HC-05 wiring (Mega TX1 D18 through a voltage divider to HC-05 RXD, HC-05 TXD to Mega RX1 D19).
* Never claim that anything works on the real robot, the real HC-05, Bluetooth pairing or Windows unless I tell you I ran it. Simulation results are simulation results.
* Do not do anything that makes a motor move. Do not open the Mega's serial port, flash a board, change the HC-05, or touch Bluetooth settings: those are my steps.
* Keep changes small. Do not rewrite unrelated files.

## Part A - do these yourself, in order

1. `git status`, `git branch --show-current`, `git log --oneline -5`. Confirm the branch and that the working tree is clean. If it is not, stop and tell me what is uncommitted.
2. Read `Connect.md`, `docs/BLUETOOTH.md` (Authentication section), and `laptop/laptop_remote.py`, so you know the intended workflow.
3. Check the tools exist and install what is missing **only if it can be done without changing system settings** (use a virtual environment for Python):
   * `python3 --version` (3.9 or newer), `g++ --version`, and for the Mega compile `avr-gcc` (the repo's `tools/avr_build.sh` explains what it needs).
   * `python3 -m venv .venv`, activate it, `python -m pip install -r laptop/requirements.txt`. (`.venv` must not be committed.)
4. Run the checks and record the exact results:
   * `tools/avr_build.sh` (Mega compile; it never uploads). Record flash/RAM use and warnings.
   * `test/run_tests.sh` (C++ firmware simulation, plus the Python client tests against the firmware simulator).
   * `test/static_analysis.sh` if cppcheck/clang-tidy/valgrind are installed; otherwise say it was not run.
   * `python tools/provision.py --selftest`
   * `python laptop/laptop_remote.py --help` and `python laptop/laptop_remote.py --list-ports` (it is fine if no ports are listed).
5. Dry-run the credential workflow on a **temporary key file**, never the real one:
   * `python tools/provision.py --key-file <temp dir>/test.key` creates a key; confirm the file exists, is not world-readable (on Linux/macOS), and that the command did not print the key.
   * Run it again and confirm it refuses to overwrite; run with `--rotate` and confirm a new fingerprint.
   * Delete the temporary directory afterwards.
6. Review the repository for problems and fix only clear, small ones (with a test if behaviour changes):
   * `git grep -n -i esp32` should find only history notes (e.g. the old controller commit `1963a3f`), not setup steps.
   * `git grep -nE '[0-9a-fA-F]{64}'` should find only the synthetic test keys and public test vectors in `test/` and `laptop/tests/` and `tools/provision.py --selftest`.
   * `.gitignore` must exclude `robot.key`, `*.key`, `*.secret`, `.venv`.
7. If you changed anything, run the checks of step 4 again, then commit with a clear message and push to `claude/sweet-knuth-hpyd7i` only.

## Part B - STOP here and hand over to me

When Part A is finished, **stop and write me a short report**, then wait. Do not attempt any of the following yourself; they need me, my hardware or my Windows laptop:

1. Flashing the Mega (Arduino IDE -> board "Arduino Mega or Mega 2560" -> Upload) with wheels lifted and the motor battery disconnected.
2. Pairing Windows with the HC-05 (PIN **2114** is what the docs assume; the HC-05 must actually be set to it) and finding the **Outgoing** Bluetooth COM port.
3. Running `python tools\provision.py --mega-port <the Mega's USB COM port>` with the real Mega attached (this creates the real key; I will keep it private).
4. Running `python laptop\laptop_remote.py --port <Bluetooth COM port>`, typing `connect`, and checking that login works.
5. The physical safety tests in `Connect.md` ("Safe first test"): an unauthenticated Bluetooth terminal gets `ERR:AUTH`, `X` still stops the robot, link loss and reconnect, and only then motors with wheels lifted.
6. Any decision about the HC-05 PIN, name or baud rate.

## Final report format

1. Branch, last commit, and whether the working tree is clean.
2. Every command you ran and its result (pass / fail / not run, with the reason). Include the Mega flash and RAM numbers.
3. Anything you changed (files), or "no changes".
4. Problems you found but did not fix.
5. A numbered checklist of exactly what I must do now (Part B), with the PowerShell commands, using placeholders such as `COM3` and `COM6` for ports and never a real secret.
6. A clear statement of what is still **unverified on hardware**.
