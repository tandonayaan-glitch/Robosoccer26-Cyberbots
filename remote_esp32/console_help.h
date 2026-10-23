// console_help.h - the help text of the ESP32 USB console (portable; unit-tested so that every command has an entry).
#pragma once
#include <string.h>
#include <stddef.h>

struct HelpEntry { const char* name; const char* alias; const char* usage; const char* text; };

static const HelpEntry CONSOLE_HELP[] = {
  // ---- this ESP32 (never sent to the robot) ----
  {"esp32connect", "auth", "esp32connect [passcode]", "Unlock this console, connect Bluetooth to the robot and authenticate to it. Without the passcode you are asked for it on the next line. The session stays until you type esp32disconnect (or the ESP32 restarts, or the idle lock fires). Wrong passcodes are throttled."},
  {"esp32disconnect", "lock", "esp32disconnect", "End the robot session and lock this console. Nothing typed afterwards is sent (except the emergency stop X). The Bluetooth link itself stays up so that X keeps working; the robot forgets this session at once."},
  {"esp32status", "", "esp32status", "Show the state of THIS remote: locked or unlocked, Bluetooth link, robot session, key fingerprint, current view. Works while locked."},
  {"help", "?help", "help [command]", "List the commands, or explain one command: help ir, help defend, help esp32connect ... Works while locked."},
  {"x", "!", "X   or   !", "EMERGENCY STOP: sent to the robot at once, even while the console is locked. The robot latches it (no movement until R after authenticating)."},
  // ---- views of the robot's data (need esp32connect; the ESP32 asks the robot for fresh frames) ----
  {"ir", "sensors", "ir", "Show the infrared inputs only: the six receivers A0..A5 (level, swing, filtered, noise floor, threshold), the two obstacle modules D34/D35 and the underside sensor D40. 'back' returns to the normal view."},
  {"imu", "gyro", "imu", "Show the MPU6050 only: acceleration, gyro rates, roll/pitch/yaw ESTIMATES, gyro bias and the drift-test result. (To RUN a drift test: gyro drift 30.)"},
  {"motors", "motor", "motors", "Show the motor COMMANDS only: PWM and direction for FL, FR, BL, BR, how long the robot has been moving or turning, watchdog trips. Commands, not measured wheel speed."},
  {"ball", "", "ball", "Show the ball only: seen or not, bearing, confidence, ambiguity, strength."},
  {"safety", "status", "safety", "Show mode, safety state, faults, emergency-stop latch, loop timing and telemetry counters."},
  {"raw", "", "raw", "Show the last raw JSON line of each telemetry kind (c, r, i, d)."},
  {"all", "back", "back   or   all", "Return to the normal full view (everything the robot sends)."},
  // ---- robot commands (forwarded to the Mega after esp32connect) ----
  {"defend", "", "defend", "Defending mode: hold the heading, watch the ball, strafe sideways to get in front of it. Refused if the receivers are not calibrated or their mounting angles were never entered."},
  {"attack", "", "attack", "Autonomous attack mode (search, approach, align, shoot-position logic). Refused when a fault or the emergency-stop latch is active."},
  {"manual", "", "manual", "Manual mode: the keys W S A D Z C drive the robot (each press is a 0.4 s lease; hold the key to keep moving)."},
  {"stop", "idle", "stop", "Stop the motors and leave attack / defend / manual (the emergency-stop latch is NOT cleared)."},
  {"calibrate", "", "calibrate", "Measure the gyro bias and the receiver noise floor. Robot still, motors stopped, ball removed. Refused otherwise."},
  {"tele", "", "tele [on|off|now <c|r|i|d>]", "Telemetry control on the robot. 'tele now r' sends one receivers frame immediately."},
  {"w", "s a d z c", "W S A D Z C", "Manual driving (after 'manual'): forward, backward, turn left, turn right, strafe left, strafe right."},
  {"m", "u q r p", "M U Q R P ?", "M manual, U attack, Q stop, R clear the emergency-stop latch (leaves the robot idle), P ping, ? one-line status."},
};
static const size_t CONSOLE_HELP_N = sizeof(CONSOLE_HELP) / sizeof(CONSOLE_HELP[0]);

static inline bool helpEq(const char* a, const char* b) {
  for (; *a && *b; a++, b++) { char x = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a, y = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b; if (x != y) return false; }
  return *a == 0 && *b == 0;
}
// aliases are space separated
static inline bool helpMatches(const HelpEntry& e, const char* word) {
  if (helpEq(e.name, word)) return true;
  const char* p = e.alias;
  while (*p) {
    char buf[16]; size_t n = 0;
    while (*p && *p != ' ' && n < sizeof(buf) - 1) buf[n++] = *p++;
    buf[n] = 0;
    while (*p == ' ') p++;
    if (n && helpEq(buf, word)) return true;
  }
  return false;
}
static inline const HelpEntry* helpFind(const char* word) {
  for (size_t i = 0; i < CONSOLE_HELP_N; i++) if (helpMatches(CONSOLE_HELP[i], word)) return &CONSOLE_HELP[i];
  return nullptr;
}
// `emit` is called once per output line (the sketch passes Serial.println).
static inline void helpPrint(const char* topic, void (*emit)(const char*)) {
  if (topic && topic[0]) {
    const HelpEntry* e = helpFind(topic);
    if (!e) { emit("no help for that word. 'help' lists every command."); return; }
    emit(e->usage); emit(e->text);
    return;
  }
  emit("---- ESP32 remote console (Serial Monitor 115200 baud, line ending: Newline) ----");
  emit("esp32connect [passcode]   unlock, connect Bluetooth, authenticate to the robot (stays until esp32disconnect)");
  emit("esp32disconnect           end the session and lock          esp32status   state of this remote");
  emit("X or !                    EMERGENCY STOP (works even when locked)");
  emit("-- views (after esp32connect; 'back' returns to the normal view) --");
  emit("ir        receivers A0-A5, obstacle modules, underside sensor    imu     accelerometer / gyro / angles");
  emit("motors    motor command PWM and directions                       ball    bearing and confidence");
  emit("safety    mode, faults, e-stop latch, timing                     raw     last raw telemetry lines       all / back   normal view");
  emit("-- robot commands (forwarded after esp32connect) --");
  emit("defend | attack | manual | stop | calibrate | gyro drift <s> | gyro cal | tele ...    and the keys  M U Q W S A D Z C R P ?");
  emit("'help <command>' explains any of them, e.g.  help ir   help defend   help esp32connect");
}
