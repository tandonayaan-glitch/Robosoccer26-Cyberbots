// RoboCup Junior 2026 - VVS Cyberbots: ESP32 remote and telemetry viewer (Bluetooth Classic SPP master -> HC-05 on the Mega 2560), WITH AUTHENTICATION.
//
// NOT BUILT FOR AN ESP32 here (no toolchain available when this was written). The protocol / lock logic (auth_client.h, sha256_hmac.h, telemetry_parse.h) is unit-tested
// on the host against the real Mega firmware code; this file is only type-checked against host stubs (tools/esp32_syntax_check.sh).
//
// Needs remote_esp32/secrets.h (NOT in Git): run  python3 tools/provision.py  or copy secrets.h.example and fill it in. See docs/BLUETOOTH.md.
// Board: an ORIGINAL ESP32 (Classic Bluetooth). Tools > Partition Scheme > "Huge APP". USB Serial Monitor: 115200 baud, line ending "Newline".
//
// USB console (Serial Monitor, 115200 baud, line ending "Newline"). Type  help  at any time.
//   esp32connect <passcode>   unlock this remote, connect Bluetooth and authenticate to the robot; stays until  esp32disconnect
//   esp32disconnect           end the session and lock
//   X  or  !                  emergency stop (works even while locked)
//   ir | imu | motors | ball | safety | raw   show only that component's data;   back | all   normal full view
//   defend | attack | manual | stop | calibrate | ...   robot commands, and the keys M U Q W S A D Z C R P ?
#include "BluetoothSerial.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "secrets.h"
#include "telemetry_parse.h"
#include "auth_client.h"
#include "console_help.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "This ESP32 build has no Bluetooth Classic (Bluedroid). Select an original ESP32 board and enable Bluetooth in the core configuration. BLE-only chips (S3, C3, ...) cannot talk to an HC-05."
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP is not enabled in this core build."
#endif

static const char* ROBOT_BT_NAME = "VVS-CYBERBOTS";   // the name you gave the HC-05 (AT+NAME). Pairing by name; the name is NOT a secret and NOT the authentication.
static const uint32_t REPEAT_MS = 100;                  // a held direction button repeats; the robot stops 400 ms after the last command (btlease)
static const uint32_t PING_MS = 500;                    // keeps the robot's authenticated session alive (it ends after 2.5 s of silence)
static const uint32_t VIEW_MS = 1000;
static const uint32_t USB_IDLE_MS = 20;                 // a line typed without a newline is completed after this much silence

static const uint8_t PIN_FWD = 32, PIN_BACK = 33, PIN_LEFT = 25, PIN_RIGHT = 26;
static const uint8_t PIN_MANUAL = 27, PIN_AUTO = 14, PIN_STOP = 12, PIN_ESTOP = 13, PIN_RESET = 15, PIN_DEFEND = 4;

BluetoothSerial bt;
static TeleSnapshot snap;
static TeleLineBuf rxBuf;
static ConsoleAuth con;
static ClientAuth cl;
static bool linked = false;
static uint32_t lastSend = 0, lastTry = 0, lastPing = 0, lastView = 0, lastLockedMsg = 0, lastBtnMsg = 0, usbLastMs = 0;
static char usbLine[48];
static uint8_t usbLen = 0;
static bool usbOvf = false;
static ClState lastState = CL_IDLE;
static uint8_t viewMode = V_ALL;
static uint32_t lastViewReq = 0, viewReqCount = 0;
#ifndef CONSOLE_REQUIRE_PASSCODE
#define CONSOLE_REQUIRE_PASSCODE 1
#endif

static void rnd(uint8_t* out, size_t n) { esp_fill_random(out, n); }          // the ESP32's hardware random number generator
static bool pressed(uint8_t p) { return digitalRead(p) == LOW; }
static void sendRaw(const char* s) { if (linked) { bt.print(s); bt.print('\n'); } }
static bool sessionOk() { return linked && clientAuthed(&cl); }
static void sendCmd(char c) { char b[2] = {c, 0}; sendRaw(b); }

static void lockNow(const char* why) {
  if (sessionOk()) sendRaw("AUTH OUT");                // tell the robot to forget this session too
  clientReset(&cl);
  con.unlocked = false;
  viewMode = V_ALL;
  Serial.print("console LOCKED ("); Serial.print(why); Serial.println("). Robot session ended. The Bluetooth link stays up so X still works.");
}
static void printLine(const char* s) { Serial.println(s); }

static void printLocalStatus(uint32_t now) {
  Serial.println("---- this remote ----");
  Serial.print("console: "); Serial.println(con.unlocked ? "UNLOCKED" : (con.provisioned ? "locked" : "locked (NOT PROVISIONED)"));
  Serial.print("bluetooth: "); Serial.println(linked ? "connected to the robot's HC-05" : "not connected");
  Serial.print("robot session: "); Serial.println(clientAuthed(&cl) ? "AUTHENTICATED" : (cl.robotOpen ? "robot does not enforce authentication (NOT trusted)" : (cl.st == CL_IDLE ? "none" : "handshake in progress")));
  Serial.print("shared key: "); Serial.println(cl.keyOk ? "provisioned" : "NOT PROVISIONED");
  Serial.print("view: "); Serial.println(teleViewName(viewMode));
  Serial.print("telemetry frames "); Serial.print((unsigned long)snap.frames); Serial.print(", bad "); Serial.print((unsigned long)snap.bad); Serial.print(", lost "); Serial.println((unsigned long)snap.lost);
  (void)now;
}

static void handleConsoleLine(const char* line, uint32_t now) {
  switch (consoleLine(&con, line, now)) {
    case CA_NONE: break;
    case CA_ESTOP: sendRaw("X"); Serial.println(linked ? "EMERGENCY STOP sent" : "EMERGENCY STOP NOT SENT: no Bluetooth link"); break;
    case CA_PROMPT: Serial.println("passcode? (type it on the next line; it is not forwarded or stored)"); break;
    case CA_UNLOCKED:
      Serial.println("console UNLOCKED. Connecting to the robot and authenticating...  (esp32disconnect ends this; help lists the commands)");
      if (!CONSOLE_REQUIRE_PASSCODE) Serial.println("NOTE: passcode check is OFF in secrets.h: anyone at this USB port can control the robot.");
      lastTry = 0; viewMode = V_ALL; lastView = 0;
      break;
    case CA_DISCONNECT: lockNow("esp32disconnect"); break;
    case CA_BAD_PASSCODE: Serial.print("wrong passcode. Console locked; next attempt allowed in "); Serial.print((con.blockedUntil - now) / 1000 + 1); Serial.println(" s"); break;
    case CA_THROTTLED: Serial.println("too many wrong attempts: wait, then try again"); break;
    case CA_NOT_PROVISIONED: Serial.println("NOT PROVISIONED: secrets.h still has the placeholders (python3 tools/provision.py). The console stays locked."); break;
    case CA_REJECT_LOCKED: Serial.println("LOCKED: nothing was sent. Type  esp32connect <passcode>   (emergency stop: X or !, help: help)"); break;
    case CA_HELP: helpPrint(con.arg, printLine); break;
    case CA_LOCALSTATUS: printLocalStatus(now); break;
    case CA_VIEW:
      viewMode = con.view; lastView = 0; lastViewReq = 0;
      Serial.print("view: "); Serial.print(teleViewName(viewMode)); Serial.println("   ('back' = normal view, 'help' = commands)");
      break;
    case CA_FORWARD:
      if (sessionOk()) sendRaw(line);
      else Serial.println(linked ? "robot session not authenticated yet: command NOT sent" : "no Bluetooth link: command NOT sent");
      break;
  }
}

static void buttonCmd(char c, uint32_t now) {                                   // every button except the emergency stop goes through the lock AND the robot session
  if (consoleButtonAllowed(&con, now) && sessionOk()) { sendCmd(c); return; }
  if (now - lastBtnMsg > 3000) { lastBtnMsg = now; Serial.println("button ignored: console locked or robot not authenticated"); }
}

void setup() {
  Serial.begin(115200);
  const uint8_t pins[] = {PIN_FWD, PIN_BACK, PIN_LEFT, PIN_RIGHT, PIN_MANUAL, PIN_AUTO, PIN_STOP, PIN_ESTOP, PIN_RESET, PIN_DEFEND};
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("Remote Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  consoleInit(&con, CONSOLE_PASSCODE, CONSOLE_IDLE_LOCK_MS, CONSOLE_REQUIRE_PASSCODE != 0);
  clientInit(&cl, SHARED_KEY_HEX);
  teleInit(&snap); teleLineInit(&rxBuf);
  if (cl.keyOk) {                                                              // a short fingerprint to compare with the Mega's `:authkey` (reveals nothing useful)
    uint8_t full[32]; char fp[9];
    hmac_sha256(cl.key, 32, (const uint8_t*)"FPR", 3, nullptr, 0, nullptr, 0, full);
    bytesToHex(full, 4, fp);
    Serial.printf("key fingerprint %s (must equal the Mega's: type  :authkey  on the Mega's USB console)\n", fp);
  } else Serial.println("SHARED KEY NOT PROVISIONED (secrets.h): the robot cannot be controlled from this remote.");
  Serial.println(con.provisioned ? "console LOCKED. Type  esp32connect <passcode>  to connect (help = all commands; emergency stop X or ! always works)." : "console passcode NOT PROVISIONED: the console stays locked.");
  bt.begin("VVS-REMOTE", true);                                                // true = master: this remote starts the connection
}

void loop() {
  uint32_t now = millis();
  if (!linked && now - lastTry > 3000) {
    lastTry = now;
    linked = bt.connect(ROBOT_BT_NAME);
    Serial.println(linked ? "bluetooth connected" : "bluetooth connect failed, retrying");
    if (linked) clientReset(&cl);
  }
  if (linked && !bt.connected(0)) { linked = false; clientReset(&cl); Serial.println("bluetooth link lost"); }
  if (consoleTick(&con, now)) lockNow("idle timeout");

  // emergency stop button: never gated
  if (pressed(PIN_ESTOP)) { sendRaw("X"); delay(20); }

  // buttons (all gated)
  static bool prevM = false, prevA = false, prevS = false, prevR = false, prevD = false;
  bool m = pressed(PIN_MANUAL), a = pressed(PIN_AUTO), s = pressed(PIN_STOP), r = pressed(PIN_RESET), d = pressed(PIN_DEFEND);
  if (m && !prevM) buttonCmd('M', now);
  if (a && !prevA) buttonCmd('U', now);
  if (d && !prevD) { if (consoleButtonAllowed(&con, now) && sessionOk()) sendRaw("defend"); else buttonCmd(0, now); }
  if (s && !prevS) buttonCmd('Q', now);
  if (r && !prevR) buttonCmd('R', now);
  prevM = m; prevA = a; prevS = s; prevR = r; prevD = d;
  if (now - lastSend >= REPEAT_MS) {
    lastSend = now;
    if (pressed(PIN_FWD)) buttonCmd('W', now);
    else if (pressed(PIN_BACK)) buttonCmd('S', now);
    else if (pressed(PIN_LEFT)) buttonCmd('A', now);
    else if (pressed(PIN_RIGHT)) buttonCmd('D', now);
  }

  // USB console: complete a line at newline, or after USB_IDLE_MS of silence
  while (Serial.available()) {
    char ch = (char)Serial.read();
    usbLastMs = now;
    if (ch == '\n' || ch == '\r') { if (usbLen || usbOvf) { usbLine[usbLen] = 0; if (usbOvf) Serial.println("line too long: ignored"); else handleConsoleLine(usbLine, now); usbLen = 0; usbOvf = false; memset(usbLine, 0, sizeof(usbLine)); } }
    else if (usbLen < sizeof(usbLine) - 1) usbLine[usbLen++] = ch;
    else usbOvf = true;
  }
  if (usbLen && now - usbLastMs >= USB_IDLE_MS) { usbLine[usbLen] = 0; handleConsoleLine(usbLine, now); usbLen = 0; usbOvf = false; memset(usbLine, 0, sizeof(usbLine)); }

  // robot -> remote
  char line[TELE_MAX_LINE + 2];
  while (linked && bt.available()) {
    char ch = (char)bt.read();
    if (!teleLineFeed(&rxBuf, ch, line, sizeof(line))) continue;
    if (line[0] == '{') { if (con.unlocked && sessionOk()) teleParseLine(&snap, line, millis()); }
    else if (!clientOnLine(&cl, line, now) && con.unlocked) Serial.println(line);
  }

  // handshake with the robot (only while the console is unlocked and the link is up)
  char out[100];
  size_t n = clientWantLine(&cl, con.unlocked && linked, now, rnd, out, sizeof(out));
  if (n) { out[n] = 0; sendRaw(out); }
  if (cl.st != lastState) {
    if (cl.st == CL_AUTHED) Serial.println("robot session AUTHENTICATED (the robot proved the key too)");
    else if (lastState == CL_AUTHED) Serial.println("robot session ended");
    if (cl.robotOpen) Serial.println("WARNING: the robot is NOT enforcing authentication (:auth off on its USB console). It is not treated as authenticated.");
    lastState = cl.st;
  }
  if (sessionOk() && now - lastPing > PING_MS) { lastPing = now; sendCmd('P'); }

  if (sessionOk() && con.unlocked && now - lastViewReq >= 500) {          // ask the robot for fresh frames of the kinds the current view needs
    lastViewReq = now;
    const char kinds4[4] = {'c', 'r', 'i', 'd'};
    char k = (viewMode == V_RAW) ? kinds4[viewReqCount % 4] : ((viewReqCount & 1) ? teleViewExtraKind(viewMode) : 'c');
    viewReqCount++;
    if (k) { char rq[16]; snprintf(rq, sizeof(rq), "tele now %c", k); sendRaw(rq); }
  }
  if (now - lastView >= VIEW_MS) {
    lastView = now;
    if (!con.unlocked) { if (now - lastLockedMsg > 15000) { lastLockedMsg = now; Serial.println("[console LOCKED - esp32connect <passcode> to connect, help for commands; X or ! = emergency stop]"); } }
    else {
      static char view[1100];
      teleRenderView(&snap, viewMode, now, sessionOk(), view, sizeof(view));
      Serial.print(view);
    }
  }
}
