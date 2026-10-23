// RoboCup Junior 2026 VVS Cyberbots - ESP32 remote and telemetry viewer (Bluetooth Classic SPP master -> HC-05 on the Mega 2560).
//
// NOT COMPILED OR TESTED ON AN ESP32: no ESP32 toolchain was available when this was written. The protocol parser (telemetry_parse.h) IS unit-tested on the host.
// Build with the Espressif "esp32" Arduino core, board "ESP32 Dev Module" (or any ORIGINAL ESP32: Bluetooth Classic exists only on the original ESP32 / ESP32-WROOM,
// not on the ESP32-S2, -S3, -C3, -C6 or -H2, which have BLE only or none). Menu: Tools > Partition Scheme > "Huge APP" (the Classic-BT stack is large).
//
// Wire protocol: line oriented, "\n" terminated, 9600 baud HC-05 side (docs/BLUETOOTH.md). Single-character commands and words go to the robot; JSON telemetry comes back.
//
// You can type any robot command into this ESP32's USB Serial Monitor (115200 baud, "Newline" or "Both NL & CR"): defend, attack, manual, stop, status, sensors, gyro,
// calibrate, motors, help, or the single keys M U Q W S A D Z C X R P ?.  Buttons (optional, active-low to GND, example pins - change to what you build):
#include "BluetoothSerial.h"
#include "esp_mac.h"
#include "telemetry_parse.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "This ESP32 build has no Bluetooth Classic (Bluedroid). Select an original ESP32 board and enable Bluetooth in the core configuration. BLE-only chips (S3, C3, ...) cannot talk to an HC-05."
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP is not enabled in this core build (Tools > Partition scheme / core config)."
#endif

static const char* ROBOT_BT_NAME = "VVS-CYBERBOTS";   // the name you give the HC-05 (AT+NAME). Connecting by name: no MAC address is hard-coded anywhere.
static const uint32_t REPEAT_MS = 100;                  // a held direction button repeats its command; the robot stops 400 ms after the last one (btlease)
static const uint32_t PING_MS = 1000;                   // keeps the robot's link supervision (btlink) satisfied while idle
static const uint32_t VIEW_MS = 1000;                   // dashboard period on the USB console

static const uint8_t PIN_FWD = 32, PIN_BACK = 33, PIN_LEFT = 25, PIN_RIGHT = 26;
static const uint8_t PIN_MANUAL = 27, PIN_AUTO = 14, PIN_STOP = 12, PIN_ESTOP = 13, PIN_RESET = 15, PIN_DEFEND = 4;

BluetoothSerial bt;
static TeleSnapshot snap;
static TeleLineBuf rxBuf;
static bool linked = false;
static uint32_t lastSend = 0, lastTry = 0, lastPing = 0, lastView = 0;
static char usbLine[40];
static uint8_t usbLen = 0;

static bool pressed(uint8_t p) { return digitalRead(p) == LOW; }
static void sendLine(const char* s) { if (linked) { bt.print(s); bt.print('\n'); } else Serial.println("(not connected: command not sent)"); }
static void sendCmd(char c) { char b[2] = {c, 0}; sendLine(b); }

void setup() {
  Serial.begin(115200);
  const uint8_t pins[] = {PIN_FWD, PIN_BACK, PIN_LEFT, PIN_RIGHT, PIN_MANUAL, PIN_AUTO, PIN_STOP, PIN_ESTOP, PIN_RESET, PIN_DEFEND};
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("Remote Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  teleInit(&snap); teleLineInit(&rxBuf);
  bt.begin("VVS-REMOTE", true);                          // true = master: this remote starts the connection
}

void loop() {
  uint32_t now = millis();
  if (!linked && now - lastTry > 3000) {                 // (re)connect by name, slowly
    lastTry = now;
    linked = bt.connect(ROBOT_BT_NAME);
    Serial.println(linked ? "connected" : "connect failed, retrying");
  }
  if (linked && !bt.connected(0)) { linked = false; Serial.println("link lost"); }

  // emergency stop first, every pass
  if (pressed(PIN_ESTOP)) { sendCmd('X'); delay(20); }
  static bool prevM = false, prevA = false, prevS = false, prevR = false, prevD = false;
  bool m = pressed(PIN_MANUAL), a = pressed(PIN_AUTO), s = pressed(PIN_STOP), r = pressed(PIN_RESET), d = pressed(PIN_DEFEND);
  if (m && !prevM) sendCmd('M');
  if (a && !prevA) sendCmd('U');
  if (d && !prevD) sendLine("defend");
  if (s && !prevS) sendCmd('Q');
  if (r && !prevR) sendCmd('R');
  prevM = m; prevA = a; prevS = s; prevR = r; prevD = d;

  if (now - lastSend >= REPEAT_MS) {
    lastSend = now;
    if (pressed(PIN_FWD)) sendCmd('W');
    else if (pressed(PIN_BACK)) sendCmd('S');
    else if (pressed(PIN_LEFT)) sendCmd('A');
    else if (pressed(PIN_RIGHT)) sendCmd('D');
  }
  if (now - lastPing > PING_MS) { lastPing = now; if (linked) sendCmd('P'); }

  // typed commands from the USB console (a line ends at newline); 'X' or '!' sends the emergency stop at once
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '!' || ch == 'x' || ch == 'X') { if (usbLen == 0 || ch == '!') { sendCmd('X'); usbLen = 0; continue; } }
    if (ch == '\n' || ch == '\r') { if (usbLen) { usbLine[usbLen] = 0; sendLine(usbLine); usbLen = 0; } }
    else if (usbLen < sizeof(usbLine) - 1) usbLine[usbLen++] = ch;
  }

  // robot -> remote: telemetry frames ('{...}'), replies (OK:/ERR:/PONG:/ST:)
  char line[TELE_MAX_LINE + 2];
  while (linked && bt.available()) {
    char ch = (char)bt.read();
    if (teleLineFeed(&rxBuf, ch, line, sizeof(line))) {
      if (line[0] == '{') teleParseLine(&snap, line, millis());
      else Serial.println(line);                         // OK:/ERR:/PONG: replies
    }
  }

  if (now - lastView >= VIEW_MS) {
    lastView = now;
    static char view[900];
    teleRender(&snap, now, linked, view, sizeof(view));
    Serial.print(view);
  }
}
