// RoboCup Junior 2026 VVS Cyberbots - ESP32 handheld remote (Bluetooth Classic master -> HC-05 on the Mega 2560).
// NOT COMPILED OR TESTED: no ESP32 toolchain was available when this was written. Build with the ESP32 Arduino core (BluetoothSerial) and
// a partition scheme with enough flash. It sends the single-character protocol documented in docs/BLUETOOTH.md.
//
// Wiring: five buttons/inputs to GND with INPUT_PULLUP. Change the pins below to what is really built; they are NOT claims about existing hardware.
#include "BluetoothSerial.h"
#include "esp_mac.h"

static const char* ROBOT_BT_NAME = "VVS-CYBERBOTS";   // the name you set on the HC-05 (AT+NAME); connecting by name, no MAC is hard-coded
static const uint32_t REPEAT_MS = 100;                  // a held direction button repeats the command; the robot stops 400 ms after the last one

static const uint8_t PIN_FWD = 32, PIN_BACK = 33, PIN_LEFT = 25, PIN_RIGHT = 26;   // example pins
static const uint8_t PIN_MANUAL = 27, PIN_AUTO = 14, PIN_STOP = 12, PIN_ESTOP = 13, PIN_RESET = 15;

BluetoothSerial bt;
static uint32_t lastSend = 0, lastTry = 0, lastPing = 0;
static bool linked = false;

static bool pressed(uint8_t p) { return digitalRead(p) == LOW; }
static void sendCmd(char c) { if (linked) bt.write((uint8_t)c); }

void setup() {
  Serial.begin(115200);
  const uint8_t pins[] = {PIN_FWD, PIN_BACK, PIN_LEFT, PIN_RIGHT, PIN_MANUAL, PIN_AUTO, PIN_STOP, PIN_ESTOP, PIN_RESET};
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("Remote Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  bt.begin("VVS-REMOTE", true);                          // true = master: this remote starts the connection
}

void loop() {
  uint32_t now = millis();
  if (!linked && now - lastTry > 3000) {                 // (re)connect by name, slowly, never blocking the buttons for long
    lastTry = now;
    linked = bt.connect(ROBOT_BT_NAME);
    Serial.println(linked ? "connected" : "connect failed, retrying");
  }
  if (linked && !bt.connected(0)) { linked = false; Serial.println("link lost"); }

  // Emergency stop first, every pass, and repeated while held.
  if (pressed(PIN_ESTOP)) { sendCmd('X'); delay(20); }
  static bool prevM = false, prevA = false, prevS = false, prevR = false;
  bool m = pressed(PIN_MANUAL), a = pressed(PIN_AUTO), s = pressed(PIN_STOP), r = pressed(PIN_RESET);
  if (m && !prevM) sendCmd('M');
  if (a && !prevA) sendCmd('U');
  if (s && !prevS) sendCmd('Q');
  if (r && !prevR) sendCmd('R');
  prevM = m; prevA = a; prevS = s; prevR = r;

  if (now - lastSend >= REPEAT_MS) {
    lastSend = now;
    if (pressed(PIN_FWD)) sendCmd('W');
    else if (pressed(PIN_BACK)) sendCmd('S');
    else if (pressed(PIN_LEFT)) sendCmd('A');
    else if (pressed(PIN_RIGHT)) sendCmd('D');
  }
  if (now - lastPing > 1000) { lastPing = now; sendCmd('P'); }   // keeps the robot's link supervision (btlink) alive while idle

  while (linked && bt.available()) Serial.write(bt.read());      // robot replies (OK:/ERR:/PONG:) to the USB console
}
