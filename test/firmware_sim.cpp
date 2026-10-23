// firmware_sim.cpp - the REAL Mega firmware (host build, mock Arduino core) driven over stdin/stdout so that the Python laptop client can be tested against it.
// One command per line, replies are hex encoded so nothing depends on newlines:
//   REBOOT [enforce|open]   power-cycle (the mock EEPROM survives, so the key and the boot counter persist); enforcement is ON unless 'open'
//   RX <hex>                bytes arriving on Serial1 (from the HC-05)      USB <hex>   bytes typed on the Mega's USB console
//   RUN <ms>                advance simulated time (50 us loop steps)       -> TX <hex> (new Serial1 output), UO <hex> (new USB output), then OK
//   STATE                   -> STATE key=value ...                          BALL <deg> <strength>   place a simulated ball
//   TAG <label> <hexA> <hexB>   the firmware's own HMAC tag with its stored key (cross-check against Python's hmac)
// NOT a model of the radio, the HC-05 or the robot: only of the firmware's logic.
#define HOST_TEST 1
#include "../Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino"
#include <string>
#include <sstream>
#include <iostream>

static size_t g_cur1 = 0, g_curU = 0;
static std::string toHex(const std::string& s) { static const char* h = "0123456789abcdef"; std::string o; for (unsigned char c : s) { o += h[c >> 4]; o += h[c & 15]; } return o; }
static std::string fromHex(const std::string& s) { std::string o; for (size_t i = 0; i + 1 < s.size(); i += 2) o += (char)strtol(s.substr(i, 2).c_str(), nullptr, 16); return o; }
static void ballAt(float deg, float strength) {
  for (int ch = 0; ch < 8; ch++) {
    float d = absf(angDiffDeg((float)g_cfg.ringBearing[ch], deg));
    float f = d < (float)g_cfg.ringFovHalf ? cosf(d / (float)g_cfg.ringFovHalf * 1.5707963f) : 0.0f;
    g_mockRingBase[ch] = 300; g_mockRingAmp[ch] = (uint16_t)(strength * (float)g_cfg.ringNear * f + 0.5f);
  }
}
static void run(uint32_t ms) { uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL; while (g_mockUs < end) { loop(); mockAdvance(1000); } }
static void boot(bool enforce) {
  g_mockAuthEnforced = enforce;
  ballAt(0, 0);
  setup();
  g_autoStartPending = false;
  g_cfg.ringEnable = 0x00FF;
  for (int i = 0; i < 8; i++) g_cfg.ringBearing[i] = (int16_t)(i <= 4 ? i * 45 : i * 45 - 360);
  g_cfg.bearingsSet = 1; g_cfg.ringCalDone = 1;
  g_cur1 = 0; g_curU = 0; Serial1.out.clear(); Serial.out.clear();
}
static void flush() {
  if (Serial1.out.size() > g_cur1) { std::cout << "TX " << toHex(Serial1.out.substr(g_cur1)) << "\n"; g_cur1 = Serial1.out.size(); }
  if (Serial.out.size() > g_curU) { std::cout << "UO " << toHex(Serial.out.substr(g_curU)) << "\n"; g_curU = Serial.out.size(); }
}
int main() {
  std::ios::sync_with_stdio(false);
  boot(true);
  run(300);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string cmd; is >> cmd;
    if (cmd == "REBOOT") { std::string a; is >> a; boot(a != "open"); run(300); flush(); std::cout << "OK\n"; }
    else if (cmd == "RX") { std::string h; is >> h; Serial1.feed(fromHex(h)); std::cout << "OK\n"; }
    else if (cmd == "USB") { std::string h; is >> h; Serial.feed(fromHex(h)); std::cout << "OK\n"; }
    else if (cmd == "RUN") { unsigned ms = 0; is >> ms; run(ms); flush(); std::cout << "OK\n"; }
    else if (cmd == "BALL") { float d = 0, s = 0; is >> d >> s; ballAt(d, s); std::cout << "OK\n"; }
    else if (cmd == "TAG") {
      std::string label, a, b; is >> label >> a >> b;
      std::string A = fromHex(a), B = fromHex(b);
      uint8_t t[16];
      if (label.size() == 3 && A.size() == 16 && B.size() == 16 && g_auth.keySet) { authTag(g_auth.key, label.c_str(), (const uint8_t*)A.data(), (const uint8_t*)B.data(), t); std::cout << "TAGOUT " << toHex(std::string((char*)t, 16)) << "\n"; }
      else std::cout << "TAGOUT none\n";
      std::cout << "OK\n";
    }
    else if (cmd == "STATE") {
      printf("STATE key=%d authed=%d enforced=%d estop=%d mode=%s wheels=%d,%d,%d,%d bootctr=%u fails=%u manual=%d defend=%d auto=%d refused=%u\n", g_auth.keySet, g_auth.authed, g_auth.enabled, g_estop, modeName(),
             (int)g_wheelOut[0], (int)g_wheelOut[1], (int)g_wheelOut[2], (int)g_wheelOut[3], (unsigned)g_auth.bootCtr, (unsigned)g_auth.fails, g_remoteManual, g_defendEnabled, g_autoEnabled, (unsigned)g_auth.refusedCount);
      std::cout.flush();
      fflush(stdout);
      std::cout << "OK\n";
    }
    else std::cout << "ERR unknown command\n";
    std::cout.flush();
  }
  return 0;
}
