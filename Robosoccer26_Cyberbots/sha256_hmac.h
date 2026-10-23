// sha256_hmac.h - SHA-256 (FIPS 180-4) and HMAC-SHA-256 (RFC 2104) in plain C++, small enough for the ATmega2560.
// The laptop client uses Python's standard hmac / hashlib instead (an independent implementation, so the two are cross-checked in laptop/tests/). Verified against the NIST / RFC 4231 test vectors (t_sha256_hmac_vectors). It is NOT a substitute for a
// reviewed crypto library: the Arduino AVR core ships none, so this minimal, standard algorithm is used for the challenge-response only (see docs/BLUETOOTH.md).
// No dynamic memory; the message schedule is a rolling 16-word window (64 bytes of stack) to suit the 8 KB of RAM.
#pragma once
#ifndef RS_SHA256_HMAC_H            // classic include guard
#define RS_SHA256_HMAC_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#if defined(__AVR__)
  #include <avr/pgmspace.h>
  #define SHA_K(i) pgm_read_dword(&SHA256_K[i])
#else
  #ifndef PROGMEM
  #define PROGMEM
  #endif
  #define SHA_K(i) (SHA256_K[i])
#endif

static const uint32_t SHA256_K[64] PROGMEM = {
  0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
  0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
  0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
  0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

struct Sha256 {
  uint32_t h[8];
  uint8_t buf[64];
  uint32_t total;          // bytes absorbed (messages here are tiny; 32 bits are plenty)
  uint8_t n;               // bytes in buf
};

static inline uint32_t sha_rotr(uint32_t x, uint8_t k) { return (x >> k) | (x << (32 - k)); }

static void sha256_block(Sha256* s, const uint8_t* p) {
  uint32_t w[16];
  for (uint8_t i = 0; i < 16; i++) w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | (uint32_t)p[4 * i + 3];
  uint32_t a = s->h[0], b = s->h[1], c = s->h[2], d = s->h[3], e = s->h[4], f = s->h[5], g = s->h[6], h = s->h[7];
  for (uint8_t i = 0; i < 64; i++) {
    if (i >= 16) {
      uint32_t w15 = w[(i + 1) & 15], w2 = w[(i + 14) & 15];
      uint32_t s0 = sha_rotr(w15, 7) ^ sha_rotr(w15, 18) ^ (w15 >> 3);
      uint32_t s1 = sha_rotr(w2, 17) ^ sha_rotr(w2, 19) ^ (w2 >> 10);
      w[i & 15] = w[i & 15] + s0 + w[(i + 9) & 15] + s1;
    }
    uint32_t S1 = sha_rotr(e, 6) ^ sha_rotr(e, 11) ^ sha_rotr(e, 25);
    uint32_t ch = (e & f) ^ (~e & g);
    uint32_t t1 = h + S1 + ch + (uint32_t)SHA_K(i) + w[i & 15];
    uint32_t S0 = sha_rotr(a, 2) ^ sha_rotr(a, 13) ^ sha_rotr(a, 22);
    uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
    uint32_t t2 = S0 + mj;
    h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
  }
  s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}

static void sha256_init(Sha256* s) {
  static const uint32_t iv[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au, 0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  for (uint8_t i = 0; i < 8; i++) s->h[i] = iv[i];
  s->total = 0; s->n = 0;
}
static void sha256_update(Sha256* s, const uint8_t* d, size_t len) {
  s->total += (uint32_t)len;
  while (len) {
    uint8_t take = (uint8_t)(64 - s->n);
    if (take > len) take = (uint8_t)len;
    memcpy(s->buf + s->n, d, take);
    s->n = (uint8_t)(s->n + take); d += take; len -= take;
    if (s->n == 64) { sha256_block(s, s->buf); s->n = 0; }
  }
}
static void sha256_final(Sha256* s, uint8_t out[32]) {
  uint32_t bits = s->total << 3, hi = s->total >> 29;
  uint8_t pad = 0x80;
  sha256_update(s, &pad, 1);
  uint8_t zero = 0;
  while (s->n != 56) sha256_update(s, &zero, 1);
  const uint8_t len[8] = {(uint8_t)(hi >> 24), (uint8_t)(hi >> 16), (uint8_t)(hi >> 8), (uint8_t)hi, (uint8_t)(bits >> 24), (uint8_t)(bits >> 16), (uint8_t)(bits >> 8), (uint8_t)bits};
  sha256_update(s, len, 8);
  for (uint8_t i = 0; i < 8; i++) { out[4 * i] = (uint8_t)(s->h[i] >> 24); out[4 * i + 1] = (uint8_t)(s->h[i] >> 16); out[4 * i + 2] = (uint8_t)(s->h[i] >> 8); out[4 * i + 3] = (uint8_t)s->h[i]; }
  memset(s, 0, sizeof(*s));
}
__attribute__((unused)) static void sha256(const uint8_t* d, size_t len, uint8_t out[32]) { Sha256 s; sha256_init(&s); sha256_update(&s, d, len); sha256_final(&s, out); }

// HMAC-SHA-256 over up to three message parts (so callers need no concatenation buffer). Keys longer than 64 bytes are hashed first (RFC 2104).
static void hmac_sha256(const uint8_t* key, size_t keyLen, const uint8_t* m1, size_t l1, const uint8_t* m2, size_t l2, const uint8_t* m3, size_t l3, uint8_t out[32]) {
  uint8_t k[64];
  memset(k, 0, sizeof(k));
  if (keyLen > 64) sha256(key, keyLen, k); else memcpy(k, key, keyLen);
  uint8_t pad[64];
  Sha256 s;
  for (uint8_t i = 0; i < 64; i++) pad[i] = (uint8_t)(k[i] ^ 0x36);
  sha256_init(&s); sha256_update(&s, pad, 64);
  if (m1 && l1) sha256_update(&s, m1, l1);
  if (m2 && l2) sha256_update(&s, m2, l2);
  if (m3 && l3) sha256_update(&s, m3, l3);
  uint8_t inner[32];
  sha256_final(&s, inner);
  for (uint8_t i = 0; i < 64; i++) pad[i] = (uint8_t)(k[i] ^ 0x5c);
  sha256_init(&s); sha256_update(&s, pad, 64); sha256_update(&s, inner, 32); sha256_final(&s, out);
  memset(k, 0, sizeof(k)); memset(pad, 0, sizeof(pad)); memset(inner, 0, sizeof(inner));
}

// Comparison that does not stop at the first difference (no timing side channel on the tag).
__attribute__((unused)) static bool ct_equal(const uint8_t* a, const uint8_t* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

__attribute__((unused)) static inline int hexVal(char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1)); }
// Exactly 2*n hex digits (and nothing after them) -> n bytes.
__attribute__((unused)) static bool hexToBytes(const char* s, uint8_t* out, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int a = hexVal(s[2 * i]); if (a < 0) return false;
    int b = hexVal(s[2 * i + 1]); if (b < 0) return false;
    out[i] = (uint8_t)(a * 16 + b);
  }
  return s[2 * n] == 0;
}
__attribute__((unused)) static void bytesToHex(const uint8_t* in, size_t n, char* out) {
  static const char hx[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) { out[2 * i] = hx[in[i] >> 4]; out[2 * i + 1] = hx[in[i] & 15]; }
  out[2 * n] = 0;
}

// ---- the challenge-response used between the laptop and the Mega -------------------------------------------------------------------------------
// sn = Mega's challenge (16 bytes), cn = the laptop's challenge (16 bytes). Tags are the first 16 bytes of an HMAC-SHA-256 keyed with the shared 32-byte key.
//   laptop -> Mega proof : HMAC(key, "E2M" | sn | cn)     Mega -> laptop proof : HMAC(key, "M2E" | cn | sn)
// The different labels stop a proof for one direction being reflected back as the other.
static const size_t AUTH_NONCE = 16, AUTH_TAG = 16, AUTH_KEY = 32;
__attribute__((unused)) static void authTag(const uint8_t key[32], const char* label3, const uint8_t* first16, const uint8_t* second16, uint8_t tag[16]) {
  uint8_t full[32];
  hmac_sha256(key, 32, (const uint8_t*)label3, 3, first16, 16, second16, 16, full);
  memcpy(tag, full, 16);
  memset(full, 0, sizeof(full));
}
#endif  // RS_SHA256_HMAC_H
