// compat.h - what the ATmega2560 (8 KB RAM, 16-bit int, no float printf) needs and the host test build shares.
//
//  * Every string literal that is printed lives in FLASH (PSTR) so the 8 KB of RAM is not eaten by text.
//  * avr-libc's printf has no %f / %g unless a special linker flag is set, which the Arduino IDE cannot do, so the firmware uses its own small
//    formatter (rsFormat) for both the board and the host tests. It supports: flags '-' '0' '+' ' ', width, '.precision', length 'l' / 'h',
//    conversions d i u x X c s S(flash string) f g %.   A double is a 4-byte float on the AVR.
#pragma once
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#if defined(__AVR__)
  #include <avr/pgmspace.h>
  #define RS_HAS_FLASH 1
#else
  #define RS_HAS_FLASH 0
  #define PROGMEM
  #define PGM_P const char*
  #define PSTR(s) (s)
  #define pgm_read_byte(a) (*(const uint8_t*)(a))
  #define pgm_read_word(a) (*(const uint16_t*)(a))
  #define pgm_read_float(a) (*(const float*)(a))
  #define pgm_read_ptr(a) (*(const void* const*)(a))
  #define memcpy_P memcpy
  #define strcmp_P strcmp
  #define strlen_P strlen
  #define strncpy_P strncpy
  #define strcpy_P strcpy
#endif

// Flash-aware string comparison with a literal: STREQ(token, "ring").
#define STREQ(s, lit) (strcmp_P((s), PSTR(lit)) == 0)

#if defined(__AVR__)
extern "C" char* dtostrf(double val, signed char width, unsigned char prec, char* s);
#endif

static inline uint8_t rsPgmByte(const char* p, bool flash) { return flash ? (uint8_t)pgm_read_byte(p) : (uint8_t)*p; }

// Formats a double with `prec` decimals into buf (no padding); returns the length. NaN / inf are spelled out.
static size_t rsFmtFloat(char* buf, size_t cap, double v, int prec, bool shortest) {
  if (cap < 2) return 0;
  if (isnan(v)) { strncpy(buf, "nan", cap - 1); buf[cap - 1] = 0; return strlen(buf); }
  if (isinf(v)) { strncpy(buf, v < 0 ? "-inf" : "inf", cap - 1); buf[cap - 1] = 0; return strlen(buf); }
  if (prec < 0) prec = 6;
  if (prec > 6) prec = 6;
  float a = (float)(v < 0 ? -v : v);
  if (a >= 1.0e9f) { strncpy(buf, v < 0 ? "-big" : "big", cap - 1); buf[cap - 1] = 0; return strlen(buf); }
  // integer part and fraction by hand: identical on the AVR and the host
  bool neg = v < 0;
  uint32_t scale = 1;
  for (int i = 0; i < prec; i++) scale *= 10u;
  double r = (double)(neg ? -v : v) * (double)scale + 0.5;
  uint32_t whole, frac;
  if (r >= 4.0e9) { whole = 4000000000u / scale; frac = 0; }
  else { uint32_t t = (uint32_t)r; whole = t / scale; frac = t % scale; }
  char tmp[24]; int n = 0;
  if (neg && (whole != 0 || frac != 0)) tmp[n++] = '-';
  char dig[12]; int dn = 0;
  if (whole == 0) dig[dn++] = '0';
  while (whole && dn < 11) { dig[dn++] = (char)('0' + whole % 10u); whole /= 10u; }
  while (dn) tmp[n++] = dig[--dn];
  if (prec > 0) {
    char fr[8];
    for (int i = prec - 1; i >= 0; i--) { fr[i] = (char)('0' + frac % 10u); frac /= 10u; }
    int fl = prec;
    if (shortest) while (fl > 0 && fr[fl - 1] == '0') fl--;       // %g: no trailing zeros
    if (fl > 0) { tmp[n++] = '.'; for (int i = 0; i < fl; i++) tmp[n++] = fr[i]; }
  }
  tmp[n] = 0;
  strncpy(buf, tmp, cap - 1); buf[cap - 1] = 0;
  return strlen(buf);
}

// fmtFlash: the format string is in flash (AVR) / plain memory (host).
static int rsFormat(char* out, size_t cap, const char* fmt, bool fmtFlash, va_list ap) {
  size_t o = 0;
  if (cap == 0) return 0;
  #define PUTC(ch) do { if (o + 1 < cap) out[o] = (char)(ch); o++; } while (0)
  for (;;) {
    char ch = (char)rsPgmByte(fmt, fmtFlash); fmt++;
    if (!ch) break;
    if (ch != '%') { PUTC(ch); continue; }
    bool left = false, zero = false, plus = false, space = false;
    for (;;) {
      char f = (char)rsPgmByte(fmt, fmtFlash);
      if (f == '-') left = true; else if (f == '0') zero = true; else if (f == '+') plus = true; else if (f == ' ') space = true; else break;
      fmt++;
    }
    int width = 0, prec = -1;
    char f = (char)rsPgmByte(fmt, fmtFlash);
    if (f == '*') { width = va_arg(ap, int); if (width < 0) { left = true; width = -width; } fmt++; }
    else while (f >= '0' && f <= '9') { width = width * 10 + (f - '0'); fmt++; f = (char)rsPgmByte(fmt, fmtFlash); }
    f = (char)rsPgmByte(fmt, fmtFlash);
    if (f == '.') {
      fmt++; prec = 0; f = (char)rsPgmByte(fmt, fmtFlash);
      if (f == '*') { prec = va_arg(ap, int); fmt++; }
      else while (f >= '0' && f <= '9') { prec = prec * 10 + (f - '0'); fmt++; f = (char)rsPgmByte(fmt, fmtFlash); }
    }
    bool isLong = false;
    for (;;) {
      f = (char)rsPgmByte(fmt, fmtFlash);
      if (f == 'l') { isLong = true; fmt++; } else if (f == 'h' || f == 'z') { fmt++; } else break;
    }
    char conv = (char)rsPgmByte(fmt, fmtFlash);
    if (!conv) break;
    fmt++;
    char body[34]; size_t bl = 0;
    const char* sp = nullptr; bool spFlash = false; size_t sl = 0;
    char sign = 0;
    switch (conv) {
      case '%': PUTC('%'); continue;
      case 'c': body[0] = (char)va_arg(ap, int); bl = 1; zero = false; break;
      case 's': case 'S': {
        sp = va_arg(ap, const char*); spFlash = (conv == 'S');
        if (!sp) { sp = "(null)"; spFlash = false; }
        while ((prec < 0 || (int)sl < prec) && rsPgmByte(sp + sl, spFlash)) sl++;
        zero = false;
        break;
      }
      case 'd': case 'i': {
        long v = isLong ? va_arg(ap, long) : (long)va_arg(ap, int);
        unsigned long u = v < 0 ? (unsigned long)(-(v + 1)) + 1ul : (unsigned long)v;
        if (v < 0) sign = '-'; else if (plus) sign = '+'; else if (space) sign = ' ';
        char t[12]; int n = 0;
        if (u == 0) t[n++] = '0';
        while (u) { t[n++] = (char)('0' + u % 10ul); u /= 10ul; }
        while (n) body[bl++] = t[--n];
        break;
      }
      case 'u': case 'x': case 'X': {
        unsigned long u = isLong ? va_arg(ap, unsigned long) : (unsigned long)va_arg(ap, unsigned int);
        unsigned base = conv == 'u' ? 10u : 16u;
        const char* digs = conv == 'X' ? "0123456789ABCDEF" : "0123456789abcdef";
        char t[12]; int n = 0;
        if (u == 0) t[n++] = '0';
        while (u) { t[n++] = digs[u % base]; u /= base; }
        while (n) body[bl++] = t[--n];
        break;
      }
      case 'f': case 'g': case 'e': {
        double v = va_arg(ap, double);
        if (v < 0) { sign = '-'; v = -v; } else if (plus) sign = '+'; else if (space) sign = ' ';
        if (isnan(v) || isinf(v)) zero = false;
        bl = rsFmtFloat(body, sizeof(body), v, conv == 'g' ? (prec < 0 ? 4 : prec) : prec, conv == 'g');
        break;
      }
      default: PUTC('%'); PUTC(conv); continue;
    }
    size_t len = sp ? sl : bl;
    if (conv == 'd' || conv == 'i' || conv == 'u' || conv == 'x' || conv == 'X') {
      if (prec >= 0) { zero = false; while ((int)len < prec) { /* handled by padding below */ break; } }
    }
    size_t zpad = 0;
    if (!sp && (conv == 'd' || conv == 'i' || conv == 'u' || conv == 'x' || conv == 'X') && prec > (int)len) zpad = (size_t)prec - len;
    size_t total = len + zpad + (sign ? 1u : 0u);
    size_t pad = (width > 0 && (size_t)width > total) ? (size_t)width - total : 0;
    if (!left && !zero) for (size_t i = 0; i < pad; i++) PUTC(' ');
    if (sign) PUTC(sign);
    if (!left && zero) for (size_t i = 0; i < pad; i++) PUTC('0');
    for (size_t i = 0; i < zpad; i++) PUTC('0');
    if (sp) for (size_t i = 0; i < sl; i++) PUTC(rsPgmByte(sp + i, spFlash));
    else for (size_t i = 0; i < bl; i++) PUTC(body[i]);
    if (left) for (size_t i = 0; i < pad; i++) PUTC(' ');
  }
  #undef PUTC
  out[o < cap ? o : cap - 1] = 0;
  return (int)o;
}

// snprintf with a format string in flash (the macro below wraps every literal in PSTR).
static int rsSnprintfP(char* out, size_t cap, const char* fmt, ...) __attribute__((unused));
static int rsSnprintfP(char* out, size_t cap, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  int n = rsFormat(out, cap, fmt, RS_HAS_FLASH != 0, ap);
  va_end(ap);
  return n;
}
#define snprintf(b, n, f, ...) rsSnprintfP((b), (n), PSTR(f), ##__VA_ARGS__)
