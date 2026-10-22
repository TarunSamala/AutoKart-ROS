#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

int frame_parse(const char* line, char* out, size_t cap) {
  if (!line || line[0] != '$') return -1;
  const char* star = strrchr(line, '*');
  if (!star) return -1;
  size_t plen = (size_t)(star - (line + 1));
  if (plen == 0 || plen + 1 > cap) return -1;
  // Exactly 4 hex digits after '*', optionally followed by CR/LF.
  const char* h = star + 1;
  uint16_t rx = 0;
  for (int i = 0; i < 4; i++) {
    int v = hexval(h[i]);
    if (v < 0) return -1;
    rx = (uint16_t)((rx << 4) | v);
  }
  if (h[4] != '\0' && h[4] != '\r' && h[4] != '\n') return -1;
  if (crc16_ccitt((const uint8_t*)(line + 1), plen) != rx) return -1;
  memcpy(out, line + 1, plen);
  out[plen] = '\0';
  return (int)plen;
}

int split_fields(char* payload, char** fields, int maxn) {
  int n = 0;
  char* p = payload;
  while (n < maxn) {
    fields[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = '\0';
    p = c + 1;
  }
  return n;
}

int frame_build(const char* payload, char* out, size_t cap) {
  size_t plen = strlen(payload);
  uint16_t crc = crc16_ccitt((const uint8_t*)payload, plen);
  int n = snprintf(out, cap, "$%s*%04X\n", payload, crc);
  return (n < 0 || (size_t)n >= cap) ? -1 : n;
}

bool parse_i32(const char* s, long* out) {
  if (!s || !*s) return false;
  const char* p = s;
  if (*p == '-') p++;
  if (!*p) return false;
  for (const char* q = p; *q; q++) if (*q < '0' || *q > '9') return false;
  *out = strtol(s, NULL, 10);
  return true;
}
