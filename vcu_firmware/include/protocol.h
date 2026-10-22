// Framing helpers. Pure C++ (no Arduino) so they can be unit-tested on the host.
#pragma once
#include <stddef.h>
#include <stdint.h>

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// Validates "$payload*XXXX" (no trailing newline) and copies payload into out (NUL-terminated).
// Returns payload length, or -1 on any framing/CRC error.
int frame_parse(const char* line, char* out, size_t cap);

// Splits a mutable payload on ',' in place. Returns the number of fields (<= maxn).
int split_fields(char* payload, char** fields, int maxn);

// Builds "$payload*XXXX\n" into out. Returns total length, or -1 if it does not fit.
int frame_build(const char* payload, char* out, size_t cap);

// Strict integer parsing (optional leading '-', digits only). Returns false on junk.
bool parse_i32(const char* s, long* out);
