#pragma once
/*
  Latitude/longitude cleanup.

  Coordinates are sent to the API as raw JSON numbers, so they must be valid ones.
  normalizeCoordinate() accepts what people type in the setup page ("-25.2846789",
  " -25,28 ", "+057.5") and rewrites it in place as a plain number with at most
  COORDINATE_DECIMALS decimals (extra decimals are cut, not rounded).
*/
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define COORDINATE_DECIMALS   6       // ~0.1 m, more than enough for a sensor location
#define COORDINATE_MAX_LEN    11      // "-180.123456"

// Returns true if `value` is a number within [-limit, limit]. On success `value` is
// rewritten in normalized form; on failure it is left untouched.
static bool normalizeCoordinate(char *value, size_t size, double limit)
{
  const char *p = value;
  char out[COORDINATE_MAX_LEN + 1];
  size_t n = 0;

  while (isspace((unsigned char)*p)) p++;
  if (*p == '-') {
    out[n++] = '-';
    p++;
  } else if (*p == '+') {
    p++;
  }

  // Integer part: at least one digit, without leading zeros (invalid in JSON)
  if (!isdigit((unsigned char)*p)) return false;
  while (*p == '0' && isdigit((unsigned char)p[1])) p++;
  size_t int_digits = 0;
  while (isdigit((unsigned char)*p)) {
    if (++int_digits > 3) return false;       // |value| < 1000
    out[n++] = *p++;
  }

  // Fraction: '.' or ',' (decimal comma), keep at most COORDINATE_DECIMALS digits
  if (*p == '.' || *p == ',') {
    p++;
    size_t decimals = 0;
    size_t dot = n;
    out[n++] = '.';
    while (isdigit((unsigned char)*p)) {
      if (decimals < COORDINATE_DECIMALS) {
        out[n++] = *p;
        decimals++;
      }
      p++;
    }
    if (decimals == 0) n = dot;               // "25." -> "25"
  }

  while (isspace((unsigned char)*p)) p++;
  if (*p != '\0') return false;               // trailing garbage, exponent, etc.
  out[n] = '\0';

  double number = strtod(out, NULL);
  if (number < -limit || number > limit) return false;
  if (n + 1 > size) return false;

  memcpy(value, out, n + 1);
  return true;
}
