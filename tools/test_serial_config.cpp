// Tests for serial_config.h (settings sent over USB), run on a computer:
//   tools/test_serial_config.sh
#include <stdio.h>
#include <string.h>

#include "../serial_config.h"

static int failures = 0;

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
      failures++;                                                         \
    }                                                                     \
  } while (0)

// Parses `json` (copied, since parsing modifies it)
static bool parse(const char *json, UsbSettings &settings, char *error) {
  static char text[SERIAL_CONFIG_MAX_LINE + 1];
  strncpy(text, json, sizeof(text) - 1);
  return parseSerialConfig(text, settings, error, 64);
}

static void expectError(const char *json, const char *expected) {
  UsbSettings settings;
  char error[64];
  bool ok = parse(json, settings, error);
  if (ok || strstr(error, expected) == NULL) {
    printf("FAIL %s -> ok=%d error='%s', expected error containing '%s'\n", json,
           ok, error, expected);
    failures++;
  }
}

static void testAllSettings() {
  UsbSettings s;
  char error[64];
  CHECK(parse("{\"wifi_ssid\":\"Casa Gómez\",\"wifi_password\":\"secret pass\","
              "\"api_key\":\"0123456789abcdef0123456789abcdef\","
              "\"latitude\":\" -25,2846789 \",\"longitude\":\"-57.6359\","
              "\"sensor\":\"PMS5003\",\"description\":\"Balcony\","
              "\"api_url\":\"https://api.example.com/m\","
              "\"ota_server\":\"https://linka.servin.dev/ota/devel\"}",
              s, error));
  CHECK(strcmp(s.wifi_ssid, "Casa Gómez") == 0);
  CHECK(strcmp(s.wifi_password, "secret pass") == 0);
  CHECK(strcmp(s.api_key, "0123456789abcdef0123456789abcdef") == 0);
  CHECK(strcmp(s.latitude, "-25.284678") == 0);   // normalized, 6 decimals
  CHECK(strcmp(s.longitude, "-57.6359") == 0);
  CHECK(strcmp(s.sensor, "PMS5003") == 0);
  CHECK(strcmp(s.description, "Balcony") == 0);
  CHECK(strcmp(s.api_url, "https://api.example.com/m") == 0);
  CHECK(strcmp(s.ota_server, "https://linka.servin.dev/ota/devel") == 0);
  CHECK(s.has_wifi_ssid && s.has_wifi_password && s.has_api_key && s.has_latitude &&
        s.has_longitude && s.has_sensor && s.has_description && s.has_api_url &&
        s.has_ota_server);
}

static void testOnlySomeSettings() {
  UsbSettings s;
  char error[64];
  CHECK(parse("{\"ota_server\":\"https://linka.servin.dev/ota\"}", s, error));
  CHECK(s.has_ota_server);
  CHECK(!s.has_wifi_ssid && !s.has_api_key && !s.has_latitude && !s.has_description);

  // Open network: no password
  CHECK(parse("{\"wifi_ssid\":\"Open\"}", s, error));
  CHECK(s.has_wifi_ssid && !s.has_wifi_password);

  // Description may be cleared
  CHECK(parse("{\"description\":\"\"}", s, error));
  CHECK(s.has_description && s.description[0] == '\0');
}

static void testLimits() {
  UsbSettings s;
  char error[64];
  // Exactly at the limits
  CHECK(parse("{\"wifi_ssid\":\"12345678901234567890123456789012\","
              "\"wifi_password\":\"1234567890123456789012345678901234567890123456789012345678901234\","
              "\"description\":\"12345678901234567890\"}",
              s, error));
  CHECK(strlen(s.wifi_ssid) == 32 && strlen(s.wifi_password) == 64 &&
        strlen(s.description) == 20);
  // One over
  expectError("{\"wifi_ssid\":\"123456789012345678901234567890123\"}", "wifi_ssid is too long");
  expectError("{\"description\":\"123456789012345678901\"}", "description is too long");
  expectError("{\"api_key\":\"0123456789abcdef0123456789abcdef0\"}", "api_key is too long");
  expectError("{\"sensor\":\"PMS70031\"}", "sensor is too long");
}

static void testInvalid() {
  expectError("not json", "invalid JSON");
  expectError("[1,2]", "invalid JSON");
  expectError("{}", "no settings");
  expectError("{\"wifi_pasword\":\"typo\"}", "unknown setting wifi_pasword");
  expectError("{\"wifi_password\":\"x\"}", "wifi_password needs wifi_ssid");
  expectError("{\"wifi_ssid\":\"\"}", "wifi_ssid is empty");
  expectError("{\"api_key\":\"\"}", "api_key is empty");
  expectError("{\"latitude\":-25.28}", "latitude must be a string");
  expectError("{\"latitude\":\"91\"}", "invalid latitude");
  expectError("{\"longitude\":\"abc\"}", "invalid longitude");
  expectError("{\"api_key\":null}", "api_key must be a string");
}

static void testFailureLeavesNothingHalfParsed() {
  UsbSettings s;
  char error[64];
  CHECK(!parse("{\"api_key\":\"abc\",\"latitude\":\"200\"}", s, error));
  // The caller only applies settings on success; parsing reports the first problem
  CHECK(strcmp(error, "invalid latitude") == 0);
}

int main() {
  testAllSettings();
  testOnlySomeSettings();
  testLimits();
  testInvalid();
  testFailureLeavesNothingHalfParsed();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("All serial_config tests passed\n");
  return 0;
}
