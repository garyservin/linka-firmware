#pragma once
/*
  Settings sent over the USB serial port, e.g. by the web flasher right after
  installing the firmware, so the sensor doesn't need its Wi-Fi setup portal.

  Protocol, one line per message at SERIAL_BAUD_RATE:

    sensor -> LINKA_READY {"version":"0.4.0","device":"linka-1a2b3c"}
              Printed at boot. The sensor then listens for SERIAL_CONFIG_WINDOW_MS.
    page   -> LINKA_CONFIG {"wifi_ssid":"...","wifi_password":"...","api_key":"...",
                            "latitude":"-25.28","longitude":"-57.63","sensor":"PMS7003",
                            "description":"...","api_url":"...","ota_server":"...",
                            "ota_password":"..."}
              Every field is optional: only the ones sent are changed.
    sensor -> LINKA_CONFIG_OK
              or LINKA_CONFIG_ERROR <reason> (nothing is changed)
    sensor -> LINKA_WIFI_CONNECTED <ip>
              or LINKA_WIFI_FAILED (the setup portal opens, as without settings)

  parseSerialConfig() only parses and validates, so it can be tested on a computer
  (tools/test_serial_config.cpp).
*/
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

#include "coordinates.h"

#define SERIAL_CONFIG_READY       "LINKA_READY"
#define SERIAL_CONFIG_PREFIX      "LINKA_CONFIG "
#define SERIAL_CONFIG_MAX_LINE    768     // Longest LINKA_CONFIG line accepted
#define SERIAL_CONFIG_WINDOW_MS   2000    // How long to listen for settings at boot

struct UsbSettings {
  // Sizes match the sensor's settings, plus the terminating '\0'
  char wifi_ssid[33];       // 802.11 limit: 32 bytes
  char wifi_password[65];   // WPA limit: 64 characters
  char api_key[33];
  char latitude[32];        // Room for what people type: normalized to <= 11 chars
  char longitude[32];
  char sensor[8];
  char description[21];
  char api_url[71];
  char ota_server[71];
  char ota_password[33];    // For uploads over the local network, "" removes it

  bool has_wifi_ssid, has_wifi_password, has_api_key, has_latitude, has_longitude,
       has_sensor, has_description, has_api_url, has_ota_server, has_ota_password;
};

// Copies json[key] into dest if present. Fails if it isn't a string, is too long,
// or is empty while `required`.
static bool serialConfigString(JsonObject &json, const char *key, char *dest,
                               size_t size, bool required, bool &present,
                               char *error, size_t error_size)
{
  present = false;
  if (!json.containsKey(key)) return true;

  // ArduinoJson 5 would also turn numbers into strings: only accept JSON strings
  const char *value = json[key].is<const char *>() ? json[key].as<const char *>() : NULL;
  if (value == NULL) {
    snprintf(error, error_size, "%s must be a string", key);
    return false;
  }
  size_t len = strlen(value);
  if (len >= size) {
    snprintf(error, error_size, "%s is too long (at most %u bytes)", key,
             (unsigned)(size - 1));
    return false;
  }
  if (required && len == 0) {
    snprintf(error, error_size, "%s is empty", key);
    return false;
  }
  memcpy(dest, value, len + 1);
  present = true;
  return true;
}

// Parses the JSON of a LINKA_CONFIG line into `config`. On failure returns false
// and describes the problem in `error`.
static bool parseSerialConfig(char *json_text, UsbSettings &config,
                              char *error, size_t error_size)
{
  static const char *const known_keys[] = {
    "wifi_ssid", "wifi_password", "api_key", "latitude", "longitude", "sensor",
    "description", "api_url", "ota_server", "ota_password",
  };

  memset(&config, 0, sizeof(config));
  error[0] = '\0';

  DynamicJsonBuffer buffer;
  JsonObject &json = buffer.parseObject(json_text);
  if (!json.success()) {
    snprintf(error, error_size, "invalid JSON");
    return false;
  }

  // Reject unknown keys, so a typo doesn't silently leave a setting unchanged
  for (JsonPair &pair : json) {
    bool known = false;
    for (const char *key : known_keys) {
      if (strcmp(pair.key, key) == 0) known = true;
    }
    if (!known) {
      snprintf(error, error_size, "unknown setting %s", pair.key);
      return false;
    }
  }

  if (!serialConfigString(json, "wifi_ssid", config.wifi_ssid, sizeof(config.wifi_ssid), true, config.has_wifi_ssid, error, error_size) ||
      !serialConfigString(json, "wifi_password", config.wifi_password, sizeof(config.wifi_password), false, config.has_wifi_password, error, error_size) ||
      !serialConfigString(json, "api_key", config.api_key, sizeof(config.api_key), true, config.has_api_key, error, error_size) ||
      !serialConfigString(json, "latitude", config.latitude, sizeof(config.latitude), true, config.has_latitude, error, error_size) ||
      !serialConfigString(json, "longitude", config.longitude, sizeof(config.longitude), true, config.has_longitude, error, error_size) ||
      !serialConfigString(json, "sensor", config.sensor, sizeof(config.sensor), true, config.has_sensor, error, error_size) ||
      !serialConfigString(json, "description", config.description, sizeof(config.description), false, config.has_description, error, error_size) ||
      !serialConfigString(json, "api_url", config.api_url, sizeof(config.api_url), true, config.has_api_url, error, error_size) ||
      !serialConfigString(json, "ota_server", config.ota_server, sizeof(config.ota_server), true, config.has_ota_server, error, error_size) ||
      !serialConfigString(json, "ota_password", config.ota_password, sizeof(config.ota_password), false, config.has_ota_password, error, error_size)) {
    return false;
  }

  if (config.has_wifi_password && !config.has_wifi_ssid) {
    snprintf(error, error_size, "wifi_password needs wifi_ssid");
    return false;
  }
  if (config.has_latitude &&
      !normalizeCoordinate(config.latitude, sizeof(config.latitude), 90)) {
    snprintf(error, error_size, "invalid latitude");
    return false;
  }
  if (config.has_longitude &&
      !normalizeCoordinate(config.longitude, sizeof(config.longitude), 180)) {
    snprintf(error, error_size, "invalid longitude");
    return false;
  }
  if (json.size() == 0) {
    snprintf(error, error_size, "no settings");
    return false;
  }
  return true;
}
