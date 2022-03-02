/**
  Particulate matter sensor firmware for D1 Mini (ESP8266) and PMS5003

  Read from a Plantower PMS5003/PMS7003 particulate matter sensor using
  a Wemos D1 Mini (or other ESP8266-based board) and report the values
  to an HTTP server

  Written by Linka Gonzalez
    https://github.com/garyservin/linka-firmware

  Inspired by https://github.com/superhouse/AirQualitySensorD1Mini

  Copyright 2020 Linka Gonzalez
*/

#include <FS.h>                   // This needs to be first, or it all crashes and burns...

/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <ArduinoJson.h>              // https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>               // Allow local OTA programming
#include <ESP8266HTTPClient.h>        // HTTP Client
#include <ESP8266httpUpdate.h>        // Allow remote OTA programming
#include <ESP8266WiFi.h>              // ESP8266 WiFi driver
#include <LittleFS.h>                 // File System library
#include <SoftwareSerial.h>           // Allows PMS to avoid the USB serial port
#include <time.h>                     // To get current time
#include <WiFiConnect.h>              // Allow configuring WiFi via captive portal
#include "PMS.h"                      // Particulate Matter Sensor driver (embedded)

/*--------------------------- Global Variables ---------------------------*/
// Particulate matter sensor
#define   PMS_STATE_ASLEEP        0   // Low power mode, laser and fan off
#define   PMS_STATE_WAKING_UP     1   // Laser and fan on, not ready yet
#define   PMS_STATE_READY         2   // Warmed up, ready to give data
uint8_t   g_pms_state           = PMS_STATE_WAKING_UP;
uint32_t  g_pms_state_start     = 0;  // Timestamp when PMS state last changed
uint8_t   g_pms_ae_readings_taken  = false;  // true/false: whether any readings have been taken
uint8_t   g_pms_ppd_readings_taken = false;  // true/false: whether PPD readings have been taken

uint16_t  g_pm1p0_sp_value      = 0;  // Standard Particle calibration pm1.0 reading
uint16_t  g_pm2p5_sp_value      = 0;  // Standard Particle calibration pm2.5 reading
uint16_t  g_pm10p0_sp_value     = 0;  // Standard Particle calibration pm10.0 reading

uint16_t  g_pm1p0_ae_value      = 0;  // Atmospheric Environment pm1.0 reading
uint16_t  g_pm2p5_ae_value      = 0;  // Atmospheric Environment pm2.5 reading
uint16_t  g_pm10p0_ae_value     = 0;  // Atmospheric Environment pm10.0 reading

uint32_t  g_pm0p3_ppd_value     = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t  g_pm0p5_ppd_value     = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t  g_pm1p0_ppd_value     = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t  g_pm2p5_ppd_value     = 0;  // Particles Per Deciliter pm2.5 reading
uint32_t  g_pm5p0_ppd_value     = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t  g_pm10p0_ppd_value    = 0;  // Particles Per Deciliter pm10.0 reading

// HTTP Server
#define JSON_BUFFER 256
char http_data_template[] = "[{"
                            "\"sensor\": \"%s\","
                            "\"source\": \"%s\","
                            "\"version\": \"%s\","
                            "\"description\": \"%s\","
                            "\"pm1dot0\": %d,"
                            "\"pm2dot5\": %d,"
                            "\"pm10\": %d,"
                            "\"longitude\": %s,"
                            "\"latitude\": %s,"
                            "\"recorded\": \"%s\""
                            "}]";

uint32_t g_device_id;                    // Unique ID from ESP chip ID

// Time keeping
time_t now;
struct tm * timeinfo;
char recorded_template[]        = "%d-%02d-%02dT%02d:%02d:%02d.000Z";

bool force_configuration_portal = false;
bool force_params_portal        = false;

#define REMOTE_OTA_TIMEOUT      24 * 60 * 60 * 1000 //Check every 24 hours
uint32_t  g_remote_ota_last_run = 0;  // Timestamp when last OTA was run

/*--------------------------- Function Signatures ------------------------*/
void initFS();
void initOta();
void initNtp();
void initWifi();
void handleRemoteOta();
void updatePmsReadings();

/*--------------------------- Instantiate Global Objects -----------------*/
// Software serial port
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN); // Rx pin = GPIO2 (D4 on Wemos D1 Mini)

// Particulate matter sensor
PMS pms(pmsSerial, false);           // Use the software serial port for the PMS
PMS::DATA g_data;

// Start HTTP client
WiFiClientSecure client;
HTTPClient http;

// WifiManager
WiFiConnect wc;

// vars to store parameters
char api_key[33] = "";
char latitude[12] = "";
char longitude[12] = "";
char description[21] = "";
char api_url[71] = "https://rald-dev.greenbeep.com/api/v1/measurements";
char ota_server[71] = "https://linka.servin.dev/ota";

// flag for saving data
bool shouldSaveConfig = false;

/*--------------------------- Program ------------------------------------*/
void configModeCallback(WiFiConnect *mWiFiConnect) {
}

// Callback notifying us of the need to save config
void saveConfigCallback () {
  shouldSaveConfig = true;
}

// Remote OTA callbacks
void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

/*
   Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);   // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)
  delay(100);
  Serial.println();
  Serial.print("Linka Air Quality Sensor v");
  Serial.println(VERSION);

  // Open a connection to the PMS and put it into passive mode
  pmsSerial.begin(PMS_BAUD_RATE);   // Connection for PMS5003
  pms.passiveMode();                // Tell PMS to stop sending data automatically
  delay(100);
  pms.wakeUp();                     // Tell PMS to wake up (turn on fan and laser)

  // Get ESP's unique ID
  g_device_id = ESP.getChipId();  // Get the unique ID of the ESP8266 chip
  Serial.print("Device ID: ");
  Serial.println(g_device_id, HEX);

  // Check if we want to factory reset the sensor
  check_reset();

  // Ignore SSL certificate, required to use SSL without providing the SSL certificate
  client.setInsecure();

  // Initialize File System
  initFS();

  // Initialize WiFi
  initWifi();

  // Initialize OTA
  initOta();

  // Initialize NTP
  initNtp();

  Serial.println("Sensor configured correctly...");
}

/*
  Main loop
*/
void loop()
{
  if (WiFi.status() == WL_CONNECTED) {
    // If we're connected to WiFi, manage OTA
    ArduinoOTA.handle();
    handleRemoteOta();
  }
  else {
    // If we've lost Wifi, start captive portal, but check periodically for WiFi
    // When updating to newer SDK, need to make sure we can store the wifi configuration
    // https://github.com/esp8266/Arduino/pull/7902
    WiFi.persistent(true);
    wc.startConfigurationPortal(AP_RESET);
    WiFi.persistent(false);
  }

  updatePmsReadings();
}

/*
  Update particulate matter sensor values
*/
void updatePmsReadings() {
  uint32_t time_now = millis();

  // Check if we've been in the sleep state for long enough
  if (PMS_STATE_ASLEEP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= ((g_pms_report_period * 1000) - (g_pms_warmup_period * 1000)))
    {
      // It's time to wake up the sensor
      Serial.println("Waking up sensor");
      pms.wakeUp();
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_WAKING_UP;
    }
  }

  // Check if we've been in the waking up state for long enough
  if (PMS_STATE_WAKING_UP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= (g_pms_warmup_period * 1000))
    {
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_READY;
    }
  }

  // Put the most recent values into globals for reference elsewhere
  if (PMS_STATE_READY == g_pms_state)
  {
    //Serial.println("Sensor is Ready");
    //pms.requestRead();
    if (pms.readUntil(g_data))  // Use a blocking road to make sure we get values
    {
      // Get current time of reading

      time(&now);
      timeinfo = localtime(&now);

      g_pm1p0_sp_value   = g_data.PM_SP_UG_1_0;
      g_pm2p5_sp_value   = g_data.PM_SP_UG_2_5;
      g_pm10p0_sp_value  = g_data.PM_SP_UG_10_0;

      g_pm1p0_ae_value   = g_data.PM_AE_UG_1_0;
      g_pm2p5_ae_value   = g_data.PM_AE_UG_2_5;
      g_pm10p0_ae_value  = g_data.PM_AE_UG_10_0;

      g_pms_ae_readings_taken = true;

      // This condition below should NOT be required, but currently I get all
      // 0 values for the PPD results every second time. This check only updates
      // the global values if there is a non-zero result for any of the values:
      if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5
          + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5
          + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0
          != 0)
      {
        g_pm0p3_ppd_value  = g_data.PM_TOTALPARTICLES_0_3;
        g_pm0p5_ppd_value  = g_data.PM_TOTALPARTICLES_0_5;
        g_pm1p0_ppd_value  = g_data.PM_TOTALPARTICLES_1_0;
        g_pm2p5_ppd_value  = g_data.PM_TOTALPARTICLES_2_5;
        g_pm5p0_ppd_value  = g_data.PM_TOTALPARTICLES_5_0;
        g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
        g_pms_ppd_readings_taken = true;
      }
      pms.sleep();

      // Report the new values
      reportToHttp();
      //reportToSerial();

      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_ASLEEP;
    }
  }
}

/*
  Report the latest values to HTTP Server
*/
void reportToHttp()
{
  char measurements[256];
  char recorded[27];
  char source[10];

  sprintf(recorded,
          recorded_template,
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
  sprintf(source, "%x", g_device_id);
  sprintf(measurements,
          http_data_template,
          sensor,
          source,
          VERSION,
          description,
          g_pm1p0_sp_value,
          g_pm2p5_sp_value,
          g_pm10p0_sp_value,
          longitude,
          latitude,
          recorded);
  Serial.println(measurements);

  if (http.begin(client, api_url)) {

    // Add headers
    http.addHeader("x-api-key", api_key);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(measurements);

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been sent and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  else {
    Serial.printf("[HTTP] Unable to connect");
  }
}

/*
  Report the latest values to the serial console
*/
void reportToSerial()
{
  if (true == g_pms_ae_readings_taken)
  {
    /* Report PM1.0 AE value */
    Serial.print("PM1:");
    Serial.print(String(g_pm1p0_ae_value));
    Serial.print(" | SP:");
    Serial.println(String(g_pm1p0_sp_value));

    /* Report PM2.5 AE value */
    Serial.print("PM2.5:");
    Serial.print(String(g_pm2p5_ae_value));
    Serial.print(" | SP:");
    Serial.println(String(g_pm2p5_sp_value));

    /* Report PM10.0 AE value */
    Serial.print("PM10:");
    Serial.print(String(g_pm10p0_ae_value));
    Serial.print(" | SP:");
    Serial.println(String(g_pm10p0_sp_value));
  }

  if (true == g_pms_ppd_readings_taken)
  {
    /* Report PM0.3 PPD value */
    Serial.print("PB0.3:");
    Serial.println(String(g_pm0p3_ppd_value));

    /* Report PM0.5 PPD value */
    Serial.print("PB0.5:");
    Serial.println(String(g_pm0p5_ppd_value));

    /* Report PM1.0 PPD value */
    Serial.print("PB1:");
    Serial.println(String(g_pm1p0_ppd_value));

    /* Report PM2.5 PPD value */
    Serial.print("PB2.5:");
    Serial.println(String(g_pm2p5_ppd_value));

    /* Report PM5.0 PPD value */
    Serial.print("PB5:");
    Serial.println(String(g_pm5p0_ppd_value));

    /* Report PM10.0 PPD value */
    Serial.print("PB10:");
    Serial.println(String(g_pm10p0_ppd_value));
  }
}

/*
  Initialize Local and Remote OTA
*/
void initOta()
{
  Serial.println("Initializing OTA...");

  // Setup OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Initialize remote OTA
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);
}

/*
  Configure Wifi and captive portal
*/
void initWifi()
{
  Serial.println("Initializing WiFi...");
  Serial.print("\tStored SSID: ");
  Serial.println(WiFi.SSID());

  // Disable debug for WiFi connect
  wc.setDebug(false);

  // Set our callbacks
  wc.setAPCallback(configModeCallback);

  // Set config save notify callback
  wc.setSaveConfigCallback(saveConfigCallback);

  // Set how many connection attempts before we fail and go to captive portal mode
  wc.setRetryAttempts(5);

  // How long we wait for the connection attempt to timeout
  wc.setConnectionTimeoutSecs(10);

  // How long to wait in captive portal mode before we try to reconnect
  wc.setAPModeTimeoutMins(1);

  // Set Access Point name for captive portal mode
  char ap_name[13];
  sprintf(ap_name, "linka-%x", g_device_id);
  wc.setAPName(ap_name);

  // Set correct hostname
  WiFi.hostname(ap_name);

  // Configure custom parameters
  WiFiConnectParam api_key_param("api_key", "API Key", api_key, 33);
  WiFiConnectParam latitude_param("latitude", "Latitude", latitude, 13);
  WiFiConnectParam longitude_param("longitude", "Longitude", longitude, 13);
  WiFiConnectParam sensor_param("sensor", "Sensor model", sensor, 8);
  WiFiConnectParam description_param("description", "Description", description, 21);
  WiFiConnectParam api_url_param("api_url", "URL for the backend", api_url, 71);
  WiFiConnectParam ota_server_param("ota_server", "Server for OTA upgrades", ota_server, 71);
  wc.addParameter(&api_key_param);
  wc.addParameter(&latitude_param);
  wc.addParameter(&longitude_param);
  wc.addParameter(&sensor_param);
  wc.addParameter(&description_param);
  wc.addParameter(&api_url_param);
  wc.addParameter(&ota_server_param);

  // Check if we need to start captive portal
  if (!wc.autoConnect()) {
      Serial.println("\tUnable to connect to wifi, starting Configuration portal and checking periodically for wifi");
      // When updating to newer SDK, need to make sure we can store the wifi configuration
      // https://github.com/esp8266/Arduino/pull/7902
      WiFi.persistent(true);
      wc.startConfigurationPortal(AP_RESET); // if not connected show the configuration portal
      WiFi.persistent(false);
  } else {
    if (force_params_portal) {
      Serial.println("\tConfig params not found, start Params Portal");
      wc.startParamsPortal(AP_WAIT); //if not connected show the configuration portal
    }
  }

  Serial.println("\tConnected to WiFi");
  Serial.print("\tSSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("\tIP address: ");
  Serial.println(WiFi.localIP());

  if (shouldSaveConfig) {
    Serial.println("\tSaving configurations to filesystem");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["api_key"] = api_key_param.getValue();
    json["latitude"] = latitude_param.getValue();
    json["longitude"] = longitude_param.getValue();
    json["sensor"] = sensor_param.getValue();
    json["description"] = description_param.getValue();
    json["api_url"] = api_url_param.getValue();
    json["ota_server"] = ota_server_param.getValue();

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("\tFailed to open config file for writing");
    } else {
      json.printTo(configFile);
      configFile.close();
    }

    Serial.print('\t');
    json.printTo(Serial);
    Serial.println();

    // Copy parameters to variables
    strcpy(api_key, json["api_key"]);
    strcpy(latitude, json["latitude"]);
    strcpy(longitude, json["longitude"]);
    strcpy(sensor, json["sensor"]);
    strcpy(description, json["description"]);
    strcpy(api_url, json["api_url"]);
    strcpy(ota_server, json["ota_server"]);
  }
}

/*
   Init filesystem to store parameters
*/
void initFS(void)
{
  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (LittleFS.begin()) {
    Serial.println("\tMounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("\tReading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("\tOpened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {

          strcpy(api_key, json["api_key"]);
          strcpy(latitude, json["latitude"]);
          strcpy(longitude, json["longitude"]);
          strcpy(sensor, json["sensor"]);
          if (json.containsKey("description")) {
            strcpy(description, json["description"]);
          }
          if (json.containsKey("api_url")) {
            strcpy(api_url, json["api_url"]);
          }
          if (json.containsKey("ota_server")) {
            strcpy(ota_server, json["ota_server"]);
          }
          if (strcmp(api_key, "") == 0) {
            Serial.println("\tStored parameters are empty, reset the parameters");
            force_params_portal = true;
          }
          else {
            Serial.println("\tRead the following parameters:");
            Serial.print("\t\tAPI URL: ");
            Serial.println(api_url);
            Serial.print("\t\tAPI-key: ");
            Serial.println(api_key);
            Serial.print("\t\tLatitude: ");
            Serial.println(latitude);
            Serial.print("\t\tLongitude: ");
            Serial.println(longitude);
            Serial.print("\t\tSensor: ");
            Serial.println(sensor);
            Serial.print("\t\tDescription: ");
            Serial.println(description);
            Serial.print("\t\tRemote OTA Server: ");
            Serial.println(ota_server);
          }
        } else {
          Serial.println("\tFailed to load json config");
        }
        configFile.close();
      } else {
        Serial.println("\tFailed to open config file");
      }
    } else {
      Serial.println("\tConfig file wasn't found");
      force_params_portal = true;
    }
  } else {
    Serial.println("\tFailed to mount FS");
  }
}


/*
   Init NTP client to get accurate time
*/
void initNtp()
{
  Serial.println("Initializing NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  time(&now);
  timeinfo = localtime(&now);
  while (timeinfo->tm_year == 70) {
    delay(500);
    time(&now);
    timeinfo = localtime(&now);
  }
}

/*
    Check if we need to check for new version on the remote OTA server
*/
void handleRemoteOta() {
  uint32_t time_now = millis();

  if (time_now - g_remote_ota_last_run > REMOTE_OTA_TIMEOUT || g_remote_ota_last_run == 0) {
    g_remote_ota_last_run = time_now;
    Serial.println("Remote OTA: Checking for new available version");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, ota_server, VERSION);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("Remote OTA: failed, Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("Remote OTA: No updates");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("Remote OTA: Update OK");
        break;
    }
  }
}

/*
    Check if factory reset was requested
*/
void check_reset()
{
  pinMode(ESP_FACTORY_RESET, INPUT_PULLUP);

  if ( digitalRead(ESP_FACTORY_RESET) == LOW) {
    delay(200);  // Wait 200ms and check if button is reset is still attempted
    if ( digitalRead(ESP_FACTORY_RESET) == LOW) {
      Serial.println("Resetting sensor to factory defaults");
      LittleFS.format(); // Format Filesystem
      WiFi.persistent(true);
      WiFi.begin("0", "0"); // Hack to force wifi to be reset
      wc.resetSettings(); // Reset WiFi Settings
    }
  }
}
