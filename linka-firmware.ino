/**
  Particulate matter sensor firmware for D1 Mini (ESP8266) and PMS5003

  Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1
  Mini (or other ESP8266-based board) and report the values to an HTTP
  server

  Written by Linka Gonzalez
    https://github.com/garyservin/linka-firmware

  Inspired by https://github.com/superhouse/AirQualitySensorD1Mini

  Copyright 2020 Linka Gonzalez
*/
#define VERSION "0.2"
#include <FS.h>                   // This needs to be first, or it all crashes and burns...
/*--------------------------- Configuration ------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries ----------------------------------*/
#include <Wire.h>                     // For I2C
#include <SoftwareSerial.h>           // Allows PMS to avoid the USB serial port
#include <WiFiConnect.h>              // Allow configuring WiFi via captive portal
#include <ESP8266WiFi.h>              // ESP8266 WiFi driver
#include <ESP8266HTTPClient.h>        // HTTP Client
#include <ArduinoOTA.h>               // Allow local OTA programming
#include <time.h>                     // To get current time
#include "PMS.h"                      // Particulate Matter Sensor driver (embedded)
#include <ArduinoJson.h>              // https://github.com/bblanchon/ArduinoJson

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
                            "\"description\": \"%s\","
                            "\"pm1dot0\": %d,"
                            "\"pm2dot5\": %d,"
                            "\"pm10\": %d,"
                            "\"longitude\": %s,"
                            "\"latitude\": %s,"
                            "\"recorded\": \"%s\""
                            "}]";

// General
uint32_t g_device_id;                    // Unique ID from ESP chip ID

int timezone = 0;
int dst = 0;
time_t now;
struct tm * timeinfo;

char recorded_template[] = "%d-%02d-%02dT%02d:%02d:%02d.000Z";
bool force_captive_portal = false;

/*--------------------------- Function Signatures ------------------------*/
void initOta();
bool initWifi();
void initFS();
void updatePmsReadings();

/*--------------------------- Instantiate Global Objects -----------------*/
// Software serial port
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN); // Rx pin = GPIO2 (D4 on Wemos D1 Mini)

// Particulate matter sensor
PMS pms(pmsSerial, false);           // Use the software serial port for the PMS
PMS::DATA g_data;

// Start HTTP client
WiFiClient client;
HTTPClient http;

// WifiManager
WiFiConnect wc;
WiFiConnectParam api_key_param("api_key", "API Key", "", 33);
WiFiConnectParam latitude_param("latitude", "Latitude", "", 13);
WiFiConnectParam longitude_param("longitude", "Longitude", "", 13);
WiFiConnectParam sensor_param("sensor", "Sensor model", "PMS7003", 8);
WiFiConnectParam description_param("description", "Desciption", "", 21);
WiFiConnectParam api_url_param("api_url", "URL for the backend", "http://rald-dev.greenbeep.com/api/v1/measurements", 71);

// vars to store parameters
char api_key[33] = "";
char latitude[12] = "";
char longitude[12] = "";
char description[21] = "";
char api_url[71] = "";

// flag for saving data
bool shouldSaveConfig = false;

/*--------------------------- Program ------------------------------------*/
void configModeCallback(WiFiConnect *mWiFiConnect) {
  Serial.println("Entering Access Point");
}

/*
    Callback notifying us of the need to save config
*/
void saveConfigCallback () {
  shouldSaveConfig = true;
}

/*
   Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);   // GPIO1, GPIO3 (TX/RX pin on ESP-12E Development Board)
  Serial.println();
  Serial.print("Air Quality Sensor starting up, v");
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

  initFS();

  // Wait before we attempt to connect to Wifi
  // In the case of a power outage, wifi might take a while to be ready, we don't want
  // the sensor to go to captive portal mode too quickly.
  // TODO(gservin): Find a way to only wait if there's already a WiFi config stored
  Serial.println("Waiting to let wifi start...");
  delay(60 * 1000); // wait roughly 60 seconds
  initWifi();

  Serial.println("\nUsing the following parameters:");
  Serial.print("API URL: ");
  Serial.println(api_url);
  Serial.print("API-key: ");
  Serial.println(api_key);
  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);
  Serial.print("Sensor: ");
  Serial.println(sensor);
  Serial.print("Description: ");
  Serial.println(description);

  delay(100);

  // Initialize OTA
  initOta();

  // Configure ntp client
  configTime(timezone * 3600, dst * 3600, "pool.ntp.org", "time.nist.gov");

  time(&now);
  timeinfo = localtime(&now);
  while (timeinfo->tm_year == 70) {
    delay(500);
    time(&now);
    timeinfo = localtime(&now);
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

/*
  Main loop
*/
void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    ArduinoOTA.handle();
  }
  else
  {
    // Wifi Dies? Start Portal Again
    if (WiFi.status() != WL_CONNECTED) {
      if (!wc.autoConnect()) wc.startConfigurationPortal(AP_WAIT);

    }
  }

  updatePmsReadings();
}

/*
  Update particulate matter sensor values
*/
void updatePmsReadings()
{
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
  Start OTA
*/
void initOta()
{
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
}

/*
  Connect to Wifi. Returns false if it can't connect.
*/
bool initWifi()
{
  // Let WiFi Manager handle connections
  wc.setDebug(true);

  /* Set our callbacks */
  wc.setAPCallback(configModeCallback);

  // Set how many connection attempts before we fail and go to captive portal mode
  wc.setRetryAttempts(5);

  // How long to wait in captive portal mode before we try to reconnect
  wc.setAPModeTimeoutMins(5);

  // Set Access Point name for captive portal mode
  char ap_name[13];
  sprintf(ap_name, "linka-%x", g_device_id);
  wc.setAPName(ap_name);

  //set config save notify callback
  wc.setSaveConfigCallback(saveConfigCallback);

  // Configure custom parameters
  wc.addParameter(&api_key_param);
  wc.addParameter(&latitude_param);
  wc.addParameter(&longitude_param);
  wc.addParameter(&sensor_param);
  wc.addParameter(&description_param);
  wc.addParameter(&api_url_param);

  //wc.resetSettings(); //helper to remove the stored wifi connection, comment out after first upload and re upload

  /*
     AP_NONE = Continue executing code
     AP_LOOP = Trap in a continuous loop - Device is useless
     AP_RESET = Restart the chip
     AP_WAIT  = Trap in a continuous loop with captive portal until we have a working WiFi connection
  */
  if (!wc.autoConnect() || force_captive_portal) { // try to connect to wifi
    /* We could also use button etc. to trigger the portal on demand within main loop */
    wc.startConfigurationPortal(AP_WAIT); //if not connected show the configuration portal
  }

  if (shouldSaveConfig) {
    Serial.println("Saving configurations to filesystem");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["api_key"] = api_key_param.getValue();
    json["latitude"] = latitude_param.getValue();
    json["longitude"] = longitude_param.getValue();
    json["sensor"] = sensor_param.getValue();
    json["description"] = description_param.getValue();
    json["api_url"] = api_url_param.getValue();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);

    strcpy(api_key, json["api_key"]);
    strcpy(latitude, json["latitude"]);
    strcpy(longitude, json["longitude"]);
    strcpy(sensor, json["sensor"]);
    strcpy(description, json["description"]);
    strcpy(api_url, json["api_url"]);

    configFile.close();
  }
}

/*
   Init filesystem to store parameters
*/
void initFS(void)
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {

          strcpy(api_key, json["api_key"]);
          strcpy(latitude, json["latitude"]);
          strcpy(longitude, json["longitude"]);
          strcpy(sensor, json["sensor"]);
          strcpy(description, json["description"]);
          strcpy(api_url, json["api_url"]);
          if (strcmp(api_key, "") == 0) {
            Serial.println("Stored parameters are empty, reset the parameters");
            force_captive_portal = true;
          }

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}
