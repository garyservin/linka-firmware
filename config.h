/* ----------------- General config -------------------------------- */
/* Particulate Matter Sensor */
uint32_t    g_pms_warmup_period   = 30;               // Seconds to warm up PMS before reading
uint32_t    g_pms_report_period   = 120;              // Seconds between reports
const char* sensor                = "PMS7003";        // Sensor model

/* Serial */
#define     SERIAL_BAUD_RATE    115200                // Speed for USB serial console

/* HTTP Client */
const char* api_url               = "http://test.com";
const char* api_key               = ""; // API Key for server
const char* latitude              = "";
const char* longitude             = "";

/* ----------------- Hardware-specific config ---------------------- */
#define     ESP_WAKEUP_PIN          D0               // To reset ESP8266 after deep sleep
#define     PMS_RX_PIN              D4               // Rx from PMS (== PMS Tx)
#define     PMS_TX_PIN              D2               // Tx to PMS (== PMS Rx)
#define     PMS_BAUD_RATE         9600               // PMS5003 uses 9600bps
