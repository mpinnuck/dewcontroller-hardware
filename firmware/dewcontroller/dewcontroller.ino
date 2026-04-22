// DewHeaterController — Dew Spread Control
// Replaces RH threshold gating with dew-point spread gating (Ta - Td)
// Default dew spread threshold = 3.0 °C, hysteresis = 1.0 °C
// Board XAIO ESP32S3 with ATH40 sensor


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SHT4x.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_netif.h>
#include "esp_wifi.h"
#include "esp_sntp.h"
#include <WebServer.h>
#include <ArduinoJson.h>
#include "esp_log.h"
#include <time.h>
#include <HTTPClient.h>
#include <math.h>

#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

#define DEVICE_VERSION "v4.5.0"
#define DEVICE_NAME "DewHeaterController"
#define SIMULATE_HARDWARE 0
#define CONFIG_FILE "/config.json"

// Weather API key — rotate if this firmware is ever shared publicly
#define WEATHER_API_KEY "5356e369de454c6f96e369de450c6f22"
#define CALIBRATION_FILE "/calibration.csv"
#define LOG_FILE "/thermal.log"
#define LOG_PRUNE_SIZE 16384       // 16KB — date-based prune threshold
#define MAX_THERMAL_LOG_SIZE 32768 // 32KB — hard cap, truncates oldest half
#define LOOP_INTERVAL_MS 1000
#define WEATHER_POLL_INTERVAL_MS 5 * 60 * 1000 // 300000 ms or 5 minutes
#define WIFI_STATUS_CHECK_INTERVAL_MS 5000 // 5 seconds - faster disconnect detection

// Main loop timing constants (Web/Network - runs in loop())
#define WIFI_RECONNECT_INTERVAL_MS 10000  // 10 seconds
#define NTP_CHECK_INTERVAL_MS 120000      // 2 minutes
#define RSSI_UPDATE_INTERVAL_MS 120000    // 2 minutes
#define MAIN_LOOP_DELAY_MS 25             // Main loop cycle time
#define AP_STA_SCAN_INTERVAL_MS 30000     // 30 seconds — scan for STA SSID while in AP fallback

// Control task timing constants (Background - runs in task)
#define APP_UPDATE_INTERVAL_MS 1000       // 1 second - heater control loop
#define CALIBRATION_UPDATE_INTERVAL_MS 30000  // 30 seconds - matches CAL_SAMPLE_INTERVAL_MS
#define THERMAL_LOG_INTERVAL_MS 600000    // 10 minutes
#define CONTROL_TASK_DELAY_MS 50          // Background control task cycle

#define I2C_SDA 5
#define I2C_SCL 6
#define THERMISTOR_PIN 3
#define PWM_PIN 7
#define LED_PIN 21
#define ON LOW
#define OFF HIGH

// default coefficients (quadratic fit: Tg = A*V² + B*V + C)
#define GLASSCOEFFA  -23.10
#define GLASSCOEFFB   61.95
#define GLASSCOEFFC  -19.20
#define GLASSCOEFFDP  2  // number of decimal places



// Heater defaults
#define DEFAULT_DEWSPREAD     3.0
#define DEWSPREAD_HYSTERISIS  1.0

#define DEFAULT_TEMPDELTA     4.0

#define DEFAULT_WIFI_SSID     "AstroNetC925"

#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x) do {} while (0)
#endif


enum SensorSource {
  SOURCE_AHT20,
  SOURCE_SHT40,
  SOURCE_WEATHER
};
SensorSource sensorSource = SOURCE_SHT40;  // Try external SHT40 first, fallback to weather  

SemaphoreHandle_t   i2cMutex;
SemaphoreHandle_t   cacheMutex;
SemaphoreHandle_t   logMutex;
TaskHandle_t        sensorTaskHandle;
TaskHandle_t        controlTaskHandle;
WebServer           server(80);
TwoWire             I2CBus = TwoWire(1);  // Dedicated I2C bus
Adafruit_AHTX0      aht;
Adafruit_SHT4x      sht4;

bool webServerInitialized = false;  // guard against repeated route registration

// -------------- Status and config variables --------------
float ambientTemp = 0, humidity = 0, glassTemp = 0;
float targetDelta = DEFAULT_TEMPDELTA;                    // desired (Tg - Ta)

// NEW: dew-spread control (spread = Ta - Td)
float dewSpreadThreshold  = DEFAULT_DEWSPREAD;            // default °C
float dewSpreadHysteresis = DEWSPREAD_HYSTERISIS;         // °C to avoid chatter

// Thermistor coefficients (best-fit)
float glassCoeffA = GLASSCOEFFA;
float glassCoeffB = GLASSCOEFFB;
float glassCoeffC = GLASSCOEFFC;

String  wifiSSID = DEFAULT_WIFI_SSID;
String  wifiPass = "";
// Timezone rule for TZ environment (AEST/AEDT for Sydney by default)
String  timezoneRule = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
bool    heaterEnabled = false, calibrating = false;
bool    dewGate = false;                             // global dew-gate state for logging

// -------------- Calibration variables --------------
int     calCount = 0, pwm = 0;
bool    pwmTest = false;
volatile bool calResetRequested = false;  // flag for /calibrate handler to reset statics in updateCalibration()
const unsigned long CAL_SAMPLE_INTERVAL_MS = 30000;  // 30 seconds

const int MAX_LOG_SIZE = 65536;  // 64KB log buffer - keep more startup history
String  logBuffer;

bool ntpInitialized = false;
bool logPruned = false;                  // true after first NTP-triggered thermal log prune
bool apFallbackActive = false;           // true when AP mode started due to STA failure
unsigned long apFallbackTime = 0;        // millis() when AP fallback began
const unsigned long AP_FALLBACK_COOLDOWN_MS = 300000;  // 5 minutes before retrying STA

// Background task cache variables
float cached_td = NAN;
float cached_spread = NAN;
String cached_calFileSize = "0 KB";
String cached_storageInfo = "0/0 KB";
int cached_rssi = -99;
unsigned long lastCacheUpdate = 0;

// Forward declarations
void setupNTP();
void setupWebServer();
void handleNTP(bool forceCheck = false);
void pruneThermalLog(const struct tm& timeinfo);


// -------------- Utility --------------

// Utility: decode Wi-Fi status to readable string
String wifiStatusToString(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:      return "Idle";
    case WL_NO_SSID_AVAIL:    return "No SSID available";
    case WL_SCAN_COMPLETED:   return "Scan completed";
    case WL_CONNECTED:        return "Connected";
    case WL_CONNECT_FAILED:   return "Connect failed";
    case WL_CONNECTION_LOST:  return "Connection lost";
    case WL_DISCONNECTED:     return "Disconnected";
    default:                  return "Unknown (" + String(status) + ")";
  }
}

// Logs the current WiFi.status() with a readable message
void logWiFiStatus(const String& prefix = "WiFi") {
  wl_status_t status = WiFi.status();
  String msg = wifiStatusToString(status);

  if (status == WL_CONNECTED) {
    msg += " (" + WiFi.localIP().toString() + ", RSSI " + String(WiFi.RSSI()) + " dBm)";
  }

  sendLog(prefix + " → " + msg);
}

// Apply timezone rule locally, even without NTP (used in AP mode)
void applyLocalTimezone() {
  if (timezoneRule.isEmpty()) {
    timezoneRule = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
    sendLog("⚙️ No timezone rule found — defaulting to Australia/Sydney");
  }

  setenv("TZ", timezoneRule.c_str(), 1);
  tzset();

  sendLog("🕒 Local timezone applied: " + timezoneRule);
}

String getStorageUsedSummary() {
  size_t total = SPIFFS.totalBytes();
  size_t used = SPIFFS.usedBytes();

  float usedKB = used / 1024.0;
  float totalKB = total / 1024.0;

  return String(usedKB, 1) + "/" + String(totalKB, 1) + " KB";
}

String getCalibrationFileSizeKB(const char* filename) {
  if (!SPIFFS.exists(filename)) {
    return "0 KB";
  }
  File f = SPIFFS.open(filename, FILE_READ);
  size_t size = f.size();
  f.close();
  float sizeKB = size / 1024.0;
  String result = String(sizeKB, 1) + " KB";
  if (sizeKB > 200.0) {
    result += " ⚠️";
  }
  return result;
}

// Returns local wall time if available, otherwise relative uptime since boot.
String getStatusTimeString() {
  struct tm timeinfo;
  char timeStr[32];

  if (getLocalTime(&timeinfo, 50)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStr);
  }

  unsigned long secs = millis() / 1000;
  return "+" + String(secs) + "s";
}

// Weather API (change as required)
const char* WEATHER_API_URL = "https://api.weather.com/v2/pws/observations/current?stationId=ISYDNEY478&format=json&units=m&apiKey=" WEATHER_API_KEY;

// Write ambient temp and humidity under mutex for cross-core safety
void writeSensorValues(float newTemp, float newHumidity) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    ambientTemp = newTemp;
    humidity = newHumidity;
    xSemaphoreGive(i2cMutex);
  }
}

// Read ambient temp and humidity under mutex for cross-core safety
void readSensorValues(float &outTemp, float &outHumidity) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    outTemp = ambientTemp;
    outHumidity = humidity;
    xSemaphoreGive(i2cMutex);
  } else {
    outTemp = ambientTemp;
    outHumidity = humidity;
  }
}

// Magnus formula for dew point (Celsius)
float dewPointC(float T, float RH) {
  if (isnan(T) || isnan(RH) || RH <= 0.0f) return NAN;
  const float a = 17.62f;
  const float b = 243.12f; // °C
  float gamma = (a * T) / (b + T) + log(RH / 100.0f);
  return (b * gamma) / (a - gamma);
}

void sendLog(const String& msg) {
  char timeStr[32];  // Local buffer - not static to prevent corruption
  struct tm timeinfo;

  // Use current local time if available, otherwise show boot time
  if (getLocalTime(&timeinfo, 50)) {  // short timeout
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  } else {
    // Before NTP sync, show time since boot
    unsigned long secs = millis() / 1000;
    snprintf(timeStr, sizeof(timeStr), "+%lus", secs);
  }

  String logLine = "[" + String(timeStr) + "] " + msg;
  DEBUG_PRINT(logLine);

  // Protect logBuffer from concurrent access by multiple cores/tasks
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100))) {
    logBuffer += logLine + "\n";
    if (logBuffer.length() > MAX_LOG_SIZE) {
      logBuffer = logBuffer.substring(logBuffer.length() - MAX_LOG_SIZE);
    }
    xSemaphoreGive(logMutex);
  } else {
    // If mutex not available yet (early startup) or timeout, just print to serial
    DEBUG_PRINT("⚠️ Log mutex unavailable");
  }
}

bool fetchOutdoorWeather(float* outTemp, float* outHumidity) {
  HTTPClient http;
  http.begin(WEATHER_API_URL);

  int httpCode = http.GET();
  if (httpCode == 200) {
    String payload = http.getString();

    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err) {
      sendLog("❌ JSON parse error: " + String(err.c_str()));
      http.end();
      return false;
    }

    JsonObject obsRoot = doc["observations"][0];
    JsonObject metric = obsRoot["metric"];

    if (obsRoot.isNull() || metric.isNull()) {
      sendLog("❌ Weather API missing required data.");
      http.end();
      return false;
    }

    float temp = metric["temp"] | NAN;
    float hum  = obsRoot["humidity"] | NAN;  // fixed

    // Data extracted - values displayed in main window

    if (!isnan(temp) && !isnan(hum)) {
      *outTemp = temp;
      *outHumidity = hum;
      http.end();
      return true;
    } else {
      sendLog("❌ Weather API returned invalid values.");
      http.end();
      return false;
    }
  } else {
    sendLog("❌ Weather API HTTP error: " + String(httpCode));
    http.end();
    return false;
  }
}

void saveConfig() {
  // Synchronous save with atomic write to prevent corruption
  StaticJsonDocument<512> doc;
  doc["delta"] = targetDelta;
  doc["dewSpread"] = dewSpreadThreshold;
  doc["dewSpreadHysteresis"] = dewSpreadHysteresis;
  doc["glassCoeffA"] = glassCoeffA;
  doc["glassCoeffB"] = glassCoeffB;
  doc["glassCoeffC"] = glassCoeffC;
  doc["heater"] = heaterEnabled;
  doc["ssid"] = wifiSSID;
  doc["password"] = wifiPass;
  doc["timezoneRule"] = timezoneRule;

  // Write to temporary file first (atomic operation)
  const char* tempFile = "/config.tmp";
  File file = SPIFFS.open(tempFile, FILE_WRITE);
  if (!file) {
    sendLog("❌ Failed to open temp config file");
    return;
  }
  
  size_t bytesWritten = serializeJson(doc, file);
  file.close();
  
  if (bytesWritten == 0) {
    sendLog("❌ Failed to write config");
    SPIFFS.remove(tempFile);
    return;
  }
  
  // Atomic rename - if this fails, old config is still intact
  SPIFFS.remove(CONFIG_FILE);
  SPIFFS.rename(tempFile, CONFIG_FILE);
  
  sendLog("💾 Config saved (" + String(bytesWritten) + " bytes)");
}

void loadConfig() {
  if (!SPIFFS.exists(CONFIG_FILE)) {
    sendLog("⚠️ No config found, using defaults");
    return;
  }

  File file = SPIFFS.open(CONFIG_FILE, FILE_READ);
  if (!file) {
    sendLog("❌ Failed to open config file");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();

  if (err) {
    sendLog("❌ Failed to parse config: " + String(err.c_str()));
    
    // Delete corrupted config file and use defaults
    if (err == DeserializationError::EmptyInput || err == DeserializationError::InvalidInput) {
      sendLog("🗑 Deleting corrupted config file");
      SPIFFS.remove(CONFIG_FILE);
      sendLog("⚠️ Using defaults - please reconfigure via web interface");
    }
    return;
  }

  targetDelta          = doc["delta"]               | DEFAULT_TEMPDELTA;
  dewSpreadThreshold   = doc["dewSpread"]           | DEFAULT_DEWSPREAD;        // default
  dewSpreadHysteresis  = doc["dewSpreadHysteresis"] | DEWSPREAD_HYSTERISIS;     // default

  // Back-compat for older configs that only had "humidity" key:
  if (!doc.containsKey("dewSpread") && doc.containsKey("humidity")) {
    dewSpreadThreshold = DEFAULT_DEWSPREAD; // migrate to sensible default
  }

  heaterEnabled       = doc["heater"]       | false;
  wifiSSID            = doc["ssid"]         | "";
  wifiPass            = doc["password"]     | "";
  timezoneRule        = doc["timezoneRule"] | "AEST-10AEDT,M10.1.0/2,M4.1.0/3";

  sendLog("✅ Config loaded from SPIFFS");
  sendLog(" Temperature Delta: " + String(targetDelta, 2));
  sendLog("📦 Dew Spread Threshold: " + String(dewSpreadThreshold, 2) + " °C");
  sendLog("📦 Dew Spread Hysteresis: " + String(dewSpreadHysteresis, 2) + " °C");
  sendLog("📦 Heater Enabled: " + String(heaterEnabled ? "true" : "false"));
  sendLog("📦 WiFi SSID: " + wifiSSID);
  sendLog("📦 Timezone Rule: " + timezoneRule);
  sendLog("📦 Tg = A·V² + B·V + C");
  sendLog("📦 A = " + String(glassCoeffA, GLASSCOEFFDP));
  sendLog("📦 B = " + String(glassCoeffB, GLASSCOEFFDP));
  sendLog("📦 C = " + String(glassCoeffC, GLASSCOEFFDP));
}

float readThermistorVoltage() {
  return analogRead(THERMISTOR_PIN) * 3.3 / 4095.0;
}

float readGlassTemp() {
  float V = readThermistorVoltage();
  return (glassCoeffA * V * V) + (glassCoeffB * V) + glassCoeffC;
}

// Called every LOOP_INTERVAL_MS
void updateGlassTemp() {
  glassTemp = readGlassTemp();
}

// Log thermal lag periodically - now called less frequently from main loop
void logThermalLagInfo() {
  char timeStr[32] = "??";
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  static float prevTa = 0.0;
  static unsigned long prevTime = 0;

  float Ta, h;
  readSensorValues(Ta, h);
  float Tg = glassTemp;

  unsigned long now = millis();

  // Initialize on first call
  if (prevTime == 0) {
    prevTa = Ta;
    prevTime = now;
    return;
  }

  float dtSeconds = (now - prevTime) / 1000.0;
  float gradientPerHour = (Ta - prevTa) / dtSeconds * 3600.0;
  float delta = Tg - Ta;

  // Extra dew metrics
  float td = dewPointC(Ta, h);
  float Vt = readThermistorVoltage();
  float TgTarget = isnan(td) ? 0.0f : (td + targetDelta);

  // Append to persistent thermal log
  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (f) {
    char buf[210];
    snprintf(buf, sizeof(buf), "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%d,%.2f",
             timeStr, Ta, Tg, delta, gradientPerHour,
             isnan(td) ? 0.0f : (Ta - td),
             isnan(td) ? 0.0f : td,
             isnan(h) ? 0.0f : h,
             pwm, Vt,
             dewGate ? 1 : 0, TgTarget);
    f.println(buf);
    f.close();
  }

  prevTa = Ta;
  prevTime = now;
}

void setupWiFi() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Check if we have valid WiFi credentials
  if (wifiSSID.isEmpty() || wifiPass.isEmpty()) {
    sendLog("⚠️ No WiFi credentials configured - starting in AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DewHeaterSetup");
    IPAddress apIP = WiFi.softAPIP();
    sendLog("📡 AP mode started — connect to SSID 'DewHeaterSetup'");
    sendLog("📶 AP IP address: " + apIP.toString());
    applyLocalTimezone();
    setupWebServer();
    return;
  }

  sendLog("🔌 Connecting to Wi-Fi SSID: " + wifiSSID + "...");

  // --- Step 1: Clean disconnect and setup STA mode ---
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Stronger signal for tough 2G routers

  // --- Step 1.5: TPG Router-specific hostname setup ---
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  int attempts = 0;
  while (!netif && attempts++ < 10) {  // wait up to ~1s for interface
    delay(100);
    netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  }

  if (netif) {
    // Stop DHCP before changing hostname
    esp_netif_dhcpc_stop(netif);
    delay(100);  // Extra delay for TPG routers

    // Set hostname on the ESP network interface - try shorter name for TPG
    esp_netif_set_hostname(netif, "DewController");
    sendLog("🏷 Hostname set via esp_netif before DHCP: DewController");

    // Force clear any cached DHCP state for TPG compatibility
    esp_netif_dhcpc_start(netif);
  } else {
    sendLog("⚠️ Failed to set hostname — esp_netif not ready");
  }

  // Set Arduino layer hostname
  WiFi.setHostname("DewController");

  // --- Step 2: Begin initial connection ---
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  delay(500); // short warm-up

  const int maxRetries = 40;  // total loop time ≈ 40s
  int retries = 0;
  wl_status_t lastStatus = WL_IDLE_STATUS;

  // --- Step 3: Poll connection progress safely ---
  sendLog("⏳ Waiting for Wi-Fi connection...");
  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    wl_status_t status = WiFi.status();

    // Log only when status changes
    if (status != lastStatus) {
      sendLog("⏳ Wi-Fi status: " + wifiStatusToString(status));
      lastStatus = status;
    }

    // Do not manually reset or reconnect here — let the stack retry itself
    delay(1000);
    retries++;
  }

  // --- Step 4: Handle success immediately ---
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    sendLog("✅ Wi-Fi connected");
    sendLog("🌐 IP address: " + WiFi.localIP().toString());
    sendLog("📶 RSSI: " + String(WiFi.RSSI()) + " dBm");

    if (!MDNS.begin("DewController")) {
      sendLog("❌ mDNS failed to start!");
    } else {
      sendLog("✅ mDNS responder started as DewController.local");
      yield();
    }

    // Initialize NTP only once
    if (!ntpInitialized) {
      setupNTP();
      ntpInitialized = true;
    } else {
      sendLog("⏱ NTP already active — skipping reinit");
    }

    setupWebServer();
    return;
  }

  // --- Step 5: Grace period for late DHCP ---
  sendLog("⚠️ Connection not confirmed after " + String(maxRetries) + " attempts.");
  sendLog("⏱ Waiting up to 15 more seconds for late DHCP...");

  unsigned long graceStart = millis();
  bool connected = false;
  while (millis() - graceStart < 15000) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    delay(1000);
  }

  if (connected) {
    sendLog("✅ Late Wi-Fi success detected!");
    digitalWrite(LED_PIN, HIGH);
    sendLog("🌐 IP address: " + WiFi.localIP().toString());
    sendLog("📶 RSSI: " + String(WiFi.RSSI()) + " dBm");

    if (!MDNS.begin("DewController")) {
      sendLog("❌ mDNS failed to start!");
    } else {
      sendLog("✅ mDNS responder started as DewController.local");
      yield();
    }

    applyLocalTimezone();

    if (!ntpInitialized) {
      setupNTP();
      ntpInitialized = true;
    } else {
      sendLog("⏱ NTP already active — skipping reinit");
    }

    setupWebServer();
    return;
  }

  // --- Step 6: Fallback to AP mode ---
  sendLog("❌ Wi-Fi connection failed after extended wait. Switching to Access Point mode...");
  apFallbackActive = true;
  apFallbackTime = millis();

  // Clean teardown of STA before starting AP — prevents radio state issues
  WiFi.disconnect(true, true);  // disconnect + erase STA credentials from RAM
  delay(200);
  WiFi.mode(WIFI_OFF);          // fully power down WiFi radio
  delay(500);                    // let radio settle
  WiFi.mode(WIFI_AP);           // start fresh in AP mode
  delay(200);
  WiFi.softAP("DewHeaterSetup");
  delay(500);                    // let AP stabilize before checking IP

  IPAddress apIP = WiFi.softAPIP();
  if (apIP == IPAddress(0, 0, 0, 0)) {
    sendLog("⚠️ AP IP is 0.0.0.0 — retrying softAP...");
    WiFi.softAP("DewHeaterSetup");
    delay(1000);
    apIP = WiFi.softAPIP();
  }
  sendLog("📡 AP mode started — connect to SSID 'DewHeaterSetup'");
  sendLog("📶 AP IP address: " + apIP.toString());
  applyLocalTimezone(); // keep timestamps correct in AP mode
  setupWebServer();
}


void setupNTP() {
  applyLocalTimezone(); // Ensure local time context before logging
  sendLog("🕓 Initializing NTP...");

  if (timezoneRule.isEmpty()) {
    timezoneRule = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
    sendLog("⚙️ No timezone rule found — defaulting to Australia/Sydney");
  }

  // Step 1 — configure NTP (sets up SNTP client)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  sendLog("🌐 NTP client configured with timezone rule: " + timezoneRule);

  // Step 2 — immediately apply the timezone AFTER NTP setup
  setenv("TZ", timezoneRule.c_str(), 1);
  tzset();

  // Step 3 — now safely call getLocalTime()
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
    sendLog(String("✅ NTP time synchronized: ") + buf);
    if (!logPruned) {
      logPruned = true;
      pruneThermalLog(timeinfo);
    }
  } else {
    sendLog("⚠️ NTP not yet synchronized (will retry automatically)");
  }
}

void setupWebServer() {
  // Guard against repeated route registration on WiFi reconnect
  if (webServerInitialized) {
    server.begin();
    sendLog("🌐 Web server restarted (routes already registered)");
    return;
  }
  webServerInitialized = true;

  server.on("/", []() {
    String html = R"rawliteral(
      <html><head><meta charset="utf-8"><title>Dew Heater</title>
      <style>
        html, body {
          margin: 0; padding: 0; height: 100%;
          font-family: sans-serif; box-sizing: border-box;
          display: flex; flex-direction: column;
        }
        #main { display: flex; padding: 10px; flex-direction: row; align-items: flex-start; gap: 20px; justify-content: flex-start; }
        #status, #settings { flex: 0 0 auto; }
        @media (max-width: 600px) {
          #main { flex-direction: column; }
          #status, #settings { width: 100%; }
        }
        #log {
          flex-grow: 1; overflow-y: auto;
          border-top: 1px solid #ccc; background: #f8f8f8;
          padding: 10px; white-space: pre-wrap;
          box-sizing: border-box; max-height: calc(100vh - 240px);
          min-height: 100px; font-family: monospace; font-size: 0.9rem;
        }
        #controls { display: flex; flex-direction: column; gap: 10px; padding: 10px; }
        .inputrow { display: flex; align-items: center; margin: 5px 0; }
        .inputrow label { width: 210px; }
        .inputrow input { flex: 1; }
        #dataModal {
          display: none; position: fixed; top: 0; left: 0;
          width: 100%; height: 100%; background: rgba(0,0,0,0.5);
          z-index: 1000; justify-content: center; align-items: center;
        }
        #dataModal.active { display: flex; }
        #dataModalBox {
          background: white; border-radius: 8px; padding: 20px;
          width: 80%; max-width: 800px; max-height: 80vh;
          display: flex; flex-direction: column; box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        #dataModalHeader {
          display: flex; justify-content: space-between; align-items: center;
          margin-bottom: 10px;
        }
        #dataModalHeader h3 { margin: 0; }
        #dataModalClose {
          background: none; border: none; font-size: 1.5rem; cursor: pointer;
          color: #666; padding: 0 4px; line-height: 1;
        }
        #dataModalClose:hover { color: #000; }
        #dataModalContent {
          flex: 1; overflow-y: auto; white-space: pre-wrap;
          font-family: monospace; font-size: 0.85rem; background: #f8f8f8;
          border: 1px solid #ccc; padding: 10px; border-radius: 4px;
          user-select: text;
        }
        #dataModalButtons { display: flex; gap: 10px; margin-top: 10px; }
        #dataModalButtons button { padding: 6px 16px; cursor: pointer; }
      </style>

      <script>
        let logPaused = false;

        function updateStatus() {
          // Add 3 second timeout for faster disconnect detection
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), 3000);
          
          fetch('/status.json', { signal: controller.signal })
            .then(r => r.json())
            .then(data => {
              clearTimeout(timeoutId);
              // Update WiFi status - connected with signal strength and color coding
            const wifiStatus = document.getElementById("wifiStatus");
            const signal = parseInt(data.wifiSignal) || 0;
            
            let quality = "";
            let bgColor = "";
            
            if (signal >= -50) {
              quality = "Excellent";
              bgColor = "#ccffcc";
            } else if (signal >= -60) {
              quality = "Good";
              bgColor = "#ccffcc";
            } else if (signal >= -70) {
              quality = "Fair";
              bgColor = "#ffcc99";
            } else if (signal >= -80) {
              quality = "Weak";
              bgColor = "#ffcc99";
            } else {
              quality = "Very Weak";
              bgColor = "#ffcccc";
            }
            
            const signalStr = data.wifiSignal ? ` ${quality} (${data.wifiSignal}dBm)` : "";
            wifiStatus.innerText = `📶 Connected${signalStr}`;
            wifiStatus.style.backgroundColor = bgColor;
            wifiStatus.style.color = "#333";

            document.getElementById("ambient").innerText = data.ambient;
            document.getElementById("humidity").innerText = data.humidity;
            document.getElementById("glass").innerText = data.glass;
            document.getElementById("delta").innerText = data.delta;
            document.getElementById("dewpoint").innerText  = data.dewpoint;
            document.getElementById("dewspread").innerText = data.dewspread;

            const heaterOn = data.heater;
            const heaterSpan = document.getElementById("heater");
            const heaterRow = document.getElementById("heaterRow");
            heaterSpan.innerText = heaterOn ? "ON" : "OFF";
            heaterRow.style.backgroundColor = heaterOn ? "#c0ffc0" : "transparent";

            const pwmVal = parseInt(data.pwm);
            const pwmSpan = document.getElementById("pwm");
            const pwmRow = document.getElementById("pwmRow");
            pwmSpan.innerText = pwmVal;
            pwmRow.style.backgroundColor = pwmVal > 0 ? "#c0ffc0" : "transparent";

            document.getElementById("calibrationFileSize").innerText = data.calibrationFileSize;
            document.getElementById("storageInfo").innerText = data.storageInfo;
            document.getElementById("version").innerText = " " + data.version;

            const calButton = document.getElementById("calButton");
            if (data.calibrating) { calButton.innerText = "Stop Calibration"; }
            else { calButton.innerText = "Start Calibration"; }

            const heaterButton = document.getElementById("heaterButton");
            if (heaterOn) { heaterButton.innerText = "Turn Heater OFF"; }
            else { heaterButton.innerText = "Turn Heater ON"; }
          }).catch(error => {
            // Update WiFi status - disconnected
            const wifiStatus = document.getElementById("wifiStatus");
            wifiStatus.innerText = "📶 Disconnected";
            wifiStatus.style.backgroundColor = "#ffcccc";
            wifiStatus.style.color = "#666";
            console.log('Status update failed:', error);
          });
        }

        function toggleHeater() {
          fetch("/toggle", { method: "POST" })
            .then(() => {
              const btn = document.getElementById("heaterButton");
              if (btn.innerText === "Turn Heater ON") btn.innerText = "Turn Heater OFF";
              else btn.innerText = "Turn Heater ON";
              setTimeout(updateLog, 300);
            });
        }

        function updateLog() {
          fetch('/log?full=1').then(r => r.text()).then(data => {
            const box = document.getElementById("log");
            box.innerText = data;
            box.scrollTop = box.scrollHeight;
          });
        }

        function updatePWMValue(val) {
          document.getElementById("pwmValue").innerText = val + "%";
        }

        function sendPWMValue() {
          const val = document.getElementById("pwmTest").value;
          const btn = document.getElementById("pwmButton");

          fetch("/pwmtest", {
            method: "POST",
            headers: { "Content-Type": "application/x-www-form-urlencoded" },
            body: "pwm=" + val
          }).then(() => {
            if (btn.innerText === "Apply") btn.innerText = "Stop";
            else btn.innerText = "Apply";
            console.log("PWM test value sent: " + val);
            setTimeout(updateLog, 300);
          });
        }

        function clearLog() {
          fetch("/clearlog", { method: "POST" })
            .then(() => {
              document.getElementById("log").innerText = "🧹 Clearing log.";
            });
        }

        function showCalibration() {
          fetch('/downloadcal').then(r => {
            if (r.ok) return r.text();
            throw new Error('No data');
          }).then(text => showDataModal('Calibration Data', text))
            .catch(() => showDataModal('Calibration Data', 'No calibration data found.'));
        }

        function toggleCalibration() {
         fetch("/calibrate", { method: "POST" })
            .then(() => {
              const btn = document.getElementById("calButton");
              if (btn.innerText === "Start Calibration") btn.innerText = "Stop Calibration";
              else btn.innerText = "Start Calibration";
              setTimeout(updateLog, 300);
            });
        }

        function clearCalibration() {
          fetch("/clearcal", { method: "POST" })
            .then(() => {
              document.getElementById("log").innerText = "🧹 Clearing calibration data.";
              setTimeout(updateLog, 300);
            });
        }

        function clearThermalLog() {
          fetch("/clearlog_thermal", { method: "POST" })
            .then(() => {
              document.getElementById("log").innerText = "🧹 Clearing thermal log.";
              setTimeout(updateLog, 300);
            });
        }

        function showThermalLog() {
          fetch('/downloadlog').then(r => {
            if (r.ok) return r.text();
            throw new Error('No data');
          }).then(text => showDataModal('Thermal Log', text))
            .catch(() => showDataModal('Thermal Log', 'No thermal log found.'));
        }

        function showDataModal(title, content) {
          document.getElementById('dataModalTitle').innerText = title;
          document.getElementById('dataModalContent').innerText = content;
          document.getElementById('dataModal').classList.add('active');
          const el = document.getElementById('dataModalContent');
          setTimeout(() => { el.scrollTop = el.scrollHeight; }, 50);
        }

        function closeDataModal() {
          document.getElementById('dataModal').classList.remove('active');
        }

        function copyModalData() {
          const text = document.getElementById('dataModalContent').innerText;
          const btn = document.getElementById('copyBtn');
          const ta = document.createElement('textarea');
          ta.value = text;
          ta.style.position = 'fixed';
          ta.style.opacity = '0';
          document.body.appendChild(ta);
          ta.select();
          document.execCommand('copy');
          document.body.removeChild(ta);
          btn.innerText = 'Copied!';
          setTimeout(() => btn.innerText = 'Copy to Clipboard', 1500);
        }

        function toggleLogPause() {
          logPaused = !logPaused;
          const btn = document.getElementById("pauseLogButton");
          btn.innerText = logPaused ? "Resume Log" : "Pause Log";
        }

        function downloadCalibration() {
          window.location.href = "/downloadcal";
        }

        setInterval(() => { 
          updateStatus();
          if (!logPaused) updateLog();
        }, )rawliteral" + String(WIFI_STATUS_CHECK_INTERVAL_MS) + R"rawliteral();

        window.onload = () => { updateStatus(); updateLog(); };

      </script>
      </head><body>
      <h2 style="padding:10px">
        Dew Heater Controller 
        <span id="version" style="font-size: 0.6em; color: #666;"></span>
        <span id="wifiStatus" style="float: right; font-size: 0.8em; padding: 5px 10px; border-radius: 4px; background-color: #ffcccc; color: #666;">📶 Disconnected</span>
      </h2>
      <div id="main">
        <div id="status">
          <table style="font-family: monospace; font-size: 1rem;">
            <tr><td>🌡 Ambient:</td>       <td><span id='ambient'>--</span> °C</td></tr>
            <tr><td>💧 Humidity:</td>      <td><span id='humidity'>--</span> %</td></tr>
            <tr><td>🧮 Dew Point:</td>     <td><span id='dewpoint'>--</span> °C</td></tr>
            <tr><td>📉 Dew Spread:</td>    <td><span id='dewspread'>--</span> °C</td></tr>
            <tr><td>🔍 Glass:</td>         <td><span id='glass'>--</span> °C</td></tr>
            <tr><td>📏 Delta (Tg - Td):</td><td><span id='delta'>--</span> °C</td></tr>
            <tr id="heaterRow"><td>🔌 Heater:</td> <td><span id='heater'>--</span></td></tr>
            <tr id="pwmRow"><td>🔥 Heating Power:</td> <td><span id='pwm'>--</span>%</td></tr>
            <tr><td>🗂 Calibration File:</td> <td><span id='calibrationFileSize'>--</span></td></tr>
            <tr><td>💾 Storage Used:</td> <td><span id='storageInfo'>--</span></td></tr>
          </table>
        </div>

        <div id="settings">
          <form action="/settings" method="POST">
            <div class="inputrow">
              <label>Margin above Dew Point (°C):</label>
              <input name="delta" value=")rawliteral" + String(targetDelta, 1) + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Dew Spread (°C):</label>
              <input name="dewspread" value=")rawliteral" + String(dewSpreadThreshold, 1) + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Dew Spread Hysteresis (°C):</label>
              <input name="dewhyst" value=")rawliteral" + String(dewSpreadHysteresis, 1) + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>SSID:</label>
              <input name="ssid" value=")rawliteral" + wifiSSID + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Password:</label>
              <input name="password" type="password" placeholder="(unchanged)">
            </div>
            <div class="inputrow">
              <label>Timezone Rule:</label>
              <input name="timezoneRule" value=")rawliteral" + timezoneRule + R"rawliteral(">
            </div>
            <button type="submit">Update Settings</button>
          </form>
            <p> T_glass = )rawliteral" 
              + String(glassCoeffA, 1) + R"rawliteral(·V² + )rawliteral"
              + String(glassCoeffB, 1) + R"rawliteral(·V + )rawliteral"
              + String(glassCoeffC, 1) + R"rawliteral(</p>
        </div>
      </div>

      <div id="controls">
        <!-- Row 1 -->
        <div style="display: flex; gap: 10px; flex-wrap: wrap;">
          <button id="heaterButton" type="button" onclick="toggleHeater()">Turn Heater ON</button>
          <button id="calButton" type="button" onclick="toggleCalibration()">Start Calibration</button>
          <button type="button" onclick="showCalibration()">Show Calibration</button>
          <button type="button" onclick="clearLog()">Clear Log</button>
          <button id="pauseLogButton" type="button" onclick="toggleLogPause()">Pause Log</button>
          <button type="button" onclick="downloadCalibration()">Download Calibration CSV</button>
          <button type="button" onclick="showThermalLog()">Show Thermal Log</button>
        </div>

        <!-- Row 2 -->
        <div style="display: flex; gap: 10px; margin-top: 10px; flex-wrap: wrap;">
          <button type="button" onclick="clearCalibration()">Clear Calibration Data</button>
          <button type="button" onclick="clearThermalLog()">Clear Thermal Log</button>
          <div style="display: flex; align-items: center; gap: 10px;">
            <label for="pwmTest">🧪 PWM Test:</label>
            <input type="range" id="pwmTest" min="0" max="100" value="0" oninput="updatePWMValue(this.value)">
            <span id="pwmValue">0%</span>
            <button id="pwmButton" onclick="sendPWMValue()" type="button">Apply</button>
          </div>
        </div>
      </div>
      <div id="log">Loading...</div>
      <div id="dataModal">
        <div id="dataModalBox">
          <div id="dataModalHeader">
            <h3 id="dataModalTitle"></h3>
            <button id="dataModalClose" onclick="closeDataModal()">&times;</button>
          </div>
          <div id="dataModalContent"></div>
          <div id="dataModalButtons">
            <button id="copyBtn" onclick="copyModalData()">Copy to Clipboard</button>
            <button onclick="closeDataModal()">Close</button>
          </div>
        </div>
      </div>
      </body></html>
    )rawliteral";
    server.send(200, "text/html; charset=UTF-8", html);
  });

  // ---- JSON/status and control endpoints ----

  server.on("/status.json", []() {
    // Get cached values updated by background task
    float cached_td_local, cached_spread_local;
    String cached_calFileSize_local, cached_storageInfo_local;
    int cached_rssi_local;
    
    if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(5))) {
      cached_td_local = cached_td;
      cached_spread_local = cached_spread;
      cached_calFileSize_local = cached_calFileSize;
      cached_storageInfo_local = cached_storageInfo;
      cached_rssi_local = cached_rssi;
      xSemaphoreGive(cacheMutex);
    } else {
      // Fallback values if mutex fails
      cached_td_local = NAN;
      cached_spread_local = NAN;
      cached_calFileSize_local = "0 KB";
      cached_storageInfo_local = "0/0 KB";
      cached_rssi_local = -99;
    }

    // Fast path - just get current values
    float t, h;
    readSensorValues(t, h);

    float delta = glassTemp - cached_td_local;  // Tg - Td (glass temp above dew point)
    int pwmPercent = pwm * 100 / 255;
    String statusTime = getStatusTimeString();

    // Simplified JSON response - avoid heavy String operations
    String json = "{";
    json += "\"ambient\":\"" + String(t, 1) + "\",";
    json += "\"humidity\":\"" + String(h, 1) + "\",";
    json += "\"dewpoint\":\"" + (isnan(cached_td_local) ? "NaN" : String(cached_td_local, 1)) + "\",";
    json += "\"dewspread\":\"" + (isnan(cached_spread_local) ? "NaN" : String(cached_spread_local, 1)) + "\",";
    json += "\"glass\":\"" + String(glassTemp, 1) + "\",";
    json += "\"delta\":\"" + String(delta, 1) + "\",";
    json += "\"heater\":" + String(heaterEnabled ? "true" : "false") + ",";
    json += "\"pwm\":" + String(pwmPercent) + ",";
    json += "\"calibrating\":" + String(calibrating ? "true" : "false") + ",";
    json += "\"calibrationFileSize\":\"" + cached_calFileSize_local + "\",";
    json += "\"storageInfo\":\"" + cached_storageInfo_local + "\",";
    json += "\"version\":\"" + String(DEVICE_VERSION) + "\",";
    json += "\"dewSpreadThreshold\":\"" + String(dewSpreadThreshold, 1) + "\",";
    json += "\"dewSpreadHysteresis\":\"" + String(dewSpreadHysteresis, 1) + "\",";
    json += "\"time\":\"" + statusTime + "\",";
    json += "\"wifiSignal\":" + String(cached_rssi_local);
    json += "}";
    
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
    server.send(200, "application/json; charset=UTF-8", json);
  });

  server.on("/log", []() {
    if (server.hasArg("full")) {
      // Full log requested explicitly
      server.send(200, "text/plain", logBuffer);
    } else {
      // Send only recent log entries to reduce network load
      String recentLog = "";
      int bufferLen = logBuffer.length();
      
      if (bufferLen > 3072) {  // 3KB threshold
        // Find the last 2KB but ensure we start at a complete line
        int startPos = bufferLen - 2048;
        
        // Yield during string processing to allow system message pump
        yield();
        
        // Find the next newline to avoid cutting lines in half
        int nextNewline = logBuffer.indexOf('\n', startPos);
        if (nextNewline != -1 && nextNewline < bufferLen - 1) {
          startPos = nextNewline + 1;  // Start after the newline
        }
        
        yield(); // Yield before substring operation
        recentLog = logBuffer.substring(startPos);
      } else {
        // Buffer is small enough, send it all
        recentLog = logBuffer;
      }
      
      server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
      server.sendHeader("Pragma", "no-cache");
      server.sendHeader("Expires", "0");
      server.send(200, "text/plain", recentLog);
    }
  });

  server.on("/toggle", HTTP_POST, []() {
    heaterEnabled = !heaterEnabled;
    if (heaterEnabled) sendLog("🔌 Heater ENABLED");
    else sendLog("🛑 Heater DISABLED");
    saveConfig();
    server.send(200);
  });

  server.on("/calibrate", HTTP_POST, []() {
    calibrating = !calibrating;
    calCount = 0;
    calResetRequested = true;  // signal updateCalibration() to reset its statics

    if (calibrating) sendLog("🧪 Calibration STARTED — sampling every 30 sec: [time, ambient, V, PWM %]");
    else sendLog("✅ Calibration STOPPED");
    server.send(200);
  });

  server.on("/settings", HTTP_POST, []() {
    bool wifiChanged = false;
    String oldSSID = wifiSSID;
    String oldPass = wifiPass;
    
    if (server.hasArg("delta"))        targetDelta      = server.arg("delta").toFloat();
    if (server.hasArg("dewspread"))    dewSpreadThreshold = server.arg("dewspread").toFloat();
    if (server.hasArg("dewhyst"))      dewSpreadHysteresis = server.arg("dewhyst").toFloat();
    if (server.hasArg("ssid"))         wifiSSID         = server.arg("ssid");
    // Only update password if user actually typed a new one
    if (server.hasArg("password") && server.arg("password").length() > 0) {
      wifiPass = server.arg("password");
    }
    if (server.hasArg("timezoneRule")) timezoneRule     = server.arg("timezoneRule");

    // Check if WiFi credentials changed
    if (oldSSID != wifiSSID || oldPass != wifiPass) {
      wifiChanged = true;
    }

    sendLog("⚙️ Settings updated");
    saveConfig();
    server.sendHeader("Location", "/");
    server.send(303);
    
    // Restart only if WiFi credentials changed
    if (wifiChanged) {
      sendLog("📡 WiFi credentials changed - restarting...");
      delay(500);  // Let response finish sending
      ESP.restart();  // Clean restart to apply new WiFi settings
    }
  });

  server.on("/pwmtest", HTTP_POST, []() {
    if (!pwmTest) {
      if (server.hasArg("pwm")) {
        int testVal = server.arg("pwm").toInt();
        pwm = map(constrain(testVal, 0, 100), 0, 100, 0, 255);
        analogWrite(PWM_PIN, pwm);
        pwmTest = true;
        sendLog("🧪 Manual PWM test: STARTED → " + String(testVal) + "% → PWM=" + String(pwm));
      }
    } else {
      pwmTest = false;
      pwm = 0;
      analogWrite(PWM_PIN, pwm);
      sendLog("🛑 Manual PWM test: STOPPED");
    }
    server.send(200);
  });

  server.on("/clearlog", HTTP_POST, []() {
    logBuffer = "🧹 Log cleared.\n";
    server.send(200);
  });

  server.on("/clearcal", HTTP_POST, []() {
    if (SPIFFS.exists(CALIBRATION_FILE)) {
      SPIFFS.remove(CALIBRATION_FILE);
      sendLog("🧹 Calibration data cleared.");
    } else {
      sendLog("ℹ️ No calibration data to clear.");
    }
    server.send(200);
  });

  server.on("/clearlog_thermal", HTTP_POST, []() {
    if (SPIFFS.exists(LOG_FILE)) {
      SPIFFS.remove(LOG_FILE);
      sendLog("🧹 Thermal log cleared.");
    } else {
      sendLog("ℹ️ No thermal log to clear.");
    }
    server.send(200);
  });

  server.on("/downloadcal", HTTP_GET, []() {
    if (SPIFFS.exists(CALIBRATION_FILE)) {
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "Calibration file not found");
    }
  });

  server.on("/downloadlog", HTTP_GET, []() {
    if (SPIFFS.exists(LOG_FILE)) {
      File f = SPIFFS.open(LOG_FILE, FILE_READ);
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "No thermal log found");
    }
  });

  server.begin();
  sendLog("🌐 Web server started");
}

void weatherTask(void* parameter) {
  sendLog("🧵 Weather task started on core " + String(xPortGetCoreID()));

  float outTemp = NAN, outHumidity = NAN;
  for (;;) {
    if (sensorSource == SOURCE_WEATHER) {
      bool ok = fetchOutdoorWeather(&outTemp, &outHumidity);
      if (ok) {
        writeSensorValues(outTemp, outHumidity);
      } else {
        sendLog("⚠️ Weather fetch failed.");
      }
    }
    delay(WEATHER_POLL_INTERVAL_MS);
  }
}

// Sensor task for AHT20
void sensorTask_AHT20(void* parameter) {
  sendLog("🧵 Sensor task (AHT20) started on core " + String(xPortGetCoreID()));
  sendLog("🧪 SDA = " + String(I2C_SDA) + ", SCL = " + String(I2C_SCL));

  I2CBus.begin(I2C_SDA, I2C_SCL);
  delay(30);  // Increased delay for sensor stabilization

  if (!aht.begin(&I2CBus)) {
    sendLog("❌ AHT20 init failed - running I2C bus scan...");
    
    // Scan I2C bus to see what's present
    bool foundAny = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      I2CBus.beginTransmission(addr);
      if (I2CBus.endTransmission() == 0) {
        sendLog("🔍 I2C device found at 0x" + String(addr, HEX));
        foundAny = true;
      }
    }
    
    if (!foundAny) {
      sendLog("❌ No I2C devices found on bus - check wiring/power");
    } else {
      sendLog("⚠️ I2C bus active but AHT20 not responding at 0x38");
    }
    
    vTaskDelete(NULL);
    return;
  }

  sendLog("✅ AHT20 initialized");

  for (;;) {
    sensors_event_t h, t;
    aht.getEvent(&h, &t);

    if (!isnan(t.temperature) && !isnan(h.relative_humidity)) {
      writeSensorValues(t.temperature, h.relative_humidity);
    } else {
      sendLog("⚠️ AHT20 returned bad data, trying soft reset...");
      I2CBus.beginTransmission(0x38);
      I2CBus.write(0xBA);  // Soft reset
      I2CBus.endTransmission();
      delay(20);
      aht.begin(&I2CBus);
    }
    delay(2000);
  }
}

// Sensor task for SHT40
void sensorTask_SHT40(void* parameter) {
  sendLog("🧵 Sensor task (SHT40) started on core " + String(xPortGetCoreID()));
  sendLog("🧪 SDA = " + String(I2C_SDA) + ", SCL = " + String(I2C_SCL));

  I2CBus.begin(I2C_SDA, I2C_SCL);
  delay(30);  // Increased delay for sensor stabilization

  if (!sht4.begin(&I2CBus)) {
    sendLog("❌ SHT40 init failed - running I2C bus scan...");
    
    // Scan I2C bus to see what's present
    bool foundAny = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      I2CBus.beginTransmission(addr);
      if (I2CBus.endTransmission() == 0) {
        sendLog("🔍 I2C device found at 0x" + String(addr, HEX));
        foundAny = true;
      }
    }
    
    if (!foundAny) {
      sendLog("❌ No I2C devices found on bus - check wiring/power");
    } else {
      sendLog("⚠️ I2C bus active but SHT40 not responding at 0x44");
    }
    
    // Fallback to weather station
    sendLog("🔄 Falling back to weather station for temperature/humidity data");
    sensorSource = SOURCE_WEATHER;
    xTaskCreatePinnedToCore(weatherTask, "WeatherTask", 8192, NULL, 1, NULL, 0);
    sendLog("📡 Weather task created as fallback");
    
    vTaskDelete(NULL);
    return;
  }

  sendLog("✅ SHT40 initialized");

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  for (;;) {
    sensors_event_t humidity_event, temp_event;
    sht4.getEvent(&humidity_event, &temp_event);

    if (!isnan(temp_event.temperature) && !isnan(humidity_event.relative_humidity)) {
      writeSensorValues(temp_event.temperature, humidity_event.relative_humidity);
    } else {
      sendLog("⚠️ SHT40 returned bad data");
    }
    
    // Yield to other tasks more frequently
    yield();
    delay(3000);  // Longer delay to reduce I2C traffic
  }
}

// Control task - Background: Heater control, calibration
void controlTask(void* parameter) {
  sendLog("🧵 Control task started on core " + String(xPortGetCoreID()));

  unsigned long lastUpdate = 0;
  unsigned long lastCalibrationUpdate = 0;
  unsigned long lastCacheUpdate_local = 0;
  
  // Wait for sensor to stabilize
  delay(2000);
  
  // Force immediate cache update on startup, then every 2 minutes
  lastCacheUpdate_local = millis() - RSSI_UPDATE_INTERVAL_MS;
  
  for (;;) {
    unsigned long now = millis();
    
    // Main heater control loop every second
    if (now - lastUpdate >= APP_UPDATE_INTERVAL_MS) {
      lastUpdate = now;
      
#if SIMULATE_HARDWARE
      simulateHardware();
#endif
      updateGlassTemp();
      updateHeaterControl();
    }
    
    // Calibration (interval matches CAL_SAMPLE_INTERVAL_MS)
    if (now - lastCalibrationUpdate >= CALIBRATION_UPDATE_INTERVAL_MS) {
      lastCalibrationUpdate = now;
      updateCalibration();
    }
    
    // Thermal lag logging every 10 minutes
    static unsigned long lastThermalLog = 0;
    if (now - lastThermalLog >= THERMAL_LOG_INTERVAL_MS) {
      lastThermalLog = now;
      logThermalLagInfo();
    }
    
    // Update cache every 2 minutes
    if (now - lastCacheUpdate_local >= RSSI_UPDATE_INTERVAL_MS) {
      lastCacheUpdate_local = now;
      
      // Update dew point calculations
      float t, h;
      readSensorValues(t, h);
      
      float td = dewPointC(t, h);
      float spread = isnan(td) ? NAN : (t - td);
      
      yield();
      String calFileSize = getCalibrationFileSizeKB(CALIBRATION_FILE);
      yield();
      String storageInfo = getStorageUsedSummary();
      int rssi = WiFi.RSSI();
      
      if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(1))) {
        cached_td = td;
        cached_spread = spread;
        cached_calFileSize = calFileSize;
        cached_storageInfo = storageInfo;
        cached_rssi = rssi;
        lastCacheUpdate = millis();
        xSemaphoreGive(cacheMutex);
      }
    }
    
    // Control task cycle time
    delay(CONTROL_TASK_DELAY_MS);
  }
}


// ------------------------------------------------------------
// Auto-reconnect + mDNS reinit
// ------------------------------------------------------------
// Handles background Wi-Fi monitoring and gentle recovery if connection is lost
void handleWiFiReconnect() {
  static unsigned long lastReconnectAttempt = 0;
  static unsigned long offlineSince = 0;
  static bool wasDisconnected = false;

  wifi_mode_t mode = WiFi.getMode();
  wl_status_t status = WiFi.status();

  bool haveCreds = !wifiSSID.isEmpty();   // we know user wants STA if SSID configured
  if (!haveCreds) return;

  // While in AP fallback, periodically scan for the target SSID and connect when found
  if (apFallbackActive) {
    static unsigned long lastScanTime = 0;
    static bool scanInProgress = false;
    unsigned long nowMs = millis();

    // Start a new async scan every AP_STA_SCAN_INTERVAL_MS
    if (!scanInProgress && (nowMs - lastScanTime >= AP_STA_SCAN_INTERVAL_MS)) {
      sendLog("🔍 AP mode: scanning for SSID '" + wifiSSID + "'...");
      if (WiFi.getMode() == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);  // STA interface required to initiate a scan
      }
      WiFi.scanNetworks(true);  // async, non-blocking
      scanInProgress = true;
      lastScanTime = nowMs;
      return;
    }

    if (scanInProgress) {
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) return;  // still scanning

      scanInProgress = false;
      bool found = false;

      if (n >= 0) {
        for (int i = 0; i < n; i++) {
          if (WiFi.SSID(i) == wifiSSID) {
            found = true;
            sendLog("📶 SSID '" + wifiSSID + "' visible (RSSI " + String(WiFi.RSSI(i)) + " dBm) — attempting STA connection...");
            break;
          }
        }
        WiFi.scanDelete();
      } else {
        sendLog("⚠️ WiFi scan error (" + String(n) + ") — staying in AP mode");
        if (WiFi.getMode() == WIFI_AP_STA) WiFi.mode(WIFI_AP);  // revert to pure AP
        return;
      }

      if (found) {
        WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
        apFallbackActive = false;  // let reconnect logic manage the connection attempt
        wasDisconnected = true;    // trigger post-connect recovery when STA succeeds
        offlineSince = nowMs;
        lastReconnectAttempt = nowMs;
        return;
      } else {
        sendLog("📡 SSID not found — staying in AP mode (next scan in " + String(AP_STA_SCAN_INTERVAL_MS / 1000) + "s)");
        if (WiFi.getMode() == WIFI_AP_STA) WiFi.mode(WIFI_AP);  // revert to pure AP
        return;
      }
    }

    return;  // AP fallback active, no scan due yet
  }

  // ---- Case 1: STA is connected ----
  if (status == WL_CONNECTED) {
    if (wasDisconnected) {
      wasDisconnected = false;
      offlineSince = 0;

      sendLog("✅ Wi-Fi reconnected: " + WiFi.localIP().toString());
      logWiFiStatus("WiFi");
      apFallbackActive = false;  // Clear fallback flag on successful connection

      // If we connected while in AP_STA mode (scanned and found SSID), drop the AP
      if (WiFi.getMode() == WIFI_AP_STA) {
        sendLog("📡 Switching from AP_STA to STA — AP no longer needed");
        WiFi.mode(WIFI_STA);
        delay(200);
      }

      delay(500); // give stack time to stabilise

      if (!MDNS.begin("DewController")) {
        sendLog("❌ Failed to restart mDNS responder!");
      } else {
        sendLog("✅ mDNS responder restarted as DewController.local");
        yield();
      }

      server.stop();
      delay(200);
      setupWebServer();
      sendLog("🌐 Web server rebound to Wi-Fi interface");

      applyLocalTimezone();

      if (!ntpInitialized) {
        setupNTP();
        ntpInitialized = true;
      } else {
        sendLog("⏱ NTP already active — skipping reinit");
      }
      
      // Immediately attempt NTP sync after reconnection
      handleNTP(true);
    }
    return;
  }

  // ---- Case 2: Not connected ----
  if (!wasDisconnected) {
    wasDisconnected = true;
    offlineSince = millis();
    sendLog("⚠️ Wi-Fi disconnected, will attempt reconnect...");
    logWiFiStatus("WiFi");
  }

  unsigned long now = millis();
  const unsigned long reconnectIntervalMs = 10000;
  if (now - lastReconnectAttempt < reconnectIntervalMs) return;
  lastReconnectAttempt = now;

  // Check current status - don't interrupt if connection is in progress
  if (status == WL_IDLE_STATUS || status == WL_DISCONNECTED) {
    // Connection attempt finished but failed - safe to retry
    if (mode == WIFI_AP) {
      sendLog("🔁 Retrying STA connection from AP mode...");
      WiFi.mode(WIFI_AP_STA);
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    } else {
      sendLog("🔁 Retrying WiFi connection...");
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
  }
  else if (status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED) {
    // Hard failure states - only after prolonged offline do a full reset
    const unsigned long offlineResetMs = 90000; // 90s
    if (offlineSince && (now - offlineSince > offlineResetMs)) {
      sendLog("🧩 WiFi offline for >90s — performing hard reset...");
      WiFi.disconnect(true, true);
      delay(200);
      WiFi.mode(WIFI_STA);
      
      esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      if (netif) {
        esp_netif_dhcpc_stop(netif);
        delay(50);
        esp_netif_set_hostname(netif, "DewController");
        esp_netif_dhcpc_start(netif);
      }
      WiFi.setHostname("DewController");
      
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
      offlineSince = now;  // Reset timer after hard reset
    } else {
      // Try again without disrupting the stack
      sendLog("🔁 Retrying after connection failure...");
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
  }
  // else: status is WL_SCAN_COMPLETED or other transient state - let it continue
}

void handleLEDStatus() {
  static unsigned long lastLED = 0;
  static bool ledState = false;
  static int currentLedMode = -1;

  if (WiFi.getMode() == WIFI_AP) {
    if (currentLedMode != 2) {
      currentLedMode = 2;
      lastLED = millis();
      ledState = false;
      digitalWrite(LED_PIN, ledState);
    }
    if (millis() - lastLED > 500) {
      lastLED = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else if (WiFi.status() == WL_CONNECTED) {
    if (currentLedMode != 1) {
      currentLedMode = 1;
      digitalWrite(LED_PIN, ON);
    }
  } else {
    if (currentLedMode != 0) {
      currentLedMode = 0;
      digitalWrite(LED_PIN, OFF);
    }
  }
}

#if SIMULATE_HARDWARE
void simulateHardware() {
  ambientTemp = 17.0 + sin(millis() / 12000.0);
  humidity = 78.0 + sin(millis() / 8000.0);
}
#endif

void updateHeaterControl() {
  // Persistent integral for PI
  static float pwmIntegral = 0.0;
  static bool dewGateLatched = false; // on/off hysteresis latch

  if (pwmTest) return;                 // manual test overrides
  if (!heaterEnabled || isnan(glassTemp)) {
    // NOTE: integral resets on disable — on re-enable, output starts from
    // Kp*error only (no integral history). This is intentional to avoid
    // stale integral terms after extended off periods.
    pwm = 0;
    pwmIntegral = 0.0;
    dewGateLatched = false;
    dewGate = false;
    analogWrite(PWM_PIN, pwm);
    return;
  }

  float t, h;
  readSensorValues(t, h);

  // ---- Calculate dew point and control target ----
  float td = dewPointC(t, h);

  if (isnan(td)) {
    // If dew point cannot be computed, fail-safe OFF
    pwm = 0;
    pwmIntegral = 0.0;
    dewGate = false;
    analogWrite(PWM_PIN, pwm);
    return;
  }

  float spread = t - td;  // smaller spread => higher condensation risk (uses mutex-protected h)
  
  // Gate by dew spread with hysteresis
  // ON when spread <= threshold; OFF when spread > threshold + hysteresis
  if (!dewGateLatched && spread <= dewSpreadThreshold) {
    dewGateLatched = true;
  } else if (dewGateLatched && spread > (dewSpreadThreshold + dewSpreadHysteresis)) {
    dewGateLatched = false;
  }
  dewGate = dewGateLatched;  // publish to global for logging

  if (dewGate) {
    // Control glass to be targetDelta °C ABOVE DEW POINT (not ambient)
    float targetGlassTemp = td + targetDelta;
    float error = targetGlassTemp - glassTemp;

    // PI control constants
    const float Kp = 20.0f;   // tune as needed
    const float Ki = 0.02f;
    const int   PWM_MIN = 26;   // ~10% keep-alive floor while dew gate is active
    const int   PWM_MAX = 255;
    const float integralMax = 500.0f;

    // Only integrate when not saturated — prevents windup
    float pwmValueF = Kp * error + Ki * pwmIntegral;
    if (pwmValueF >= PWM_MIN && pwmValueF <= PWM_MAX) {
      pwmIntegral += error;
      if (pwmIntegral > integralMax) pwmIntegral = integralMax;
      if (pwmIntegral < -integralMax) pwmIntegral = -integralMax;
    }

    // Single output computation using (possibly updated) integral
    pwmValueF = Kp * error + Ki * pwmIntegral;
    int pwmValue = constrain(int(pwmValueF), 0, PWM_MAX);

    // Enforce minimum keep-alive power while dew gate is active
    if (pwmValue < PWM_MIN) pwmValue = PWM_MIN;

    pwm = pwmValue;
  } else {
    // Safe spread => heater off
    pwm = 0;
    pwmIntegral = 0.0;
  }

  analogWrite(PWM_PIN, pwm);
}

void updateCalibration() {
  static unsigned long lastCalSampleTime = 0;
  static bool firstCalSample = true;

  // Reset statics when requested by /calibrate handler
  if (calResetRequested) {
    lastCalSampleTime = 0;
    firstCalSample = true;
    calResetRequested = false;
  }

  if (!calibrating) return;
  if (millis() - lastCalSampleTime < CAL_SAMPLE_INTERVAL_MS) return;
  lastCalSampleTime = millis();

  float Ta, hUnused;
  readSensorValues(Ta, hUnused);
  float V = readThermistorVoltage();
  int pwmPercent = pwm * 100 / 255;

  struct tm timeinfo;
  char timeStr[32] = "??";
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  File f = SPIFFS.open(CALIBRATION_FILE, FILE_APPEND);
  if (f) {
    if (firstCalSample) {
      String header = "Sample,Time,Ta,V_Thermistor,PWM%";
      f.println(header);
      sendLog(header);
      firstCalSample = false;
    }

    String logLine = String(calCount + 1) + "," +
                     String(timeStr) + "," +
                     String(Ta, 2) + "," +
                     String(V, 3) + "," +
                     String(pwmPercent);

    f.println(logLine);
    f.close();

    sendLog(logLine);
    ++calCount;
  } else {
    sendLog("❌ Could not open calibration file for writing.");
  }
}

void pruneThermalLog(const struct tm& timeinfo) {
  if (!SPIFFS.exists(LOG_FILE)) {
    // No file yet — just write session header and return
    File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
    if (f) {
      char timeStr[32];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      f.println("--- Session start: " + String(timeStr) + " ---");
      f.println("Time,Ta,Tg,Tg-Ta,dTa/dt_per_hr,Spread,Td,RH,PWM%,Vt,GateOn,TgTarget");
      f.close();
    }
    sendLog("📝 Thermal log created");
    return;
  }

  File check = SPIFFS.open(LOG_FILE, FILE_READ);
  size_t fileSize = check.size();
  check.close();

  if (fileSize > MAX_THERMAL_LOG_SIZE) {
    // Hard cap — keep most recent half from clean line boundary
    File src = SPIFFS.open(LOG_FILE, FILE_READ);
    src.seek(fileSize / 2);
    src.readStringUntil('\n');  // discard partial line
    String kept = src.readString();
    src.close();
    SPIFFS.remove(LOG_FILE);
    File dst = SPIFFS.open(LOG_FILE, FILE_WRITE);
    if (dst) {
      dst.print(kept);
      dst.close();
      sendLog("⚠️ Thermal log exceeded 32KB — oldest half removed");
    }
  } else if (fileSize > LOG_PRUNE_SIZE) {
    // Date-based prune — discard entries older than 2 days
    struct tm cutoffTime = timeinfo;
    cutoffTime.tm_hour = 0;
    cutoffTime.tm_min  = 0;
    cutoffTime.tm_sec  = 0;
    cutoffTime.tm_mday -= 2;
    time_t cutoff = mktime(&cutoffTime);

    File src = SPIFFS.open(LOG_FILE, FILE_READ);
    String kept = "";
    while (src.available()) {
      String line = src.readStringUntil('\n');
      line.trim();
      if (line.isEmpty()) continue;

      // Always keep session separators and header lines
      if (line.startsWith("---") || line.startsWith("Time,")) {
        kept += line + "\n";
        continue;
      }

      // Parse timestamp from start of CSV line
      if (line.length() >= 19) {
        struct tm rowTime = {};
        if (strptime(line.c_str(), "%Y-%m-%d %H:%M:%S", &rowTime)) {
          time_t rowEpoch = mktime(&rowTime);
          if (rowEpoch >= cutoff) {
            kept += line + "\n";
          }
        } else {
          kept += line + "\n";  // unparseable — keep it
        }
      } else {
        kept += line + "\n";  // short line — keep it
      }
    }
    src.close();

    SPIFFS.remove(LOG_FILE);
    File dst = SPIFFS.open(LOG_FILE, FILE_WRITE);
    if (dst) {
      dst.print(kept);
      dst.close();
      sendLog("🔄 Thermal log pruned — entries older than date-2 removed");
    }
  } else {
    sendLog("📝 Thermal log OK (" + String(fileSize / 1024.0, 1) + " KB)");
  }

  // Append session separator with real timestamp
  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (f) {
    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    f.println("--- Session start: " + String(timeStr) + " ---");
    f.println("Time,Ta,Tg,Tg-Ta,dTa/dt_per_hr,Spread,Td,RH,PWM%,Vt,GateOn,TgTarget");
    f.close();
  }
  sendLog("📝 Thermal log session started");
}

void handleNTP(bool forceCheck) {
  static bool timeSynced = false;

  // Force check on reconnection, otherwise only check until first successful sync
  if (!forceCheck && timeSynced) return;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
    sendLog(String("✅ NTP time synchronized: ") + buf);
    timeSynced = true;  // Stop checking once synced

    // Prune once on first successful NTP sync this boot
    if (!logPruned) {
      logPruned = true;
      pruneThermalLog(timeinfo);
    }
  } 
  else {
    sendLog("⚠️ NTP not yet synchronized, retrying...");
    // Optional: re-init NTP in case config was lost
    if (timezoneRule.isEmpty()) {
      timezoneRule = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
    }
    setenv("TZ", timezoneRule.c_str(), 1);
    tzset();
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  }
}


// ---------------- Setup ----------------
void setup() {
  // Create mutexes ABSOLUTELY FIRST — before any code that could call sendLog()
  i2cMutex = xSemaphoreCreateMutex();
  cacheMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();

  logBuffer = "";
#if DEBUG_MODE 
  Serial.begin(115200);
  delay(1200);
  DEBUG_PRINT("🔍 Serial started at 115200 baud");
#endif

  sendLog("🚀 Sketch started");

  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 1000);
  analogWrite(PWM_PIN, 0);

  SPIFFS.begin(true);
  loadConfig();
  applyLocalTimezone();
  setupWiFi();
  
  sendLog("🏗 Initializing architecture...");
  sendLog("   Main loop: Web Server, WiFi, mDNS, NTP (foreground)");
  sendLog("   Background tasks: Sensors, Heater Control, Calibration");
  delay(100);
  
  // Background: Sensor tasks
  if (sensorSource == SOURCE_SHT40) {
    xTaskCreatePinnedToCore(sensorTask_SHT40, "SensorTask_SHT40", 8192, NULL, 1, &sensorTaskHandle, 0);
    sendLog("📡 SHT40 sensor task created");
  } else if (sensorSource == SOURCE_WEATHER) {
    xTaskCreatePinnedToCore(weatherTask, "WeatherTask", 8192, NULL, 1, NULL, 0);
    sendLog("📡 Weather task created");
  } else {  // SOURCE_AHT20
    xTaskCreatePinnedToCore(sensorTask_AHT20, "SensorTask_AHT20", 8192, NULL, 1, &sensorTaskHandle, 0);
    sendLog("📡 AHT20 sensor task created");
  }
  delay(100);  // Wait for sensor task to start and initialize
  
  // Background: Heater control task
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 8192, NULL, 1, &controlTaskHandle, 0);
  sendLog("🔥 Heater control task created");
  
  delay(100);  // Let control task start and log
  sendLog("✅ Architecture initialized - Web GUI in main loop");
  delay(50);
}

// ---------------- Loop ----------------
void loop() {
  // Main loop: Web server and network operations (foreground)
  static unsigned long lastWiFiCheck = 0;
  static unsigned long lastNTPCheck = 0;
  static unsigned long lastHealthCheck = 0;
  
  unsigned long now = millis();
  
  // Handle web requests - highest priority for responsive GUI
  server.handleClient();
  yield();
  
  // LED status indication
  handleLEDStatus();
  
  // WiFi reconnect every 10 seconds
  if (now - lastWiFiCheck >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWiFiCheck = now;
    handleWiFiReconnect();
  }
  
  // NTP sync every 2 minutes
  if (now - lastNTPCheck >= NTP_CHECK_INTERVAL_MS) {
    lastNTPCheck = now;
    handleNTP();
  }
  
  // Monitor background task health every 60 seconds
  if (now - lastHealthCheck > 60000) {
    lastHealthCheck = now;
    
    // Check if control task is still running
    if (controlTaskHandle && eTaskGetState(controlTaskHandle) == eDeleted) {
      sendLog("⚠️ Control task died - system restart required");
      ESP.restart();
    }
    
    // Check if sensor task is still running
    if (sensorTaskHandle && eTaskGetState(sensorTaskHandle) == eDeleted) {
      sendLog("⚠️ Sensor task died - system restart required");
      ESP.restart();
    }
  }
  
  // Fast loop cycle for responsive web interface
  delay(MAIN_LOOP_DELAY_MS);
}
