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
#include <ESPmDNS.h>
#include "esp_log.h"
#include <time.h>
#include <HTTPClient.h>
#include <math.h>

//#define DEBUG_MODE 1

#define DEVICE_VERSION "v3.0.0"
#define DEVICE_NAME "DewHeaterController"
#define SIMULATE_HARDWARE 0
#define CONFIG_FILE "/config.json"
#define CALIBRATION_FILE "/calibration.csv"
#define LOOP_INTERVAL_MS 1000
#define WEATHER_POLL_INTERVAL_MS 5 * 60 * 1000 // 300000 ms or 5 minutes
#define WIFI_STATUS_CHECK_INTERVAL_MS 5000 // 5 seconds - faster disconnect detection

// Main loop timing constants (Web/Network - runs in loop())
#define WIFI_RECONNECT_INTERVAL_MS 10000  // 10 seconds
#define NTP_CHECK_INTERVAL_MS 120000      // 2 minutes
#define RSSI_UPDATE_INTERVAL_MS 120000    // 2 minutes
#define FILE_CACHE_UPDATE_INTERVAL_MS 600000  // 10 minutes
#define MAIN_LOOP_DELAY_MS 25             // Main loop cycle time

// Control task timing constants (Background - runs in task)
#define APP_UPDATE_INTERVAL_MS 1000       // 1 second - heater control loop
#define CALIBRATION_UPDATE_INTERVAL_MS 10000  // 10 seconds
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

// Tracks recent history for dT/dt estimation
#define TA_HISTORY_SIZE 10
#define TA_SAMPLE_INTERVAL_SEC 5

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

float TaHistory[TA_HISTORY_SIZE];
unsigned long lastTaSampleTime = 0;

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

// -------------- Calibration variables --------------
float   lastSampledTemp = -100.0;
int     calCount = 0, pwm = 0;
bool    pwmTest = false;
const unsigned long CAL_SAMPLE_INTERVAL_MS = 30000;  // 30 seconds
unsigned long lastCalSampleTime = 0;
bool    firstCalSample = true;  // header line

const int MAX_LOG_SIZE = 65536;  // 64KB log buffer - keep more startup history
String  logBuffer;

bool ntpInitialized = false;
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


// -------------- Utility --------------

// Smart delay that yields periodically without constant overhead
void delayWithYield(unsigned long ms) {
  unsigned long chunks = ms / 100;
  unsigned long remainder = ms % 100;
  
  for (unsigned long i = 0; i < chunks; i++) {
    delay(100);
    yield();
  }
  
  if (remainder > 0) {
    delay(remainder);
  }
}

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

// Weather API (change as required)
const char* WEATHER_API_URL = "https://api.weather.com/v2/pws/observations/current?stationId=ISYDNEY478&format=json&units=m&apiKey=5356e369de454c6f96e369de450c6f22";

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

// Update ambient temperature history
void updateTaHistory(float newTa) {
  for (int i = 1; i < TA_HISTORY_SIZE; i++) {
    TaHistory[i - 1] = TaHistory[i];
  }
  TaHistory[TA_HISTORY_SIZE - 1] = newTa;
}

// Called every LOOP_INTERVAL_MS
void updateGlassTemp() {
  float t, h;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    t = ambientTemp;
    h = humidity;
    xSemaphoreGive(i2cMutex);
  }

  glassTemp = readGlassTemp();

  // Update ambient history every few seconds
  if (millis() - lastTaSampleTime > TA_SAMPLE_INTERVAL_SEC * 1000UL) {
    updateTaHistory(t);
    lastTaSampleTime = millis();
  }
}

// Log thermal lag periodically - now called less frequently from main loop
void logThermalLagInfo() {
  static float prevTa = 0.0;
  static unsigned long prevTime = 0;

  float Ta = ambientTemp;
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

  sendLog(String("📊 Tg = ") + String(Tg, 2) +
          "°C, Ta = " + String(Ta, 2) +
          "°C, Tg - Ta = " + String(delta, 2) +
          "°C, dTa/dt = " + String(gradientPerHour, 2) + " °C/hour");

  // Extra dew metrics
  float td = dewPointC(Ta, humidity);
  if (!isnan(td)) {
    float spread = Ta - td;
    sendLog(String("💧 Td = ") + String(td, 2) + "°C, "
          + "Spread (Ta-Td) = " + String(spread, 2) + "°C, "
          + "Thresh = " + String(dewSpreadThreshold, 1) + "±" + String(dewSpreadHysteresis, 1));
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
  } else {
    sendLog("⚠️ NTP not yet synchronized (will retry automatically)");
  }
}

void setupWebServer() {
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
          fetch("/showcal", { method: "POST" })
            .then(() => setTimeout(updateLog, 300));
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
              <input name="password" type="password" value=")rawliteral" + wifiPass + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Timezone Rule:</label>
              <input name="timezoneRule" value=")rawliteral" + timezoneRule + R"rawliteral(">
            </div>
            <button type="submit">Update Settings</button>
          </form>
            <p> T_glass = T_ambient + )rawliteral" 
              + String(glassCoeffA, 1) + R"rawliteral(·V² + )rawliteral"
              + String(glassCoeffB, 1) + R"rawliteral(·V + )rawliteral"
              + String(glassCoeffC, 1) + R"rawliteral(</p>
            </p>
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
        </div>

        <!-- Row 2 -->
        <div style="display: flex; gap: 10px; margin-top: 10px; flex-wrap: wrap;">
          <button type="button" onclick="clearCalibration()">Clear Calibration Data</button>
          <div style="display: flex; align-items: center; gap: 10px;">
            <label for="pwmTest">🧪 PWM Test:</label>
            <input type="range" id="pwmTest" min="0" max="100" value="0" oninput="updatePWMValue(this.value)">
            <span id="pwmValue">0%</span>
            <button id="pwmButton" onclick="sendPWMValue()" type="button">Apply</button>
          </div>
        </div>
      </div>
      <div id="log">Loading...</div>
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
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2))) {
      t = ambientTemp;
      h = humidity;
      xSemaphoreGive(i2cMutex);
    } else {
      t = ambientTemp;
      h = humidity;
    }

    float delta = glassTemp - cached_td_local;  // Tg - Td (glass temp above dew point)
    int pwmPercent = pwm * 100 / 255;

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
    lastSampledTemp = -100.0;
    lastCalSampleTime = 0;
    calCount = 0;
    firstCalSample = true;

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
    if (server.hasArg("password"))     wifiPass         = server.arg("password");
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

  server.on("/showcal", HTTP_POST, []() {
    File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
    if (!f) {
      sendLog("❌ No calibration data found.");
    } else {
      sendLog("📈 Calibration Data:");
      int lineCount = 0;
      while (f.available()) {
        String line = f.readStringUntil('\n');
        sendLog("📌 " + line);
        
        // Yield every 10 lines to prevent blocking
        if (++lineCount % 10 == 0) {
          yield();
        }
      }
      f.close();
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

  server.on("/downloadcal", HTTP_GET, []() {
    if (SPIFFS.exists(CALIBRATION_FILE)) {
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "Calibration file not found");
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
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
          ambientTemp = outTemp;
          humidity = outHumidity;
          xSemaphoreGive(i2cMutex);
        }
        // Weather data updated - displayed in main window
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
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
        ambientTemp = t.temperature;
        humidity = h.relative_humidity;
        xSemaphoreGive(i2cMutex);
      }
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
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))) {
        ambientTemp = temp_event.temperature;
        humidity = humidity_event.relative_humidity;
        xSemaphoreGive(i2cMutex);
      }
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
    
    // Calibration every 10 seconds
    if (now - lastCalibrationUpdate >= CALIBRATION_UPDATE_INTERVAL_MS) {
      lastCalibrationUpdate = now;
      updateCalibration();
    }
    
    // Update cache every 2 minutes
    if (now - lastCacheUpdate_local >= RSSI_UPDATE_INTERVAL_MS) {
      lastCacheUpdate_local = now;
      
      // Update dew point calculations
      float t, h;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1))) {
        t = ambientTemp;
        h = humidity;
        xSemaphoreGive(i2cMutex);
      } else {
        t = ambientTemp;
        h = humidity;
      }
      
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

  // Don't retry STA during AP fallback cooldown — let AP mode run undisturbed
  if (apFallbackActive) {
    if (millis() - apFallbackTime < AP_FALLBACK_COOLDOWN_MS) return;
    // Cooldown expired — allow STA retry
    sendLog("🔁 AP fallback cooldown expired — retrying STA connection...");
    apFallbackActive = false;
  }

  // ---- Case 1: STA is connected ----
  if (status == WL_CONNECTED) {
    if (wasDisconnected) {
      wasDisconnected = false;
      offlineSince = 0;

      sendLog("✅ Wi-Fi reconnected: " + WiFi.localIP().toString());
      logWiFiStatus("WiFi");
      apFallbackActive = false;  // Clear fallback flag on successful connection

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

void simulateHardware() {
  ambientTemp = 17.0 + sin(millis() / 12000.0);
  humidity = 78.0 + sin(millis() / 8000.0);
}

void updateHeaterControl() {
  // Persistent integral for PI
  static float pwmIntegral = 0.0;

  if (pwmTest) return;                 // manual test overrides
  if (!heaterEnabled || isnan(glassTemp)) {
    pwm = 0;
    pwmIntegral = 0.0;
    analogWrite(PWM_PIN, pwm);
    return;
  }

  float t;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    t = ambientTemp;
    xSemaphoreGive(i2cMutex);
  } else {
    t = ambientTemp;
  }

  // ---- Calculate dew point and control target ----
  float td = dewPointC(t, humidity);
  bool dewGate = false;
  static bool dewGateLatched = false; // on/off hysteresis latch

  if (isnan(td)) {
    // If dew point cannot be computed, fail-safe OFF
    pwm = 0;
    pwmIntegral = 0.0;
    analogWrite(PWM_PIN, pwm);
    return;
  }

  float spread = t - td;  // smaller spread => higher condensation risk
  
  // Gate by dew spread with hysteresis
  // ON when spread <= threshold; OFF when spread > threshold + hysteresis
  if (!dewGateLatched && spread <= dewSpreadThreshold) {
    dewGateLatched = true;
  } else if (dewGateLatched && spread > (dewSpreadThreshold + dewSpreadHysteresis)) {
    dewGateLatched = false;
  }
  dewGate = dewGateLatched;

  if (dewGate) {
    // Control glass to be targetDelta °C ABOVE DEW POINT (not ambient)
    float targetGlassTemp = td + targetDelta;
    float error = targetGlassTemp - glassTemp;

    // PI control constants
    const float Kp = 50.0f;   // tune as needed
    const float Ki = 0.02f;
    const int   PWM_MIN = 64;
    const int   PWM_MAX = 255;
    const float integralMax = 500.0f;

    // Update integral term
    pwmIntegral += error;

    // Anti-windup clamp
    if (pwmIntegral > integralMax) pwmIntegral = integralMax;
    else if (pwmIntegral < -integralMax) pwmIntegral = -integralMax;

    // Compute PI output
    float pwmValueF = Kp * error + Ki * pwmIntegral;

    // Clamp to allowed range
    int pwmValue = constrain(int(pwmValueF), PWM_MIN, PWM_MAX);

    // If glass already at target, maintain minimum hold for stability
    if (glassTemp >= targetGlassTemp && pwmValue < PWM_MIN) {
      pwmValue = PWM_MIN;
    }

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

  if (!calibrating) return;
  if (millis() - lastCalSampleTime < CAL_SAMPLE_INTERVAL_MS) return;
  lastCalSampleTime = millis();

  float Ta = ambientTemp;
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

// Check if NTP has synced; only once per session
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
  logBuffer = "";
#if DEBUG_MODE 
  Serial.begin(115200);
  delay(1200);
  DEBUG_PRINT("🔍 Serial started at 115200 baud");
#endif

  // Create mutexes FIRST - before any logging or multi-threaded operations
  i2cMutex = xSemaphoreCreateMutex();
  cacheMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();

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
  delayWithYield(100);
  
  // Background: Sensor tasks
  if (sensorSource == SOURCE_SHT40) {
    xTaskCreatePinnedToCore(sensorTask_SHT40, "SensorTask_SHT40", 4096, NULL, 1, &sensorTaskHandle, 0);
    sendLog("📡 SHT40 sensor task created");
  } else if (sensorSource == SOURCE_WEATHER) {
    xTaskCreatePinnedToCore(weatherTask, "WeatherTask", 8192, NULL, 1, NULL, 0);
    sendLog("📡 Weather task created");
  } else {  // SOURCE_AHT20
    xTaskCreatePinnedToCore(sensorTask_AHT20, "SensorTask_AHT20", 4096, NULL, 1, &sensorTaskHandle, 0);
    sendLog("📡 AHT20 sensor task created");
  }
  delayWithYield(100);  // Wait for sensor task to start and initialize
  
  // Background: Heater control task
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, &controlTaskHandle, 0);
  sendLog("🔥 Heater control task created");
  
  delayWithYield(100);  // Let control task start and log
  sendLog("✅ Architecture initialized - Web GUI in main loop");
  delayWithYield(50);
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
