// DewHeaterController - Environment-Driven Dew Control
// Control is based entirely on ambient Ta, RH and computed dew-point spread.
// The ring thermistor is retained ONLY as an over-temperature safety cutoff.
// Board: XIAO ESP32S3 with SHT40 sensor
//
// Algorithm - spread table driven:
//   Use spread = Ta - Td and lookup power% from the configured table.
//   Max Power (%) scales the entire table output.
// Thermistor safety: if ring exceeds RING_TEMP_CUTOFF C, PWM is clamped.

#include <Arduino.h>
#include <Wire.h>
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

#define DEVICE_VERSION        "v5.0.1"
#define DEVICE_NAME           "DewHeaterController"
#define CONFIG_FILE           "/config.json"
#define LOG_FILE              "/thermal.log"
#define LOG_PRUNE_SIZE        16384       // 16 KB - date-based prune threshold
#define MAX_THERMAL_LOG_SIZE  32768       // 32 KB - hard cap, truncates oldest half

// Weather API fallback (if SHT40 absent)
#define WEATHER_API_KEY       "5356e369de454c6f96e369de450c6f22"

// Timing
#define WEATHER_POLL_INTERVAL_MS      300000   // 5 min
#define WIFI_STATUS_CHECK_INTERVAL_MS   5000   // 5 s  - UI poll rate
#define WIFI_RECONNECT_INTERVAL_MS     10000   // 10 s
#define NTP_CHECK_INTERVAL_MS         120000   // 2 min
#define RSSI_UPDATE_INTERVAL_MS       120000   // 2 min
#define MAIN_LOOP_DELAY_MS                25
#define AP_STA_SCAN_INTERVAL_MS        30000   // 30 s
#define APP_UPDATE_INTERVAL_MS          1000   // 1 s  - heater control loop
#define THERMAL_LOG_INTERVAL_MS       600000   // 10 min
#define CONTROL_TASK_DELAY_MS             50

// Hardware
#define I2C_SDA        5
#define I2C_SCL        6
#define THERMISTOR_PIN 3
#define PWM_PIN        7
#define LED_PIN        21
#define ON             LOW
#define OFF            HIGH

// Thermistor coefficients - used ONLY for ring over-temperature safety
// Tg = A*V^2 + B*V + C  (quadratic fit from calibration)
#define GLASSCOEFFA   -23.10f
#define GLASSCOEFFB    61.95f
#define GLASSCOEFFC   -19.20f

// Ring over-temperature safety cutoff default (C)
#define DEFAULT_RING_TEMP_CUTOFF   45.0f

// Default hysteresis applied only when spread is increasing (risk easing)
#define DEFAULT_SPREAD_RISE_HYST_C  0.5f

// Max power cap as a percentage of full PWM.
#define DEFAULT_MAX_PWM_PERCENT     90     // leave 10% headroom - ring at 90% is 20.6 W

// Heater electrical model for UI power reference
#define HEATER_SUPPLY_VOLTAGE       12.0f
#define HEATER_RING_RESISTANCE_OHM   6.3f

// Default WiFi
#define DEFAULT_WIFI_SSID   "AstroNetC925"

#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x) do {} while (0)
#endif

// -- Sensor source ---------------------------------------------------------
enum SensorSource { SOURCE_SHT40, SOURCE_WEATHER };
SensorSource sensorSource = SOURCE_SHT40;

// -- FreeRTOS handles ------------------------------------------------------
SemaphoreHandle_t  i2cMutex;
SemaphoreHandle_t  cacheMutex;
SemaphoreHandle_t  logMutex;
TaskHandle_t       sensorTaskHandle  = NULL;
TaskHandle_t       controlTaskHandle = NULL;

// -- Peripheral objects ----------------------------------------------------
WebServer        server(80);
TwoWire          I2CBus = TwoWire(1);
Adafruit_SHT4x   sht4;
bool             webServerInitialized = false;

// -- Live sensor values (mutex-protected) ---------------------------------
float ambientTemp = 0.0f, humidity = 0.0f;

// -- Control state ---------------------------------------------------------
int   pwm          = 0;
bool  pwmTest      = false;
bool  heaterEnabled = false;

// Phase tracking (published for GUI/log)
enum HeaterPhase { PHASE_OFF, PHASE_PREHEAT, PHASE_ACTIVE, PHASE_SATURATED };
HeaterPhase currentPhase = PHASE_OFF;
String phaseLabel() {
  switch (currentPhase) {
    case PHASE_PREHEAT:   return "Preheat";
    case PHASE_ACTIVE:    return "Active";
    case PHASE_SATURATED: return "Saturated";
    default:              return "Off";
  }
}

// -- Configuration ---------------------------------------------------------
int    maxPwmPercent    = DEFAULT_MAX_PWM_PERCENT;
float  spreadRiseHystC  = DEFAULT_SPREAD_RISE_HYST_C;
float  ringTempCutoff   = DEFAULT_RING_TEMP_CUTOFF;
String wifiSSID         = DEFAULT_WIFI_SSID;
String wifiPass         = "";
String timezoneRule     = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";

// -- NTP / AP state --------------------------------------------------------
bool           ntpInitialized   = false;
bool           logPruned        = false;
bool           apFallbackActive = false;
unsigned long  apFallbackTime   = 0;

// -- Background cache ------------------------------------------------------
float  cached_td      = NAN;
float  cached_spread  = NAN;
int    cached_rssi    = -99;
String cached_storageInfo = "0/0 KB";

// -- In-memory log ---------------------------------------------------------
const int MAX_LOG_SIZE = 65536;
String    logBuffer;

// -- Forward declarations --------------------------------------------------
void setupNTP();
void setupWebServer();
void handleNTP(bool forceCheck = false);
void pruneThermalLog(const struct tm& timeinfo);
void sendLog(const String& msg);

const int SPREAD_POWER_TABLE_SIZE = 6;
float spreadPointC[SPREAD_POWER_TABLE_SIZE] = {4.0f, 3.0f, 2.0f, 1.0f, 0.5f, 0.0f};
int spreadPowerPct[SPREAD_POWER_TABLE_SIZE] = {15, 25, 40, 65, 75, 85};
const float DEFAULT_SPREAD_POINT_C[SPREAD_POWER_TABLE_SIZE] = {4.0f, 3.0f, 2.0f, 1.0f, 0.5f, 0.0f};
const int DEFAULT_SPREAD_POWER_PCT[SPREAD_POWER_TABLE_SIZE] = {15, 25, 40, 65, 75, 85};

void sanitizeSpreadTable() {
  for (int i = 0; i < SPREAD_POWER_TABLE_SIZE; i++) {
    spreadPointC[i] = constrain(spreadPointC[i], 0.0f, 20.0f);
    spreadPowerPct[i] = constrain(spreadPowerPct[i], 0, 100);
  }

  // Keep table ordered by descending spread so lookup interpolation is valid.
  for (int i = 0; i < SPREAD_POWER_TABLE_SIZE - 1; i++) {
    for (int j = 0; j < SPREAD_POWER_TABLE_SIZE - 1 - i; j++) {
      if (spreadPointC[j] < spreadPointC[j + 1]) {
        float sTmp = spreadPointC[j];
        spreadPointC[j] = spreadPointC[j + 1];
        spreadPointC[j + 1] = sTmp;
        int pTmp = spreadPowerPct[j];
        spreadPowerPct[j] = spreadPowerPct[j + 1];
        spreadPowerPct[j + 1] = pTmp;
      }
    }
  }
}

// ==========================================================================
//  UTILITY
// ==========================================================================

String wifiStatusToString(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "Idle";
    case WL_NO_SSID_AVAIL:   return "No SSID available";
    case WL_SCAN_COMPLETED:  return "Scan completed";
    case WL_CONNECTED:       return "Connected";
    case WL_CONNECT_FAILED:  return "Connect failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED:    return "Disconnected";
    default:                 return "Unknown (" + String(s) + ")";
  }
}

void applyLocalTimezone() {
  if (timezoneRule.isEmpty()) timezoneRule = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
  setenv("TZ", timezoneRule.c_str(), 1);
  tzset();
  sendLog("Timezone applied: " + timezoneRule);
}

String getStorageUsedSummary() {
  float used  = SPIFFS.usedBytes()  / 1024.0f;
  float total = SPIFFS.totalBytes() / 1024.0f;
  return String(used, 1) + "/" + String(total, 1) + " KB";
}

String getStatusTimeString() {
  struct tm ti;
  char buf[32];
  if (getLocalTime(&ti, 50)) {
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
    return String(buf);
  }
  return "+" + String(millis() / 1000) + "s";
}

float heaterMaxPowerW() {
  return (HEATER_SUPPLY_VOLTAGE * HEATER_SUPPLY_VOLTAGE) / HEATER_RING_RESISTANCE_OHM;
}

float lookupSpreadTablePowerPct(float spread) {
  if (isnan(spread)) return 0.0f;
  if (spread > spreadPointC[0]) return 0.0f;
  if (spread <= spreadPointC[SPREAD_POWER_TABLE_SIZE - 1])
    return (float)spreadPowerPct[SPREAD_POWER_TABLE_SIZE - 1];

  for (int i = 0; i < SPREAD_POWER_TABLE_SIZE - 1; i++) {
    float sHi = spreadPointC[i];
    float sLo = spreadPointC[i + 1];
    if (spread <= sHi && spread > sLo) {
      float pHi = (float)spreadPowerPct[i];
      float pLo = (float)spreadPowerPct[i + 1];
      float t = (sHi - spread) / (sHi - sLo);
      return pHi + (pLo - pHi) * t;
    }
  }
  return 0.0f;
}

// Magnus formula dew point
float dewPointC(float T, float RH) {
  if (isnan(T) || isnan(RH) || RH <= 0.0f) return NAN;
  const float a = 17.62f, b = 243.12f;
  float g = (a * T) / (b + T) + logf(RH / 100.0f);
  return (b * g) / (a - g);
}

// Thermistor voltage -> ring temperature (safety use only)
float readRingTemp() {
  float V = analogRead(THERMISTOR_PIN) * 3.3f / 4095.0f;
  return (GLASSCOEFFA * V * V) + (GLASSCOEFFB * V) + GLASSCOEFFC;
}

void sendLog(const String& msg) {
  char buf[32];
  struct tm ti;
  if (getLocalTime(&ti, 50)) strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
  else snprintf(buf, sizeof(buf), "+%lus", millis() / 1000);

  String line = "[" + String(buf) + "] " + msg;
  DEBUG_PRINT(line);

  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100))) {
    logBuffer += line + "\n";
    if ((int)logBuffer.length() > MAX_LOG_SIZE)
      logBuffer = logBuffer.substring(logBuffer.length() - MAX_LOG_SIZE);
    xSemaphoreGive(logMutex);
  }
}

// ==========================================================================
//  SENSOR I/O
// ==========================================================================

void writeSensorValues(float t, float h) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    ambientTemp = t; humidity = h;
    xSemaphoreGive(i2cMutex);
  }
}

void readSensorValues(float& t, float& h) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
    t = ambientTemp; h = humidity;
    xSemaphoreGive(i2cMutex);
  } else { t = ambientTemp; h = humidity; }
}

// ==========================================================================
//  WEATHER API FALLBACK
// ==========================================================================

const char* WEATHER_API_URL =
  "https://api.weather.com/v2/pws/observations/current"
  "?stationId=ISYDNEY478&format=json&units=m&apiKey=" WEATHER_API_KEY;

bool fetchOutdoorWeather(float* outTemp, float* outHumidity) {
  HTTPClient http;
  http.begin(WEATHER_API_URL);
  int code = http.GET();
  if (code != 200) {
    sendLog("Weather API HTTP error: " + String(code));
    http.end(); return false;
  }
  String payload = http.getString();
  http.end();

  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, payload)) { sendLog("Weather JSON parse error"); return false; }

  JsonObject obs    = doc["observations"][0];
  JsonObject metric = obs["metric"];
  if (obs.isNull() || metric.isNull()) { sendLog("Weather API missing data"); return false; }

  float t = metric["temp"] | NAN;
  float h = obs["humidity"]  | NAN;
  if (!isnan(t) && !isnan(h)) { *outTemp = t; *outHumidity = h; return true; }
  sendLog("Weather API invalid values");
  return false;
}

// ==========================================================================
//  CONFIG
// ==========================================================================

void saveConfig() {
  StaticJsonDocument<512> doc;
  doc["maxPwmPercent"] = maxPwmPercent;
  doc["spreadRiseHystC"] = spreadRiseHystC;
  doc["ringTempCutoff"] = ringTempCutoff;
  doc["s1"] = spreadPointC[0]; doc["p1"] = spreadPowerPct[0];
  doc["s2"] = spreadPointC[1]; doc["p2"] = spreadPowerPct[1];
  doc["s3"] = spreadPointC[2]; doc["p3"] = spreadPowerPct[2];
  doc["s4"] = spreadPointC[3]; doc["p4"] = spreadPowerPct[3];
  doc["s5"] = spreadPointC[4]; doc["p5"] = spreadPowerPct[4];
  doc["s6"] = spreadPointC[5]; doc["p6"] = spreadPowerPct[5];
  doc["heater"]        = heaterEnabled;
  doc["ssid"]          = wifiSSID;
  doc["password"]      = wifiPass;
  doc["timezoneRule"]  = timezoneRule;

  const char* tmp = "/config.tmp";
  File f = SPIFFS.open(tmp, FILE_WRITE);
  if (!f) { sendLog("Failed to open temp config"); return; }
  size_t n = serializeJson(doc, f);
  f.close();
  if (!n) { sendLog("Config write failed"); SPIFFS.remove(tmp); return; }
  SPIFFS.remove(CONFIG_FILE);
  SPIFFS.rename(tmp, CONFIG_FILE);
  sendLog("Config saved (" + String(n) + " bytes)");
}

void loadConfig() {
  if (!SPIFFS.exists(CONFIG_FILE)) { sendLog("No config - using defaults"); return; }
  File f = SPIFFS.open(CONFIG_FILE, FILE_READ);
  if (!f) { sendLog("Failed to open config"); return; }
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    sendLog("Config parse error: " + String(err.c_str()));
    if (err == DeserializationError::EmptyInput || err == DeserializationError::InvalidInput) {
      SPIFFS.remove(CONFIG_FILE);
      sendLog("Corrupted config deleted - using defaults");
    }
    return;
  }
  maxPwmPercent = doc["maxPwmPercent"] | DEFAULT_MAX_PWM_PERCENT;
  spreadRiseHystC = doc["spreadRiseHystC"] | DEFAULT_SPREAD_RISE_HYST_C;
  ringTempCutoff = doc["ringTempCutoff"] | DEFAULT_RING_TEMP_CUTOFF;
  spreadPointC[0] = doc["s1"] | DEFAULT_SPREAD_POINT_C[0];
  spreadPointC[1] = doc["s2"] | DEFAULT_SPREAD_POINT_C[1];
  spreadPointC[2] = doc["s3"] | DEFAULT_SPREAD_POINT_C[2];
  spreadPointC[3] = doc["s4"] | DEFAULT_SPREAD_POINT_C[3];
  spreadPointC[4] = doc["s5"] | DEFAULT_SPREAD_POINT_C[4];
  spreadPointC[5] = doc["s6"] | DEFAULT_SPREAD_POINT_C[5];
  spreadPowerPct[0] = doc["p1"] | DEFAULT_SPREAD_POWER_PCT[0];
  spreadPowerPct[1] = doc["p2"] | DEFAULT_SPREAD_POWER_PCT[1];
  spreadPowerPct[2] = doc["p3"] | DEFAULT_SPREAD_POWER_PCT[2];
  spreadPowerPct[3] = doc["p4"] | DEFAULT_SPREAD_POWER_PCT[3];
  spreadPowerPct[4] = doc["p5"] | DEFAULT_SPREAD_POWER_PCT[4];
  spreadPowerPct[5] = doc["p6"] | DEFAULT_SPREAD_POWER_PCT[5];

  maxPwmPercent = constrain(maxPwmPercent, 10, 100);
  spreadRiseHystC = constrain(spreadRiseHystC, 0.0f, 3.0f);
  ringTempCutoff = constrain(ringTempCutoff, 20.0f, 90.0f);
  sanitizeSpreadTable();

  heaterEnabled = doc["heater"]       | false;
  wifiSSID      = doc["ssid"]         | "";
  wifiPass      = doc["password"]     | "";
  timezoneRule  = doc["timezoneRule"] | "AEST-10AEDT,M10.1.0/2,M4.1.0/3";

  sendLog("Config loaded");
  sendLog("   Max Power (%): " + String(maxPwmPercent));
  sendLog("   Rise Hysteresis (C): " + String(spreadRiseHystC, 2));
    sendLog("   Table: [" + String(spreadPointC[0], 2) + "->" + String(spreadPowerPct[0]) + "%, " +
      String(spreadPointC[1], 2) + "->" + String(spreadPowerPct[1]) + "%, " +
      String(spreadPointC[2], 2) + "->" + String(spreadPowerPct[2]) + "%, " +
      String(spreadPointC[3], 2) + "->" + String(spreadPowerPct[3]) + "%, " +
      String(spreadPointC[4], 2) + "->" + String(spreadPowerPct[4]) + "%, " +
      String(spreadPointC[5], 2) + "->" + String(spreadPowerPct[5]) + "%]");
  sendLog("   Ring Temp Cutoff: " + String(ringTempCutoff, 1));
  sendLog("   Heater: "       + String(heaterEnabled ? "enabled" : "disabled"));
  sendLog("   WiFi SSID: "    + wifiSSID);
  sendLog("   Timezone: "     + timezoneRule);
}

// ==========================================================================
//  ENVIRONMENT-DRIVEN HEATER CONTROL
//
//  No thermistor feedback for control. Power is a pure function of:
//    spread = Ta - Td   (computed from SHT40 Ta and RH)
//    spread->power table, scaled by Max Power (%)
//
//  Thermistor used only for ring over-temperature safety cutoff.
// ==========================================================================

void updateHeaterControl() {
  static float lastSpread = NAN;

  if (pwmTest) return;  // manual override active

  if (!heaterEnabled) {
    pwm = 0;
    currentPhase = PHASE_OFF;
    analogWrite(PWM_PIN, 0);
    lastSpread = NAN;
    return;
  }

  float Ta, RH;
  readSensorValues(Ta, RH);

  float Td = dewPointC(Ta, RH);
  if (isnan(Td) || isnan(Ta)) {
    // Cannot compute dew point - fail safe OFF
    pwm = 0;
    currentPhase = PHASE_OFF;
    analogWrite(PWM_PIN, 0);
    lastSpread = NAN;
    sendLog("Heater control: invalid sensor data - heater off");
    return;
  }

  float spread = Ta - Td;

  // Apply hysteresis only while spread is rising to avoid rapid power drop-offs.
  float lookupSpread = spread;
  if (!isnan(lastSpread) && spread > lastSpread) {
    lookupSpread = max(0.0f, spread - spreadRiseHystC);
  }
  lastSpread = spread;

  // -- Spread table + max power scaling -----------------------------------
  float tablePct = lookupSpreadTablePowerPct(lookupSpread); // 0-100%
  float effectivePct = tablePct * ((float)maxPwmPercent / 100.0f);
  int targetPWM = (int)roundf((effectivePct / 100.0f) * 255.0f);

  if (tablePct <= 0.0f) currentPhase = PHASE_OFF;
  else if (spread > spreadPointC[2]) currentPhase = PHASE_PREHEAT;
  else if (spread > spreadPointC[SPREAD_POWER_TABLE_SIZE - 2]) currentPhase = PHASE_ACTIVE;
  else currentPhase = PHASE_SATURATED;

  // -- Ring over-temperature safety cutoff --------------------------------
  // Thermistor ONLY used here - not for control target
  float ringTemp = readRingTemp();
  if (!isnan(ringTemp) && ringTemp > ringTempCutoff) {
    // Ring too hot - clamp to 20% regardless of algorithm output
    targetPWM = min(targetPWM, (int)(0.20f * 255.0f));
    sendLog("Ring over-temp (" + String(ringTemp, 1) + "C) - PWM clamped");
  }

  pwm = constrain(targetPWM, 0, 255);
  analogWrite(PWM_PIN, pwm);
}

// ==========================================================================
//  THERMAL LOG  (retained for analysis)
// ==========================================================================

void logThermalData() {
  struct tm ti;
  char timeStr[32] = "??";
  if (getLocalTime(&ti)) strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &ti);

  float Ta, RH;
  readSensorValues(Ta, RH);
  float Td     = dewPointC(Ta, RH);
  float spread = isnan(Td) ? NAN : (Ta - Td);
  float ringT  = readRingTemp();
  int   pwmPct = (int)lroundf(((float)pwm * 100.0f) / 255.0f);

  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (f) {
    char buf[200];
    snprintf(buf, sizeof(buf),
             "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s,%.3f",
             timeStr,
             Ta,
             isnan(Td)     ? 0.0f : Td,
             isnan(spread) ? 0.0f : spread,
             RH,
             ringT,
             pwmPct,
             phaseLabel().c_str(),
             analogRead(THERMISTOR_PIN) * 3.3f / 4095.0f);
    f.println(buf);
    f.close();
  }
}

void pruneThermalLog(const struct tm& timeinfo) {
  if (!SPIFFS.exists(LOG_FILE)) {
    File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
    if (f) {
      char ts[32];
      strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &timeinfo);
      f.println("--- Session start: " + String(ts) + " ---");
      f.println("Time,Ta,Td,Spread,RH,RingTemp,PWM%,Phase,Vt");
      f.close();
    }
    sendLog("Thermal log created");
    return;
  }

  File check = SPIFFS.open(LOG_FILE, FILE_READ);
  size_t sz = check.size();
  check.close();

  if (sz > MAX_THERMAL_LOG_SIZE) {
    File src = SPIFFS.open(LOG_FILE, FILE_READ);
    src.seek(sz / 2);
    src.readStringUntil('\n');
    String kept = src.readString();
    src.close();
    SPIFFS.remove(LOG_FILE);
    File dst = SPIFFS.open(LOG_FILE, FILE_WRITE);
    if (dst) { dst.print(kept); dst.close(); }
    sendLog("Thermal log exceeded 32KB - oldest half removed");
  } else if (sz > LOG_PRUNE_SIZE) {
    struct tm cutoff = timeinfo;
    cutoff.tm_mday -= 2; cutoff.tm_hour = 0; cutoff.tm_min = 0; cutoff.tm_sec = 0;
    time_t cutoffEpoch = mktime(&cutoff);

    File src = SPIFFS.open(LOG_FILE, FILE_READ);
    String kept = "";
    while (src.available()) {
      String line = src.readStringUntil('\n'); line.trim();
      if (line.isEmpty()) continue;
      if (line.startsWith("---") || line.startsWith("Time,")) { kept += line + "\n"; continue; }
      if (line.length() >= 19) {
        struct tm rt = {};
        if (strptime(line.c_str(), "%Y-%m-%d %H:%M:%S", &rt) && mktime(&rt) >= cutoffEpoch)
          kept += line + "\n";
      } else kept += line + "\n";
    }
    src.close();
    SPIFFS.remove(LOG_FILE);
    File dst = SPIFFS.open(LOG_FILE, FILE_WRITE);
    if (dst) { dst.print(kept); dst.close(); }
    sendLog("Thermal log pruned - entries older than 2 days removed");
  } else {
    sendLog("Thermal log OK (" + String(sz / 1024.0f, 1) + " KB)");
  }

  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (f) {
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &timeinfo);
    f.println("--- Session start: " + String(ts) + " ---");
    f.println("Time,Ta,Td,Spread,RH,RingTemp,PWM%,Phase,Vt");
    f.close();
  }
  sendLog("Thermal log session started");
}

// ==========================================================================
//  WiFi
// ==========================================================================

void setupWiFi() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (wifiSSID.isEmpty() || wifiPass.isEmpty()) {
    sendLog("No WiFi credentials - starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DewHeaterSetup");
    sendLog("AP: connect to 'DewHeaterSetup' -> " + WiFi.softAPIP().toString());
    applyLocalTimezone();
    setupWebServer();
    return;
  }

  sendLog("Connecting to: " + wifiSSID);
  WiFi.disconnect(true, true); delay(200);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  for (int i = 0; !netif && i < 10; i++) { delay(100); netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); }
  if (netif) {
    esp_netif_dhcpc_stop(netif); delay(100);
    esp_netif_set_hostname(netif, "DewController");
    esp_netif_dhcpc_start(netif);
  }
  WiFi.setHostname("DewController");
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  delay(500);

  // Wait up to 40 s
  wl_status_t last = WL_IDLE_STATUS;
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    wl_status_t s = WiFi.status();
    if (s != last) { sendLog("Waiting: " + wifiStatusToString(s)); last = s; }
    delay(1000);
  }

  // Grace period 15 s
  if (WiFi.status() != WL_CONNECTED) {
    sendLog("Grace period...");
    unsigned long t0 = millis();
    while (millis() - t0 < 15000 && WiFi.status() != WL_CONNECTED) delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    sendLog("WiFi: " + WiFi.localIP().toString() + " RSSI " + String(WiFi.RSSI()) + " dBm");
    if (!MDNS.begin("DewController")) sendLog("mDNS failed");
    else sendLog("mDNS: DewController.local");
    if (!ntpInitialized) { setupNTP(); ntpInitialized = true; }
    setupWebServer();
    return;
  }

  // AP fallback
  sendLog("WiFi failed - AP fallback");
  apFallbackActive = true; apFallbackTime = millis();
  WiFi.disconnect(true, true); delay(200);
  WiFi.mode(WIFI_OFF); delay(500);
  WiFi.mode(WIFI_AP); delay(200);
  WiFi.softAP("DewHeaterSetup"); delay(500);
  IPAddress ip = WiFi.softAPIP();
  sendLog("AP: 'DewHeaterSetup' -> " + ip.toString());
  applyLocalTimezone();
  setupWebServer();
}

// ==========================================================================
//  NTP
// ==========================================================================

void setupNTP() {
  applyLocalTimezone();
  sendLog("Initialising NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", timezoneRule.c_str(), 1); tzset();
  struct tm ti;
  if (getLocalTime(&ti, 5000)) {
    char buf[64]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &ti);
    sendLog("NTP synced: " + String(buf));
    if (!logPruned) { logPruned = true; pruneThermalLog(ti); }
  } else sendLog("NTP not yet synced");
}

void handleNTP(bool forceCheck) {
  static bool synced = false;
  if (!forceCheck && synced) return;
  struct tm ti;
  if (getLocalTime(&ti)) {
    char buf[64]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &ti);
    sendLog("NTP: " + String(buf));
    synced = true;
    if (!logPruned) { logPruned = true; pruneThermalLog(ti); }
  } else {
    sendLog("NTP retry...");
    setenv("TZ", timezoneRule.c_str(), 1); tzset();
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  }
}

// ==========================================================================
//  WEB SERVER
// ==========================================================================

void setupWebServer() {
  if (webServerInitialized) { server.begin(); sendLog("Web server restarted"); return; }
  webServerInitialized = true;

  // -- Root page -----------------------------------------------------------
  server.on("/", []() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
<title>Dew Heater Controller</title>
<style>
  :root {
    --bg: #0d1117; --panel: #161b22; --border: #30363d;
    --text: #e6edf3; --muted: #8b949e; --accent: #58a6ff;
    --green: #3fb950; --amber: #d29922; --red: #f85149;
    --preheat-col: #d29922; --active-col: #58a6ff; --sat-col: #f85149;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'Courier New', monospace; font-size: 14px; }
  h1 { padding: 12px 16px; border-bottom: 1px solid var(--border); font-size: 1rem; letter-spacing: 2px; text-transform: uppercase; display: flex; justify-content: space-between; align-items: center; }
  h1 span.ver { font-size: 0.7rem; color: var(--muted); }
  #wifi-badge { font-size: 0.75rem; padding: 3px 10px; border-radius: 12px; background: #f8514933; color: var(--red); }
  #wifi-badge.ok { background: #3fb95033; color: var(--green); }

  #layout { display: flex; gap: 0; flex-wrap: wrap; }
  #left { flex: 0 0 220px; border-right: 1px solid var(--border); }
  #right { flex: 1 1 300px; display: flex; flex-direction: column; }

  /* -- Status panel -- */
  .stat-grid { padding: 12px 16px; display: grid; grid-template-columns: 1fr 1fr; gap: 8px; border-bottom: 1px solid var(--border); }
  .stat { background: var(--panel); border: 1px solid var(--border); border-radius: 6px; padding: 8px 10px; }
  .stat .lbl { font-size: 0.65rem; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; }
  .stat .val { font-size: 1.1rem; font-weight: bold; margin-top: 2px; }
  .stat.wide { grid-column: span 2; }
  .stat.phase-off      .val { color: var(--muted); }
  .stat.phase-preheat  .val { color: var(--preheat-col); }
  .stat.phase-active   .val { color: var(--active-col); }
  .stat.phase-saturated .val { color: var(--red); }

  /* -- Phase bar -- */
  #phasebar { margin: 0 16px 12px; height: 6px; border-radius: 3px; background: var(--border); overflow: hidden; }
  #phasefill { height: 100%; width: 0%; border-radius: 3px; transition: width 1s, background 1s; background: var(--muted); }

  /* -- Settings -- */
  #settings { padding: 12px 16px; border-bottom: 1px solid var(--border); }
  #settings h2 { font-size: 0.7rem; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); margin-bottom: 10px; }
  .row { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
  .row label { flex: 0 0 160px; color: var(--muted); font-size: 0.8rem; }
  .row input[type=text], .row input[type=password] { flex: 1; background: var(--bg); border: 1px solid var(--border); color: var(--text); padding: 5px 8px; border-radius: 4px; font-family: inherit; font-size: 0.85rem; }
  .powertable input.num { background: var(--bg); border: 1px solid var(--border); color: var(--text); padding: 5px 8px; border-radius: 4px; font-family: inherit; font-size: 0.85rem; width: 100%; min-width: 0; text-align: right; }
  .row input[type=text]:focus, .row input[type=password]:focus, .powertable input.num:focus { outline: none; border-color: #9aa4af; background: #2a3037; }
  .row input.num { flex: 0 0 82px; width: 82px; text-align: right; }
  .row .hint { min-width: 128px; text-align: right; color: var(--muted); font-size: 0.78rem; }
  .powertable { margin: 10px 0 12px; border: 1px solid var(--border); border-radius: 6px; overflow-x: auto; }
  .powertable table { width: auto; min-width: 208px; margin: 0 auto; border-collapse: collapse; table-layout: fixed; }
  .powertable col.col-spread { width: 72px; }
  .powertable col.col-power  { width: 72px; }
  .powertable col.col-watts  { width: 64px; }
  .powertable th, .powertable td { padding: 5px 6px; border-bottom: 1px solid var(--border); font-size: 0.78rem; white-space: nowrap; }
  .powertable tr:last-child td { border-bottom: none; }
  .powertable th { color: var(--muted); text-align: right; background: #0f141c; }
  .powertable th:nth-child(1), .powertable td:nth-child(1) { text-align: right; }
  .powertable th:nth-child(2), .powertable td:nth-child(2) { text-align: right; }
  .powertable th:nth-child(3), .powertable td:nth-child(3) { text-align: right; }

  /* -- Buttons -- */
  .btn-row { display: flex; gap: 8px; flex-wrap: wrap; padding: 10px 16px; border-bottom: 1px solid var(--border); }
  button { background: var(--panel); border: 1px solid var(--border); color: var(--text); padding: 6px 14px; border-radius: 4px; font-family: inherit; font-size: 0.8rem; cursor: pointer; transition: border-color .15s, color .15s; }
  button:hover { border-color: var(--accent); color: var(--accent); }
  button.active { border-color: var(--green); color: var(--green); }
  button.danger { border-color: var(--red); color: var(--red); }
  #saveBtn { border-color: var(--accent); color: var(--accent); }
  #saveBtn.saved { border-color: var(--green); color: var(--green); }
  #saveBtn:disabled { opacity: 0.85; cursor: default; }

  /* PWM test row */
  #pwm-row { display: flex; align-items: center; gap: 10px; padding: 8px 16px; border-bottom: 1px solid var(--border); }
  #pwm-row label { color: var(--muted); font-size: 0.8rem; }
  #pwmTest { flex: 1; accent-color: var(--red); }
  #pwmVal { min-width: 36px; color: var(--red); font-weight: bold; }

  /* -- Log -- */
  #log-header { display: flex; justify-content: space-between; align-items: center; padding: 6px 16px; border-bottom: 1px solid var(--border); }
  #log-header span { font-size: 0.7rem; text-transform: uppercase; letter-spacing: 1px; color: var(--muted); }
  #log { flex: 1; overflow-y: auto; background: var(--bg); padding: 10px 16px; white-space: pre-wrap; font-size: 0.8rem; line-height: 1.5; color: #8b949e; min-height: 200px; max-height: calc(100vh - 420px); }
  #log .ts { color: #3d4450; }

  /* -- Modal -- */
  .modal { display: none; position: fixed; inset: 0; background: rgba(0,0,0,.7); z-index: 999; justify-content: center; align-items: center; }
  .modal.open { display: flex; }
  .modal-box { background: var(--panel); border: 1px solid var(--border); border-radius: 8px; padding: 20px; width: 85%; max-width: 860px; max-height: 80vh; display: flex; flex-direction: column; }
  .modal-box h3 { font-size: 0.9rem; margin-bottom: 12px; color: var(--accent); }
  .modal-content { flex: 1; overflow-y: auto; white-space: pre-wrap; font-size: 0.78rem; line-height: 1.5; background: var(--bg); border: 1px solid var(--border); padding: 10px; border-radius: 4px; color: var(--muted); }
  .modal-btns { display: flex; gap: 8px; margin-top: 12px; }

  @media (max-width: 600px) { #left { flex: 1 1 100%; border-right: none; border-bottom: 1px solid var(--border); } }
</style>
</head><body>

<h1>
  Dew Heater Controller
  <span class="ver" id="ver"></span>
  <span id="wifi-badge">Disconnected</span>
</h1>

<div id="layout">
  <!-- LEFT: status + settings -->
  <div id="left">
    <div class="stat-grid">
      <div class="stat wide" id="powerBox">
        <div class="lbl">Heater Output</div>
        <div class="val" id="powerVal">-</div>
      </div>
      <div class="stat"><div class="lbl">Ambient</div><div class="val"><span id="Ta">-</span> C</div></div>
      <div class="stat"><div class="lbl">Humidity</div><div class="val"><span id="RH">-</span> %</div></div>
      <div class="stat"><div class="lbl">Dew Point</div><div class="val"><span id="Td">-</span> C</div></div>
      <div class="stat"><div class="lbl">Spread</div><div class="val"><span id="spread">-</span> C</div></div>
      <div class="stat"><div class="lbl">Ring Temp</div><div class="val"><span id="ringTemp">-</span> C</div></div>
      <div class="stat"><div class="lbl">Heater Power</div><div class="val"><span id="pwmPct">-</span> %</div></div>
      <div class="stat wide"><div class="lbl">Storage</div><div class="val" style="font-size:.85rem" id="storage">-</div></div>
    </div>

    <div id="phasebar"><div id="phasefill"></div></div>

    <div class="btn-row">
      <button id="heaterBtn" onclick="toggleHeater()">Enable Heater</button>
    </div>

    <div id="pwm-row">
      <label>PWM Test</label>
      <input type="range" id="pwmTest" min="0" max="100" value="0" oninput="document.getElementById('pwmVal').innerText=this.value+'%'">
      <span id="pwmVal">0%</span>
      <button id="pwmBtn" onclick="sendPWM()">Apply</button>
    </div>

    <div id="settings">
      <h2>Settings</h2>
      <div class="row"><label>SSID</label><input type="text" id="ssid" name="ssid"></div>
      <div class="row"><label>Password</label><input type="password" id="password" placeholder="(unchanged)"></div>
      <div class="row"><label>Timezone Rule</label><input type="text" id="tz" name="tz"></div>
      <div class="row"><label>Max Power (%)</label><input class="num" type="text" id="maxPwmPercent" name="maxPwmPercent"><span class="hint" id="maxPowerInfo">Max Power: 23W</span></div>
      <div class="row"><label>Rise Hysteresis (C)</label><input class="num" type="text" id="spreadRiseHystC" name="spreadRiseHystC"></div>
      <div class="row"><label>Ring Temp Cutoff (C)</label><input class="num" type="text" id="ringTempCutoff" name="ringTempCutoff"></div>

      <div class="powertable">
        <table>
          <colgroup>
            <col class="col-spread">
            <col class="col-power">
            <col class="col-watts">
          </colgroup>
          <thead><tr><th>Spread</th><th>Power %</th><th>Watts</th></tr></thead>
          <tbody>
            <tr><td><input class="num" type="text" id="s1" value="4.0"></td><td><input class="num" type="text" id="p1" value="15"></td><td id="w1">-</td></tr>
            <tr><td><input class="num" type="text" id="s2" value="3.0"></td><td><input class="num" type="text" id="p2" value="25"></td><td id="w2">-</td></tr>
            <tr><td><input class="num" type="text" id="s3" value="2.0"></td><td><input class="num" type="text" id="p3" value="40"></td><td id="w3">-</td></tr>
            <tr><td><input class="num" type="text" id="s4" value="1.0"></td><td><input class="num" type="text" id="p4" value="65"></td><td id="w4">-</td></tr>
            <tr><td><input class="num" type="text" id="s5" value="0.5"></td><td><input class="num" type="text" id="p5" value="75"></td><td id="w5">-</td></tr>
            <tr><td><input class="num" type="text" id="s6" value="0.0"></td><td><input class="num" type="text" id="p6" value="85"></td><td id="w6">-</td></tr>
          </tbody>
        </table>
      </div>
      <button id="saveBtn" onclick="saveSettings()">Save Settings</button>
    </div>
  </div>

  <!-- RIGHT: log -->
  <div id="right">
    <div id="log-header">
      <span>Event Log</span>
      <div style="display:flex;gap:6px">
        <button onclick="showThermalLog()">Thermal Log</button>
        <button onclick="clearLog()">Clear</button>
        <button id="pauseBtn" onclick="togglePause()">Pause</button>
      </div>
    </div>
    <div id="log">Loading...</div>
  </div>
</div>

<!-- Modal -->
<div class="modal" id="modal">
  <div class="modal-box">
    <h3 id="modal-title"></h3>
    <div class="modal-content" id="modal-content"></div>
    <div class="modal-btns">
      <button onclick="copyModal()">Copy</button>
      <button onclick="closeModal()">Close</button>
    </div>
  </div>
</div>

<script>
  let logPaused = false;
  let settingsDirty = false;
  let latestMaxPowerW = 0;

  function updateSaveButtonLabel() {
    const btn = document.getElementById('saveBtn');
    if (!btn || btn.dataset.saving === '1') return;
    btn.innerText = settingsDirty ? 'Unsaved Settings' : 'Save Settings';
  }

  function recalcTableWatts() {
    const maxPct = parseFloat((document.getElementById('maxPwmPercent') || {}).value) || 0;
    const capW = latestMaxPowerW * (maxPct / 100);
    const pctAt = (id) => parseFloat((document.getElementById(id) || {}).value) || 0;
    const setW = (id, pct) => {
      const el = document.getElementById(id);
      if (el) el.innerText = (capW * (pct / 100)).toFixed(1) + 'W';
    };
    setW('w1', pctAt('p1'));
    setW('w2', pctAt('p2'));
    setW('w3', pctAt('p3'));
    setW('w4', pctAt('p4'));
    setW('w5', pctAt('p5'));
    setW('w6', pctAt('p6'));
  }

  function watchSettingsChanges() {
    document.querySelectorAll('#settings input').forEach(el => {
      el.addEventListener('input', () => {
        settingsDirty = true;
        updateSaveButtonLabel();
        recalcTableWatts();
      });
    });
  }

  function updateStatus() {
    const ctrl = new AbortController();
    setTimeout(() => ctrl.abort(), 3000);
    fetch('/status.json', { signal: ctrl.signal })
      .then(r => r.json()).then(d => {
        document.getElementById('ver').innerText  = d.version || '';

        // WiFi badge
        const wb = document.getElementById('wifi-badge');
        if (d.wifiSignal && d.wifiSignal > -200) {
          wb.className = 'ok';
          wb.innerText = d.wifiSignal + ' dBm';
        } else {
          wb.className = '';
          wb.innerText = 'Disconnected';
        }

        document.getElementById('Ta').innerText     = d.ambient;
        document.getElementById('RH').innerText     = d.humidity;
        document.getElementById('Td').innerText     = d.dewpoint;
        document.getElementById('spread').innerText = d.dewspread;
        document.getElementById('ringTemp').innerText = d.ringTemp;
        document.getElementById('pwmPct').innerText = d.pwm;
        document.getElementById('storage').innerText = d.storageInfo;
        document.getElementById('ver').innerText     = d.version;

        // Heater output display (% of capped runtime power and watts)
        const pb = document.getElementById('powerBox');
        const pv = document.getElementById('powerVal');
        const pf = document.getElementById('phasefill');
        const pwmPctFull = parseFloat(d.pwm) || 0;              // % of full hardware power
        const maxPct = parseFloat(d.maxPwmPercent) || 0;        // runtime cap % of full power
        const maxW = parseFloat(d.maxPowerW) || 0;
        const outW = maxW * (pwmPctFull / 100.0);
        const manualPwmTest = !!d.pwmTest;
        // In PWM Test mode, show direct hardware PWM %, not % of runtime cap.
        const outPctRaw = manualPwmTest
          ? pwmPctFull
          : ((maxPct > 0.0) ? (pwmPctFull / maxPct) * 100.0 : 0.0);
        const outPctShown = Math.min(100, Math.max(0, outPctRaw));
        pv.innerText = outPctShown.toFixed(0) + '% / ' + outW.toFixed(1) + 'W';
        pv.style.color = pwmPctFull > 0.0 ? 'var(--green)' : 'var(--muted)';
        pb.className = 'stat wide';

        // Output progress bar tracks % of capped runtime power.
        pf.style.width = outPctShown.toFixed(1) + '%';
        pf.style.background = outPctShown >= 90 ? 'var(--red)' : outPctShown >= 60 ? 'var(--amber)' : 'var(--accent)';

        // Heater button
        const hb = document.getElementById('heaterBtn');
        hb.innerText  = d.heater ? 'Disable Heater' : 'Enable Heater';
        hb.className  = d.heater ? 'active' : '';

        // Keep settings live-synced unless user is actively working in Settings.
        const settingsPanel = document.getElementById('settings');
        const editingSettings = settingsPanel && settingsPanel.contains(document.activeElement);

        if (!editingSettings) {
        const setIfNotFocused = (id, value) => {
          const el = document.getElementById(id);
          if (document.activeElement !== el) {
            el.value = (value === undefined || value === null) ? '' : String(value);
          }
        };
        setIfNotFocused('ssid', d.ssid);
        setIfNotFocused('tz', d.tz);
        setIfNotFocused('maxPwmPercent', d.maxPwmPercent);
        setIfNotFocused('spreadRiseHystC', d.spreadRiseHystC);
        setIfNotFocused('ringTempCutoff', d.ringTempCutoff);
        setIfNotFocused('p1', d.p1);
        setIfNotFocused('s1', d.s1);
        setIfNotFocused('s2', d.s2);
        setIfNotFocused('s3', d.s3);
        setIfNotFocused('s4', d.s4);
        setIfNotFocused('s5', d.s5);
        setIfNotFocused('s6', d.s6);
        setIfNotFocused('p2', d.p2);
        setIfNotFocused('p3', d.p3);
        setIfNotFocused('p4', d.p4);
        setIfNotFocused('p5', d.p5);
        setIfNotFocused('p6', d.p6);
        }

        latestMaxPowerW = parseFloat(d.maxPowerW) || 0;
        document.getElementById('maxPowerInfo').innerText = 'Max Power: ' + Math.round(latestMaxPowerW) + 'W';
        recalcTableWatts();
      }).catch(() => {
        document.getElementById('wifi-badge').className = '';
        document.getElementById('wifi-badge').innerText = 'Disconnected';
      });
  }

  function updateLog() {
    if (logPaused) return;
    fetch('/log').then(r => r.text()).then(t => {
      const box = document.getElementById('log');
      box.innerText = t;
      box.scrollTop = box.scrollHeight;
    });
  }

  function toggleHeater() {
    fetch('/toggle', { method: 'POST' }).then(() => setTimeout(updateStatus, 300));
  }

  function saveSettings() {
    const btn = document.getElementById('saveBtn');
    if (btn.dataset.saving === '1') return;

    const defaultLabel = 'Save Settings';
    btn.dataset.saving = '1';
    btn.disabled = true;
    btn.classList.remove('saved', 'danger');
    btn.innerText = 'Saving...';

    const body = new URLSearchParams({
      ssid:       document.getElementById('ssid').value,
      password:   document.getElementById('password').value,
      tz:         document.getElementById('tz').value,
      maxPwmPercent: document.getElementById('maxPwmPercent').value,
      spreadRiseHystC: document.getElementById('spreadRiseHystC').value,
      ringTempCutoff: document.getElementById('ringTempCutoff').value,
      p1: document.getElementById('p1').value,
      s1: document.getElementById('s1').value,
      s2: document.getElementById('s2').value,
      s3: document.getElementById('s3').value,
      s4: document.getElementById('s4').value,
      s5: document.getElementById('s5').value,
      s6: document.getElementById('s6').value,
      p2: document.getElementById('p2').value,
      p3: document.getElementById('p3').value,
      p4: document.getElementById('p4').value,
      p5: document.getElementById('p5').value,
      p6: document.getElementById('p6').value
    });
    fetch('/settings', { method: 'POST', body })
      .then(r => {
        if (!r.ok) throw new Error('save failed');
        settingsDirty = false;
        btn.classList.add('saved');
        btn.innerText = 'Saved';
        setTimeout(updateStatus, 400);
      })
      .catch(() => {
        settingsDirty = true;
        btn.classList.add('danger');
        btn.innerText = 'Save Failed';
      })
      .finally(() => {
        setTimeout(() => {
          btn.classList.remove('saved', 'danger');
          btn.disabled = false;
          btn.dataset.saving = '0';
          btn.innerText = defaultLabel;
          updateSaveButtonLabel();
        }, 1200);
      });
  }

  function sendPWM() {
    const val = document.getElementById('pwmTest').value;
    const btn = document.getElementById('pwmBtn');
    fetch('/pwmtest', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: 'pwm=' + val })
      .then(() => { btn.innerText = btn.innerText === 'Apply' ? 'Stop' : 'Apply'; });
  }

  function clearLog() {
    fetch('/clearlog', { method: 'POST' }).then(() => document.getElementById('log').innerText = '');
  }

  function togglePause() {
    logPaused = !logPaused;
    document.getElementById('pauseBtn').innerText = logPaused ? 'Resume' : 'Pause';
  }

  function showThermalLog() {
    fetch('/downloadlog').then(r => r.ok ? r.text() : Promise.reject())
      .then(t => showModal('Thermal Log', t))
      .catch(() => showModal('Thermal Log', 'No thermal log found.'));
  }

  function showModal(title, content) {
    document.getElementById('modal-title').innerText   = title;
    document.getElementById('modal-content').innerText = content;
    document.getElementById('modal').classList.add('open');
    setTimeout(() => {
      const el = document.getElementById('modal-content');
      el.scrollTop = el.scrollHeight;
    }, 50);
  }

  function closeModal() { document.getElementById('modal').classList.remove('open'); }

  function copyModal() {
    navigator.clipboard.writeText(document.getElementById('modal-content').innerText)
      .catch(() => {
        const ta = document.createElement('textarea');
        ta.value = document.getElementById('modal-content').innerText;
        document.body.appendChild(ta); ta.select(); document.execCommand('copy'); document.body.removeChild(ta);
      });
  }

  setInterval(() => { updateStatus(); updateLog(); }, )rawliteral"
    + String(WIFI_STATUS_CHECK_INTERVAL_MS) +
    R"rawliteral();
  window.onload = () => { watchSettingsChanges(); updateSaveButtonLabel(); updateStatus(); updateLog(); };
</script>
</body></html>
)rawliteral";
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html; charset=UTF-8", html);
  });

  // -- /status.json --------------------------------------------------------
  server.on("/status.json", []() {
    float t, h, ring;
    float td_l, spread_l;
    int   rssi_l;
    String storage_l;

    readSensorValues(t, h);
    ring = readRingTemp();

    if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(5))) {
      td_l      = cached_td;
      spread_l  = cached_spread;
      rssi_l    = cached_rssi;
      storage_l = cached_storageInfo;
      xSemaphoreGive(cacheMutex);
    } else {
      td_l = spread_l = NAN; rssi_l = -99; storage_l = "0/0 KB";
    }

    int pwmPct = (int)lroundf(((float)pwm * 100.0f) / 255.0f);

    String json = "{";
    json += "\"ambient\":\""     + String(t, 1)                                     + "\",";
    json += "\"humidity\":\""    + String(h, 1)                                     + "\",";
    json += "\"dewpoint\":\""    + (isnan(td_l)     ? "NaN" : String(td_l, 1))     + "\",";
    json += "\"dewspread\":\""   + (isnan(spread_l) ? "NaN" : String(spread_l, 1)) + "\",";
    json += "\"ringTemp\":\""    + (isnan(ring)      ? "NaN" : String(ring, 1))     + "\",";
    json += "\"heater\":"        + String(heaterEnabled ? "true" : "false")         + ",";
    json += "\"pwm\":"           + String(pwmPct)                                   + ",";
    json += "\"pwmTest\":"       + String(pwmTest ? "true" : "false")              + ",";
    json += "\"phase\":\""       + phaseLabel()                                     + "\",";
    json += "\"maxPwmPercent\":\"" + String(maxPwmPercent)                           + "\",";
    json += "\"spreadRiseHystC\":\"" + String(spreadRiseHystC, 2)                    + "\",";
    json += "\"maxPowerW\":\"" + String(heaterMaxPowerW(), 1)                         + "\",";
    json += "\"ringTempCutoff\":\"" + String(ringTempCutoff, 1)                     + "\",";
    json += "\"s1\":\"" + String(spreadPointC[0], 2)                                 + "\",";
    json += "\"p1\":\"" + String(spreadPowerPct[0])                                   + "\",";
    json += "\"s2\":\"" + String(spreadPointC[1], 2)                                 + "\",";
    json += "\"p2\":\"" + String(spreadPowerPct[1])                                   + "\",";
    json += "\"s3\":\"" + String(spreadPointC[2], 2)                                 + "\",";
    json += "\"p3\":\"" + String(spreadPowerPct[2])                                   + "\",";
    json += "\"s4\":\"" + String(spreadPointC[3], 2)                                 + "\",";
    json += "\"p4\":\"" + String(spreadPowerPct[3])                                   + "\",";
    json += "\"s5\":\"" + String(spreadPointC[4], 2)                                 + "\",";
    json += "\"p5\":\"" + String(spreadPowerPct[4])                                   + "\",";
    json += "\"s6\":\"" + String(spreadPointC[5], 2)                                 + "\",";
    json += "\"p6\":\"" + String(spreadPowerPct[5])                                   + "\",";
    json += "\"ssid\":\""        + wifiSSID                                         + "\",";
    json += "\"tz\":\""          + timezoneRule                                     + "\",";
    json += "\"storageInfo\":\"" + storage_l                                        + "\",";
    json += "\"version\":\""     + String(DEVICE_VERSION)                           + "\",";
    json += "\"time\":\""        + getStatusTimeString()                            + "\",";
    json += "\"wifiSignal\":"    + String(rssi_l);
    json += "}";

    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(200, "application/json; charset=UTF-8", json);
  });

  // -- /log ----------------------------------------------------------------
  server.on("/log", []() {
    String out;
    int len = logBuffer.length();
    if (len > 3072) {
      int start = logBuffer.indexOf('\n', len - 2048);
      out = (start != -1) ? logBuffer.substring(start + 1) : logBuffer;
    } else out = logBuffer;
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(200, "text/plain", out);
  });

  // -- Control endpoints ----------------------------------------------------
  server.on("/toggle", HTTP_POST, []() {
    heaterEnabled = !heaterEnabled;
    sendLog(heaterEnabled ? "Heater ENABLED" : "Heater DISABLED");
    saveConfig();
    server.send(200);
  });

  server.on("/settings", HTTP_POST, []() {
    bool wifiChanged = false;
    String oldSSID = wifiSSID, oldPass = wifiPass;

    if (server.hasArg("maxPwmPercent")) {
      maxPwmPercent = constrain(server.arg("maxPwmPercent").toInt(), 10, 100);
    }
    if (server.hasArg("spreadRiseHystC")) {
      spreadRiseHystC = constrain(server.arg("spreadRiseHystC").toFloat(), 0.0f, 3.0f);
    }
    if (server.hasArg("ringTempCutoff")) {
      ringTempCutoff = constrain(server.arg("ringTempCutoff").toFloat(), 20.0f, 90.0f);
    }
    if (server.hasArg("ssid"))     wifiSSID     = server.arg("ssid");
    if (server.hasArg("password") && server.arg("password").length() > 0)
      wifiPass = server.arg("password");
    if (server.hasArg("tz"))       timezoneRule = server.arg("tz");
    if (server.hasArg("s1")) spreadPointC[0] = constrain(server.arg("s1").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("s2")) spreadPointC[1] = constrain(server.arg("s2").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("s3")) spreadPointC[2] = constrain(server.arg("s3").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("s4")) spreadPointC[3] = constrain(server.arg("s4").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("s5")) spreadPointC[4] = constrain(server.arg("s5").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("s6")) spreadPointC[5] = constrain(server.arg("s6").toFloat(), 0.0f, 20.0f);
    if (server.hasArg("p1")) spreadPowerPct[0] = constrain(server.arg("p1").toInt(), 0, 100);
    if (server.hasArg("p2")) spreadPowerPct[1] = constrain(server.arg("p2").toInt(), 0, 100);
    if (server.hasArg("p3")) spreadPowerPct[2] = constrain(server.arg("p3").toInt(), 0, 100);
    if (server.hasArg("p4")) spreadPowerPct[3] = constrain(server.arg("p4").toInt(), 0, 100);
    if (server.hasArg("p5")) spreadPowerPct[4] = constrain(server.arg("p5").toInt(), 0, 100);
    if (server.hasArg("p6")) spreadPowerPct[5] = constrain(server.arg("p6").toInt(), 0, 100);
    sanitizeSpreadTable();

    if (oldSSID != wifiSSID || oldPass != wifiPass) wifiChanged = true;

        sendLog("Settings saved - maxPower=" + String(maxPwmPercent) + "%" +
          ", riseHyst=" + String(spreadRiseHystC, 2) + "C" +
          ", ringCutoff=" + String(ringTempCutoff, 1) + "C" +
          ", table=[" + String(spreadPointC[0], 2) + "->" + String(spreadPowerPct[0]) + "%"
          + "," + String(spreadPointC[1], 2) + "->" + String(spreadPowerPct[1]) + "%"
          + "," + String(spreadPointC[2], 2) + "->" + String(spreadPowerPct[2]) + "%"
          + "," + String(spreadPointC[3], 2) + "->" + String(spreadPowerPct[3]) + "%"
          + "," + String(spreadPointC[4], 2) + "->" + String(spreadPowerPct[4]) + "%"
          + "," + String(spreadPointC[5], 2) + "->" + String(spreadPowerPct[5]) + "%]");
    saveConfig();

    // Respond with JSON so JS fetch() can handle it without redirect
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", "{\"ok\":true}");

    if (wifiChanged) {
      sendLog("WiFi credentials changed - restarting...");
      delay(500); ESP.restart();
    }
  });

  server.on("/pwmtest", HTTP_POST, []() {
    if (!pwmTest) {
      if (server.hasArg("pwm")) {
        int v = server.arg("pwm").toInt();
        pwm = map(constrain(v, 0, 100), 0, 100, 0, 255);
        analogWrite(PWM_PIN, pwm);
        pwmTest = true;
        sendLog("PWM test STARTED -> " + String(v) + "%");
      }
    } else {
      pwmTest = false; pwm = 0; analogWrite(PWM_PIN, 0);
      sendLog("PWM test STOPPED");
    }
    server.send(200);
  });

  server.on("/clearlog", HTTP_POST, []() {
    logBuffer = "Log cleared.\n"; server.send(200);
  });

  server.on("/clearlog_thermal", HTTP_POST, []() {
    if (SPIFFS.exists(LOG_FILE)) { SPIFFS.remove(LOG_FILE); sendLog("Thermal log cleared."); }
    else sendLog("No thermal log to clear.");
    server.send(200);
  });

  server.on("/downloadlog", HTTP_GET, []() {
    if (SPIFFS.exists(LOG_FILE)) {
      File f = SPIFFS.open(LOG_FILE, FILE_READ);
      server.streamFile(f, "text/csv"); f.close();
    } else server.send(404, "text/plain", "No thermal log found");
  });

  server.begin();
  sendLog("Web server started");
}

// ==========================================================================
//  WiFi RECONNECT
// ==========================================================================

void handleWiFiReconnect() {
  static unsigned long lastAttempt = 0, offlineSince = 0;
  static bool wasDown = false;

  if (wifiSSID.isEmpty()) return;

  if (apFallbackActive) {
    static unsigned long lastScan = 0; static bool scanning = false;
    unsigned long now = millis();
    if (!scanning && now - lastScan >= AP_STA_SCAN_INTERVAL_MS) {
      if (WiFi.getMode() == WIFI_AP) WiFi.mode(WIFI_AP_STA);
      WiFi.scanNetworks(true); scanning = true; lastScan = now; return;
    }
    if (scanning) {
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) return;
      scanning = false;
      bool found = false;
      if (n >= 0) {
        for (int i = 0; i < n; i++) if (WiFi.SSID(i) == wifiSSID) { found = true; break; }
        WiFi.scanDelete();
      }
      if (found) {
        WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
        apFallbackActive = false; wasDown = true; offlineSince = lastAttempt = now;
      } else if (WiFi.getMode() == WIFI_AP_STA) WiFi.mode(WIFI_AP);
    }
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (wasDown) {
      wasDown = false;
      sendLog("WiFi reconnected: " + WiFi.localIP().toString());
      if (WiFi.getMode() == WIFI_AP_STA) { WiFi.mode(WIFI_STA); delay(200); }
      delay(500);
      if (!MDNS.begin("DewController")) sendLog("mDNS restart failed");
      else sendLog("mDNS restarted");
      server.stop(); delay(200); setupWebServer();
      applyLocalTimezone();
      if (!ntpInitialized) { setupNTP(); ntpInitialized = true; }
      else handleNTP(true);
    }
    return;
  }

  if (!wasDown) { wasDown = true; offlineSince = millis(); sendLog("WiFi disconnected"); }
  unsigned long now = millis();
  if (now - lastAttempt < WIFI_RECONNECT_INTERVAL_MS) return;
  lastAttempt = now;

  wl_status_t s = WiFi.status();
  if (s == WL_IDLE_STATUS || s == WL_DISCONNECTED) WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  else if ((s == WL_NO_SSID_AVAIL || s == WL_CONNECT_FAILED) && offlineSince && now - offlineSince > 90000) {
    WiFi.disconnect(true, true); delay(200); WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str()); offlineSince = now;
  } else WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
}

// ==========================================================================
//  LED
// ==========================================================================

void handleLEDStatus() {
  static unsigned long lastT = 0; static bool state = false; static int mode = -1;
  if (WiFi.getMode() == WIFI_AP) {
    if (mode != 2) { mode = 2; lastT = millis(); state = false; }
    if (millis() - lastT > 500) { lastT = millis(); state = !state; digitalWrite(LED_PIN, state); }
  } else if (WiFi.status() == WL_CONNECTED) {
    if (mode != 1) { mode = 1; digitalWrite(LED_PIN, ON); }
  } else {
    if (mode != 0) { mode = 0; digitalWrite(LED_PIN, OFF); }
  }
}

// ==========================================================================
//  SENSOR TASKS
// ==========================================================================

void weatherTask(void* p) {
  float t = NAN, h = NAN;
  for (;;) {
    if (sensorSource == SOURCE_WEATHER && fetchOutdoorWeather(&t, &h))
      writeSensorValues(t, h);
    delay(WEATHER_POLL_INTERVAL_MS);
  }
}

void sensorTask_SHT40(void* p) {
  sendLog("SHT40 task core " + String(xPortGetCoreID()));
  I2CBus.begin(I2C_SDA, I2C_SCL); delay(30);

  if (!sht4.begin(&I2CBus)) {
    sendLog("SHT40 init failed - I2C scan:");
    bool any = false;
    for (uint8_t a = 1; a < 127; a++) {
      I2CBus.beginTransmission(a);
      if (!I2CBus.endTransmission()) { sendLog("  0x" + String(a, HEX)); any = true; }
    }
    if (!any) sendLog("  No I2C devices found");
    sendLog("Falling back to weather API");
    sensorSource = SOURCE_WEATHER;
    xTaskCreatePinnedToCore(weatherTask, "WeatherTask", 8192, NULL, 1, NULL, 0);
    vTaskDelete(NULL); return;
  }

  sendLog("SHT40 ready");
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  for (;;) {
    sensors_event_t he, te;
    sht4.getEvent(&he, &te);
    if (!isnan(te.temperature) && !isnan(he.relative_humidity))
      writeSensorValues(te.temperature, he.relative_humidity);
    else sendLog("SHT40 bad reading");
    yield(); delay(3000);
  }
}

// ==========================================================================
//  CONTROL TASK  (background, core 0)
// ==========================================================================

void controlTask(void* p) {
  sendLog("Control task core " + String(xPortGetCoreID()));
  delay(2000);  // wait for sensor to stabilise

  unsigned long lastControl  = 0;
  unsigned long lastThermal  = 0;
  unsigned long lastCache    = millis() - RSSI_UPDATE_INTERVAL_MS;  // force immediate

  for (;;) {
    unsigned long now = millis();

    // Heater control - every second
    if (now - lastControl >= APP_UPDATE_INTERVAL_MS) {
      lastControl = now;
      updateHeaterControl();
    }

    // Thermal log - every 10 minutes
    if (now - lastThermal >= THERMAL_LOG_INTERVAL_MS) {
      lastThermal = now;
      logThermalData();
    }

    // Cache update - every 2 minutes
    if (now - lastCache >= RSSI_UPDATE_INTERVAL_MS) {
      lastCache = now;
      float t, h;
      readSensorValues(t, h);
      float td     = dewPointC(t, h);
      float spread = isnan(td) ? NAN : (t - td);
      String stor  = getStorageUsedSummary();
      int rssi     = WiFi.RSSI();
      if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(5))) {
        cached_td = td; cached_spread = spread;
        cached_storageInfo = stor; cached_rssi = rssi;
        xSemaphoreGive(cacheMutex);
      }
    }

    delay(CONTROL_TASK_DELAY_MS);
  }
}

// ==========================================================================
//  SETUP & LOOP
// ==========================================================================

void setup() {
  i2cMutex   = xSemaphoreCreateMutex();
  cacheMutex = xSemaphoreCreateMutex();
  logMutex   = xSemaphoreCreateMutex();
  logBuffer  = "";

#if DEBUG_MODE
  Serial.begin(115200); delay(1200);
  DEBUG_PRINT("Serial 115200");
#endif

  sendLog(String(DEVICE_NAME) + " " + String(DEVICE_VERSION));

  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 1000);
  analogWrite(PWM_PIN, 0);

  SPIFFS.begin(true);
  loadConfig();
  applyLocalTimezone();
  setupWiFi();

  sendLog("Starting tasks...");

  xTaskCreatePinnedToCore(sensorTask_SHT40, "SHT40Task", 8192, NULL, 1, &sensorTaskHandle, 0);
  sendLog("SHT40 task created");
  delay(100);

  xTaskCreatePinnedToCore(controlTask, "ControlTask", 8192, NULL, 1, &controlTaskHandle, 0);
  sendLog("Control task created");
  delay(100);

  sendLog("Ready - spread table dew control active");
  sendLog("   Table: " + String(spreadPointC[0], 2) + "C->" + String(spreadPowerPct[0]) + "%"
    ", " + String(spreadPointC[1], 2) + "C->" + String(spreadPowerPct[1]) + "%"
    ", " + String(spreadPointC[2], 2) + "C->" + String(spreadPowerPct[2]) + "%"
    ", " + String(spreadPointC[3], 2) + "C->" + String(spreadPowerPct[3]) + "%"
    ", " + String(spreadPointC[4], 2) + "C->" + String(spreadPowerPct[4]) + "%"
    ", " + String(spreadPointC[5], 2) + "C->" + String(spreadPowerPct[5]) + "%");
  sendLog("   Spread > " + String(spreadPointC[0], 2) + "C -> Off");
  sendLog("   Max Power: " + String(maxPwmPercent) + "%");
  sendLog("   Rise Hysteresis: " + String(spreadRiseHystC, 2) + "C");
}

void loop() {
  static unsigned long lastWiFi = 0, lastNTP = 0, lastHealth = 0;
  unsigned long now = millis();

  server.handleClient();
  yield();
  handleLEDStatus();

  if (now - lastWiFi >= WIFI_RECONNECT_INTERVAL_MS) { lastWiFi = now; handleWiFiReconnect(); }
  if (now - lastNTP  >= NTP_CHECK_INTERVAL_MS)       { lastNTP  = now; handleNTP(); }

  if (now - lastHealth > 60000) {
    lastHealth = now;
    if (controlTaskHandle && eTaskGetState(controlTaskHandle) == eDeleted) {
      sendLog("Control task died - restarting"); ESP.restart();
    }
    if (sensorTaskHandle && eTaskGetState(sensorTaskHandle) == eDeleted) {
      sendLog("Sensor task died - restarting"); ESP.restart();
    }
  }

  delay(MAIN_LOOP_DELAY_MS);
}
