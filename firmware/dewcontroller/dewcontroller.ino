// DewHeaterController — Dew Spread Control
// Replaces RH threshold gating with dew-point spread gating (Ta - Td)
// Default dew spread threshold = 3.0 °C, hysteresis = 1.0 °C

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SHT4x.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include "esp_log.h"
#include <time.h>
#include <HTTPClient.h>
#include <math.h>

#define DEVICE_VERSION "v2.6.0"
#define DEVICE_NAME "DewHeaterController"
#define SIMULATE_HARDWARE 0
#define DEBUG_MODE 0
#define CONFIG_FILE "/config.json"
#define CALIBRATION_FILE "/calibration.csv"
#define LOOP_INTERVAL_MS 2000
#define WEATHER_POLL_INTERVAL_MS 5 * 60 * 1000 // 300000 ms or 5 minutes

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

#define DEFAULT_TEMPDELTA     2.0


//#define DEBUG_MODE 1
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
SensorSource sensorSource = SOURCE_SHT40;  // default

SemaphoreHandle_t   i2cMutex;
TaskHandle_t        sensorTaskHandle;
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
float dewSpreadHysteresis = DEWSPREAD_HYSTERISIS;            // °C to avoid chatter

// Thermistor coefficients (best-fit)
float glassCoeffA = GLASSCOEFFA;
float glassCoeffB = GLASSCOEFFB;
float glassCoeffC = GLASSCOEFFC;

String  wifiSSID = "MicroConcepts-2G";
String  wifiPass = "";
float   timezoneOffsetHours = 10.0;
bool    heaterEnabled = false, calibrating = false;

// -------------- Calibration variables --------------
float   lastSampledTemp = -100.0;
int     calCount = 0, pwm = 0;
bool    pwmTest = false;
const unsigned long CAL_SAMPLE_INTERVAL_MS = 30000;  // 30 seconds
unsigned long lastCalSampleTime = 0;
bool    firstCalSample = true;  // header line

const int MAX_LOG_SIZE = 32768;
String  logBuffer;

// -------------- Utility --------------
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

    sendLog("📡 Extracted temp=" + String(temp) + ", humidity=" + String(hum));

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

void sendLog(const String& msg) {
  DEBUG_PRINT(msg);
  logBuffer += msg + "\n";
  if (logBuffer.length() > MAX_LOG_SIZE) {
    logBuffer = logBuffer.substring(logBuffer.length() - MAX_LOG_SIZE);
  }
}

void saveConfig() {
  StaticJsonDocument<512> doc;
  doc["delta"] = targetDelta;
  doc["dewSpread"] = dewSpreadThreshold;         // NEW
  doc["dewSpreadHysteresis"] = dewSpreadHysteresis; // NEW
  doc["glassCoeffA"] = glassCoeffA;
  doc["glassCoeffB"] = glassCoeffB;
  doc["glassCoeffC"] = glassCoeffC;
  doc["heater"] = heaterEnabled;
  doc["ssid"] = wifiSSID;
  doc["password"] = wifiPass;
  doc["timezone"] = timezoneOffsetHours;
  File file = SPIFFS.open(CONFIG_FILE, FILE_WRITE);
  if (file) {
    serializeJson(doc, file);
    file.close();
    sendLog("💾 Config saved");
  } else {
    sendLog("❌ Failed to save config");
  }
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
    return;
  }

  targetDelta          = doc["delta"]               | DEFAULT_TEMPDELTA;
  dewSpreadThreshold   = doc["dewSpread"]           | DEFAULT_DEWSPREAD;  // default
  dewSpreadHysteresis  = doc["dewSpreadHysteresis"] | DEWSPREAD_HYSTERISIS;  // default

  // Back-compat for older configs that only had "humidity" key:
  if (!doc.containsKey("dewSpread") && doc.containsKey("humidity")) {
    dewSpreadThreshold = DEFAULT_DEWSPREAD; // migrate to sensible default
  }

  heaterEnabled       = doc["heater"]      | false;
  wifiSSID            = doc["ssid"]        | "MicroConcepts-2G";
  wifiPass            = doc["password"]    | "leanneannatinka";
  timezoneOffsetHours = doc["timezone"]    | 10.0;

  sendLog("✅ Config loaded from SPIFFS");
  sendLog("📦 Temperature Delta: " + String(targetDelta, 2));
  sendLog("📦 Dew Spread Threshold: " + String(dewSpreadThreshold, 2) + " °C");
  sendLog("📦 Dew Spread Hysteresis: " + String(dewSpreadHysteresis, 2) + " °C");
  sendLog("📦 Heater Enabled: " + String(heaterEnabled ? "true" : "false"));
  sendLog("📦 WiFi SSID: " + wifiSSID);
  sendLog(String("📦 Timezone Offset: UTC") + (timezoneOffsetHours >= 0 ? "+" : "") + String(timezoneOffsetHours, 1));
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

  logThermalLagInfo();
}

// Log thermal lag periodically
void logThermalLagInfo() {
  static float prevTa = 0.0;
  static unsigned long prevTime = 0;

  float Ta = ambientTemp;
  float Tg = glassTemp;

  unsigned long now = millis();
  float dtSeconds = (now - prevTime) / 1000.0;

  if (prevTime == 0 || dtSeconds < 60) {
    prevTa = Ta;
    prevTime = now;
    return;
  }

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

  sendLog("🔌 Connecting to Wi-Fi...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("DewController");
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  WiFi.setSleep(false);
  #if DEBUG_MODE
    esp_log_level_set("*", ESP_LOG_ERROR);
  #endif

  int maxRetries = 10;
  int retries = 0;

  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    delay(500);
    retries++;
    sendLog("⏳ Attempt " + String(retries) + ": Status = " + String(WiFi.status()));
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    sendLog("✅ WiFi IP: " + WiFi.localIP().toString());

    if (!MDNS.begin("DewController")) {
      sendLog("❌ Error setting up MDNS responder!");
    } else {
      sendLog("✅ MDNS responder started as DewController.local");
    }

    long gmtOffset_sec = timezoneOffsetHours * 3600;
    configTime(gmtOffset_sec, 0, "pool.ntp.org", "time.nist.gov");

    struct tm timeinfo;
    int waitCount = 0;
    while (!getLocalTime(&timeinfo) && waitCount < 20) {
      delay(200);
      waitCount++;
    }

    if (getLocalTime(&timeinfo)) {
      char buf[64];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
      sendLog(String("⏰ NTP time set: ") + buf);
    } else {
      sendLog("⚠️ Failed to get NTP time.");
    }

  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DewHeaterSetup");
    sendLog("❌ WiFi failed. Starting fallback AP...");
    sendLog("📡 AP mode IP: " + WiFi.softAPIP().toString());
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
          fetch('/status.json').then(r => r.json()).then(data => {
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
          fetch('/log').then(r => r.text()).then(data => {
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
        }, 3000);

        window.onload = () => { updateStatus(); updateLog(); };

      </script>
      </head><body>
      <h2 style="padding:10px">
        Dew Heater Controller 
        <span id="version" style="font-size: 0.6em; color: #666;"></span>
      </h2>
      <div id="main">
        <div id="status">
          <table style="font-family: monospace; font-size: 1rem;">
            <tr><td>🌡 Ambient:</td>       <td><span id='ambient'>--</span> °C</td></tr>
            <tr><td>💧 Humidity:</td>      <td><span id='humidity'>--</span> %</td></tr>
            <tr><td>🧮 Dew Point:</td>     <td><span id='dewpoint'>--</span> °C</td></tr>
            <tr><td>📉 Dew Spread:</td>    <td><span id='dewspread'>--</span> °C</td></tr>
            <tr><td>🔍 Glass:</td>         <td><span id='glass'>--</span> °C</td></tr>
            <tr><td>📏 Delta (Tg - Ta):</td><td><span id='delta'>--</span> °C</td></tr>
            <tr id="heaterRow"><td>🔌 Heater:</td> <td><span id='heater'>--</span></td></tr>
            <tr id="pwmRow"><td>🔥 Heating Power:</td> <td><span id='pwm'>--</span>%</td></tr>
            <tr><td>🗂 Calibration File:</td> <td><span id='calibrationFileSize'>--</span></td></tr>
            <tr><td>💾 Storage Used:</td> <td><span id='storageInfo'>--</span></td></tr>
          </table>
        </div>

        <div id="settings">
          <form action="/settings" method="POST">
            <div class="inputrow">
              <label>Temperature Delta (°C):</label>
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
              <label>Timezone Offset:</label>
              <input name="timezone" value=")rawliteral" + String(timezoneOffsetHours, 1) + R"rawliteral(">
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
    float t, h;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
      t = ambientTemp;
      h = humidity;
      xSemaphoreGive(i2cMutex);
    } else {
      t = ambientTemp;
      h = humidity;
    }

    float delta = glassTemp - t;
    float td = dewPointC(t, h);
    float spread = isnan(td) ? NAN : (t - td);
    int pwmPercent = pwm * 100 / 255;

    StaticJsonDocument<256> doc;
    doc["ambient"] = String(t, 1);
    doc["humidity"] = String(h, 1);
    doc["dewpoint"] = isnan(td) ? String("NaN") : String(td, 1);
    doc["dewspread"] = isnan(spread) ? String("NaN") : String(spread, 1);
    doc["glass"] = String(glassTemp, 1);
    doc["delta"] = String(delta, 1);
    doc["heater"] = heaterEnabled;
    doc["pwm"] = pwmPercent;
    doc["calibrating"] = calibrating;
    doc["calibrationFileSize"] = getCalibrationFileSizeKB(CALIBRATION_FILE);
    doc["storageInfo"] = getStorageUsedSummary();
    doc["version"] = DEVICE_VERSION;
    doc["dewSpreadThreshold"] = String(dewSpreadThreshold, 1);
    doc["dewSpreadHysteresis"] = String(dewSpreadHysteresis, 1);

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json; charset=UTF-8", json);
  });

  server.on("/log", []() {
    server.send(200, "text/plain", logBuffer);
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
    if (server.hasArg("delta"))     targetDelta = server.arg("delta").toFloat();
    if (server.hasArg("dewspread")) dewSpreadThreshold = server.arg("dewspread").toFloat();
    if (server.hasArg("dewhyst"))   dewSpreadHysteresis = server.arg("dewhyst").toFloat();
    if (server.hasArg("ssid"))      wifiSSID = server.arg("ssid");
    if (server.hasArg("password"))  wifiPass = server.arg("password");
    if (server.hasArg("timezone"))  timezoneOffsetHours = server.arg("timezone").toFloat();

    sendLog("⚙️ Settings updated");
    saveConfig();
    server.sendHeader("Location", "/");
    server.send(303);
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
      while (f.available()) {
        String line = f.readStringUntil('\n');
        sendLog("📌 " + line);
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
        sendLog("🌤 Weather updated: " + String(outTemp,1) + "°C, " + String(outHumidity,1) + "%");
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
  delay(10);

  if (!aht.begin(&I2CBus)) {
    sendLog("❌ AHT20 init failed");
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
  delay(10);

  if (!sht4.begin(&I2CBus)) {
    sendLog("❌ SHT40 init failed");
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
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
        ambientTemp = temp_event.temperature;
        humidity = humidity_event.relative_humidity;
        xSemaphoreGive(i2cMutex);
      }
    } else {
      sendLog("⚠️ SHT40 returned bad data");
    }
    delay(2000);
  }
}

void handleWiFiReconnect() {
  static unsigned long lastReconnectAttempt = 0;
  static bool wasDisconnected = false;

  if (WiFi.status() != WL_CONNECTED) {
    if (!wasDisconnected) {
      sendLog("⚠️ WiFi disconnected, attempting reconnect...");
      wasDisconnected = true;
    }
    if (millis() - lastReconnectAttempt > 10000) {
      lastReconnectAttempt = millis();
      WiFi.disconnect();
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
  } else {
    if (wasDisconnected) {
      sendLog("✅ WiFi reconnected: " + WiFi.localIP().toString());
      wasDisconnected = false;
    }
  }
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

  float delta = glassTemp - t;
  float error = targetDelta - delta;

  // ---- Gate by dew spread (Ta - Td) with hysteresis ----
  float td = dewPointC(t, humidity);
  bool dewGate = false;
  static bool dewGateLatched = false; // on/off hysteresis latch

  if (!isnan(td)) {
    float spread = t - td;  // smaller spread => higher condensation risk
    // ON when spread <= threshold; OFF when spread > threshold + hysteresis
    if (!dewGateLatched && spread <= dewSpreadThreshold) {
      dewGateLatched = true;
    } else if (dewGateLatched && spread > (dewSpreadThreshold + dewSpreadHysteresis)) {
      dewGateLatched = false;
    }
    dewGate = dewGateLatched;
  } else {
    // If dew point cannot be computed, fail-safe OFF
    dewGate = false;
  }

  if (dewGate) {
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

    // If delta >= target, keep at least a small hold to stabilize glass
    if ((glassTemp - t) >= targetDelta && pwmValue < PWM_MIN) {
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

// ---------------- Setup ----------------
void setup() {
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
  setupWiFi();
  setupWebServer();

  i2cMutex = xSemaphoreCreateMutex();
  if (sensorSource == SOURCE_SHT40) {
    xTaskCreatePinnedToCore(sensorTask_SHT40, "SensorTask_SHT40", 4096, NULL, 1, &sensorTaskHandle, 1);
    sendLog("📡 Using SHT40 sensor");
  } else if (sensorSource == SOURCE_WEATHER) {
    xTaskCreatePinnedToCore(weatherTask, "WeatherTask", 8192, NULL, 1, NULL, 1);
    sendLog("📡 Using Weather station");
  } else {  // SOURCE_AHT20
    xTaskCreatePinnedToCore(sensorTask_AHT20, "SensorTask_AHT20", 4096, NULL, 1, &sensorTaskHandle, 1);
    sendLog("📡 Using AHT20 sensor");
  }
  delay(500);
}

// ---------------- Loop ----------------
unsigned long lastUpdate = 0;

void loop() {
  server.handleClient();
  handleWiFiReconnect();
  handleLEDStatus();

  if (millis() - lastUpdate >= LOOP_INTERVAL_MS) {
    lastUpdate = millis();

#if SIMULATE_HARDWARE
    simulateHardware();
#endif
    updateGlassTemp();
    updateHeaterControl();
    updateCalibration();
    logThermalLagInfo();
  }
}
