#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#define DEVICE_NAME "DewHeaterController"
#define SIMULATE_HARDWARE 1
#define DEBUG_MODE 1
#define CONFIG_FILE "/config.json"
#define CALIBRATION_FILE "/calibration.csv"
#define LOOP_INTERVAL_MS 2000

#define I2C_SDA 4
#define I2C_SCL 5
#define THERMISTOR_PIN 6
#define PWM_PIN 9

#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x) do {} while (0)
#endif

WebServer server(80);
Adafruit_AHTX0 aht;

float ambientTemp = 0, humidity = 0, glassTemp = 0;
float targetDelta = 2.0, humidityThreshold = 80.0;
float calibScale = 20.0, calibOffset = 0.0;
String wifiSSID = "MicroConcepts";
String wifiPass = "leanneannatinka";
bool heaterEnabled = false, calibrating = false;
float lastSampledTemp = -100.0;
int calCount = 0;
int currentPWM = 0;

String logBuffer;
const int MAX_LOG_SIZE = 2048;

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
  doc["humidity"] = humidityThreshold;
  doc["scale"] = calibScale;
  doc["offset"] = calibOffset;
  doc["heater"] = heaterEnabled;
  doc["ssid"] = wifiSSID;
  doc["password"] = wifiPass;
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
  if (deserializeJson(doc, file)) {
    sendLog("❌ Failed to parse config");
    file.close();
    return;
  }
  file.close();

  targetDelta = doc["delta"] | 2.0;
  humidityThreshold = doc["humidity"] | 80.0;
  calibScale = doc["scale"] | 20.0;
  calibOffset = doc["offset"] | 0.0;
  heaterEnabled = doc["heater"] | false;
  wifiSSID = doc["ssid"] | "MicroConcepts";
  wifiPass = doc["password"] | "leanneannatinka";
}

float readThermistorVoltage() {
  return analogRead(THERMISTOR_PIN) * 3.3 / 4095.0;
}

float readGlassTemp() {
  return readThermistorVoltage() * calibScale + calibOffset;
}

void computeCalibrationFromCSV() {
  File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
  if (!f) {
    sendLog("❌ Failed to open CSV for reading");
    return;
  }

  String line = f.readStringUntil('\n');
  float sumT = 0, sumV = 0, sumTV = 0, sumV2 = 0;
  int n = 0;
  while (f.available()) {
    line = f.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma < 0) continue;
    float T = line.substring(0, comma).toFloat();
    float V = line.substring(comma + 1).toFloat();
    sumT += T;
    sumV += V;
    sumTV += T * V;
    sumV2 += V * V;
    ++n;
  }
  f.close();

  if (n >= 2) {
    float denom = n * sumV2 - sumV * sumV;
    if (denom != 0) {
      calibScale = (n * sumTV - sumT * sumV) / denom;
      calibOffset = (sumT - calibScale * sumV) / n;
      sendLog("📐 T = " + String(calibScale, 3) + " × V + " + String(calibOffset, 3));
      saveConfig();
    } else {
      sendLog("⚠️ Denominator zero in fit");
    }
  } else {
    sendLog("⚠️ Not enough samples to calibrate");
  }
}

void setupWiFi() {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    sendLog("✅ WiFi IP: " + WiFi.localIP().toString());
  } else {
    WiFi.softAP("DewHeaterSetup");
    sendLog("📡 AP mode IP: " + WiFi.softAPIP().toString());
  }
}

void setupWebServer() {
  server.on("/", []() {
    String html = R"rawliteral(
      <!DOCTYPE html>
      <html><head><meta charset="utf-8"><title>Dew Heater</title>
      <style>
        html, body { margin: 0; padding: 0; font-family: sans-serif; height: 100vh; display: flex; flex-direction: column; }
        #panel { display: flex; justify-content: space-between; padding: 10px; gap: 20px; }
        #status, #settings { flex: 1; }
        #buttons { text-align: center; margin: 10px 0; }
        #log { flex-grow: 1; overflow-y: auto; background: #f8f8f8; border-top: 1px solid #ccc; padding: 10px; white-space: pre-wrap; }
      </style>
      <script>
        function updateStatus() {
          fetch('/status.json')
            .then(r => r.json())
            .then(data => {
              document.getElementById("ambient").innerText = data.ambient + " °C";
              document.getElementById("humidity").innerText = data.humidity + " %";
              document.getElementById("glass").innerText = data.glass + " °C";
              document.getElementById("delta").innerText = data.delta + " °C";
              document.getElementById("heater").innerText = data.heater ? "ON" : "OFF";
              document.getElementById("pwm").innerText = data.pwm + " %";
              document.getElementById("tformula").innerText = "T = " + data.scale + " × V + " + data.offset;
              document.getElementById("deltaInput").value = data.setdelta;
              document.getElementById("humidityInput").value = data.sethumidity;
            });
        }
        function updateLog() {
          fetch('/log')
            .then(r => r.text())
            .then(data => {
              let logDiv = document.getElementById("log");
              logDiv.innerText = data;
              logDiv.scrollTop = logDiv.scrollHeight;
            });
        }
        function updateSettings(e) {
          e.preventDefault();
          const formData = new FormData(e.target);
          fetch('/update', {
            method: 'POST',
            body: new URLSearchParams(formData)
          }).then(() => updateStatus());
        }
        setInterval(() => { updateStatus(); updateLog(); }, 3000);
        window.onload = () => { updateStatus(); updateLog(); }
      </script></head>
      <body>
        <h2 style="text-align:center;">Dew Heater Controller</h2>
        <div id="panel">
          <div id="status">
            <p>Ambient: <span id="ambient">--</span></p>
            <p>Humidity: <span id="humidity">--</span></p>
            <p>Glass: <span id="glass">--</span></p>
            <p>Delta: <span id="delta">--</span></p>
            <p>Heater: <span id="heater">--</span></p>
            <p>Heating Power: <span id="pwm">--</span></p>
          </div>
          <div id="settings">
            <form onsubmit="updateSettings(event)">
              <p>Temperature Delta: <input id="deltaInput" name="delta" type="number" step="0.1"></p>
              <p>Humidity Threshold: <input id="humidityInput" name="humidity" type="number" step="0.1"></p>
              <p><button type="submit">Update Settings</button></p>
            </form>
            <p>Calibration: <span id="tformula">T = --</span></p>
          </div>
        </div>
        <div id="buttons">
          <form style="display:inline;" action="/toggle" method="POST"><button>Toggle Heater</button></form>
          <form style="display:inline;" action="/calibrate" method="POST"><button>Toggle Calibration</button></form>
        </div>
        <div id="log">Loading logs...</div>
      </body></html>
    )rawliteral";
    server.send(200, "text/html; charset=UTF-8", html);
  });

  server.on("/status.json", []() {
    float delta = glassTemp - ambientTemp;
    int pwmPercent = (int)(currentPWM * 100.0 / 255.0);
    String json = "{";
    json += "\"ambient\":" + String(ambientTemp, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"glass\":" + String(glassTemp, 1) + ",";
    json += "\"delta\":" + String(delta, 1) + ",";
    json += "\"heater\":" + String(heaterEnabled ? "true" : "false") + ",";
    json += "\"pwm\":" + String(pwmPercent) + ",";
    json += "\"scale\":" + String(calibScale, 2) + ",";
    json += "\"offset\":" + String(calibOffset, 2) + ",";
    json += "\"setdelta\":" + String(targetDelta, 1) + ",";
    json += "\"sethumidity\":" + String(humidityThreshold, 1);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/log", []() {
    server.send(200, "text/plain", logBuffer);
  });

  server.on("/toggle", HTTP_POST, []() {
    heaterEnabled = !heaterEnabled;
    sendLog("Toggled heater: " + String(heaterEnabled));
    saveConfig();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/calibrate", HTTP_POST, []() {
    calibrating = !calibrating;
    lastSampledTemp = -100.0;
    calCount = 0;
    sendLog(calibrating ? "🧪 Calibration STARTED" : "✅ Calibration STOPPED");
    if (calibrating) {
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_WRITE);
      if (f) {
        f.println("T,V");
        f.close();
      }
    } else {
      computeCalibrationFromCSV();
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/update", HTTP_POST, []() {
    if (server.hasArg("delta")) targetDelta = server.arg("delta").toFloat();
    if (server.hasArg("humidity")) humidityThreshold = server.arg("humidity").toFloat();
    sendLog("Updated delta = " + String(targetDelta) + ", humidity = " + String(humidityThreshold));
    saveConfig();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
  sendLog("🌐 Web server started");
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 1000);
  analogWrite(PWM_PIN, 0);
  Wire.begin(I2C_SDA, I2C_SCL);
#if !SIMULATE_HARDWARE
  aht.begin(&Wire);
#endif
  SPIFFS.begin(true);
  loadConfig();
  setupWiFi();
  setupWebServer();
}

unsigned long lastUpdate = 0;
void loop() {
  server.handleClient();
  if (millis() - lastUpdate >= LOOP_INTERVAL_MS) {
    lastUpdate = millis();
#if SIMULATE_HARDWARE
    ambientTemp = 17.0 + sin(millis() / 12000.0);
    humidity = 78.0 + sin(millis() / 8000.0);
#else
    sensors_event_t t, h;
    aht.getEvent(&h, &t);
    ambientTemp = t.temperature;
    humidity = h.relative_humidity;
#endif
    glassTemp = readGlassTemp();
    float delta = glassTemp - ambientTemp;
    currentPWM = (heaterEnabled && humidity >= humidityThreshold && delta < targetDelta)
                  ? map(constrain(targetDelta - delta, 0, 5), 0, 5, 64, 255)
                  : 0;
    analogWrite(PWM_PIN, currentPWM);

    if (calibrating && abs(ambientTemp - lastSampledTemp) >= 0.1) {
      float v = readThermistorVoltage();
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_APPEND);
      if (f) {
        f.printf("%.2f,%.3f\n", ambientTemp, v);
        f.close();
        sendLog("📌 Sample " + String(calCount + 1) + ": T=" + String(ambientTemp, 2) + " V=" + String(v, 3));
      }
      lastSampledTemp = ambientTemp;
      calCount++;
      if (calCount >= 40) {
        calibrating = false;
        sendLog("✅ Calibration STOPPED (40 samples collected)");
        computeCalibrationFromCSV();
      }
    }
  }
}
