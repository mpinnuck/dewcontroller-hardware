#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#define DEVICE_NAME "DewHeaterController"
#define SIMULATE_HARDWARE 0
#define DEBUG_MODE 1
#define CONFIG_FILE "/config.json"
#define CALIBRATION_FILE "/calibration.csv"
#define LOOP_INTERVAL_MS 2000

#define I2C_SDA 5
#define I2C_SCL 6
#define THERMISTOR_PIN 3
#define PWM_PIN 7
#define LED_PIN 21
#define ON LOW
#define OFF HIGH



#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x) do {} while (0)
#endif

SemaphoreHandle_t   i2cMutex;
TaskHandle_t        sensorTaskHandle;

WebServer           server(80);

TwoWire I2CBus = TwoWire(1);  // Dedicated I2C bus
Adafruit_AHTX0      aht;

float   ambientTemp = 0, humidity = 0, glassTemp = 0;
float   targetDelta = 2.0, humidityThreshold = 80.0;
float   calibScale = 20.0, calibOffset = 0.0;
String  wifiSSID = "MicroConcepts";
String  wifiPass = "leanneannatinka";
bool    heaterEnabled = false, calibrating = false;
float   lastSampledTemp = -100.0;
int     calCount = 0, pwm = 0;
bool pwmTest = false;

String  logBuffer;
const int MAX_LOG_SIZE = 4096;

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
  DeserializationError err = deserializeJson(doc, file);
  file.close();

  if (err) {
    sendLog("❌ Failed to parse config: " + String(err.c_str()));
    return;
  }

  // Assign values
  targetDelta       = doc["delta"]       | 2.0;
  humidityThreshold = doc["humidity"]    | 80.0;
  calibScale        = doc["scale"]       | 20.0;
  calibOffset       = doc["offset"]      | 0.0;
  heaterEnabled     = doc["heater"]      | false;
  wifiSSID          = doc["ssid"]        | "MicroConcepts-2G";
  wifiPass          = doc["password"]    | "leanneannatinka";

  sendLog("✅ Config loaded from SPIFFS");
  sendLog("📦 Temperature Delta: " + String(targetDelta, 2));
  sendLog("📦 Humidity Threshold: " + String(humidityThreshold, 2));
  sendLog("📦 Heater Enabled: " + String(heaterEnabled ? "true" : "false"));
  sendLog("📦 WiFi SSID: " + wifiSSID);
  sendLog("📦 WiFi Password: " + wifiPass);
  sendLog("📦 T = " + String(calibScale, 2) + " × V + " + String(calibOffset, 2));
}

float readThermistorVoltage() {
  return analogRead(THERMISTOR_PIN) * 3.3 / 4095.0;
}

float readGlassTemp() {
  return readThermistorVoltage() * calibScale + calibOffset;
}

void computeCalibrationFromCSV() {
  File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
  if (!f) return;

  String line = f.readStringUntil('\n');  // skip header
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
    }
  }
}

void setupWiFi() {
  i2cMutex = xSemaphoreCreateMutex();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, OFF);  // Start with LED off

  sendLog("📶 Target SSID: " + wifiSSID);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  // Try to connect regardless (maybe hidden network or weak signal)
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
  sendLog("🔌 Attempting connection...");

  int maxRetries = 15;
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    delay(500);
    wl_status_t status = WiFi.status();
//    sendLog("⏳ Attempt " + String(retries + 1) + ": Status = " + String(status));
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, ON);  // Start with LED off
    sendLog("✅ WiFi connected, IP: " + WiFi.localIP().toString());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DewHeaterSetup");
    sendLog("🚨 Fallback to AP mode");
    sendLog("📡 AP IP: " + WiFi.softAPIP().toString());
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
      #main {
        display: flex;
        padding: 10px;
        flex-direction: row;         /* left to right */
        align-items: flex-start;     /* top align */
        gap: 20px;                   /* spacing between status and settings */
        justify-content: flex-start; /* 👈 stick both to the left */
      }

      /* prevent flex children from growing */
      #status, #settings {
        flex: 0 0 auto;              /* 👈 fixed width based on content */
      }

      /*  Stack vertically on small screens (less than 600px wide) */
      @media (max-width: 600px) {
        #main {
          flex-direction: column;
        }

        #status, #settings {
          width: 100%;
        }
      }


        #log {
          flex-grow: 1; overflow-y: auto;
          border-top: 1px solid #ccc; background: #f8f8f8;
          padding: 10px; white-space: pre-wrap;
          box-sizing: border-box; max-height: calc(100vh - 240px);
          min-height: 100px; /* Prevent collapse if log is empty */
        }
        #controls {
          display: flex; gap: 10px; padding: 10px;
        }
        .inputrow {
          display: flex; align-items: center; margin: 5px 0;
        }
        .inputrow label {
          width: 150px;
        }
        .inputrow input {
          flex: 1;
        }
      </style>
      <script>
        function updateStatus() {
          fetch('/status.json').then(r => r.json()).then(data => {
            document.getElementById("ambient").innerText = data.ambient;
            document.getElementById("humidity").innerText = data.humidity;
            document.getElementById("glass").innerText = data.glass;
            document.getElementById("delta").innerText = data.delta;
            const heaterOn = data.heater;
            const heaterSpan = document.getElementById("heater");
            const heaterRow = document.getElementById("heaterRow");
            heaterSpan.innerText = heaterOn ? "ON" : "OFF";
            heaterRow.style.backgroundColor = heaterOn ? "#c0ffc0" : "transparent";
            const pwmVal = parseInt(data.pwm);
            const pwmSpan = document.getElementById("pwm");
            const pwmRow = document.getElementById("pwmRow");
            pwmSpan.innerText = pwmVal + "%";
            pwmRow.style.backgroundColor = pwmVal > 0 ? "#c0ffc0" : "transparent";
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
          fetch("/pwmtest", {
            method: "POST",
            headers: { "Content-Type": "application/x-www-form-urlencoded" },
            body: "pwm=" + val
          }).then(() => {
            console.log("PWM test value sent: " + val);
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
            .then(() => setTimeout(updateLog, 300)); // optional short delay to ensure log is flushed
        }

        setInterval(() => { updateStatus(); updateLog(); }, 3000);
        window.onload = () => { updateStatus(); updateLog(); };
      </script>
      </head><body>
      <h2 style="padding:10px">Dew Heater Controller</h2>
      <div id="main">
        <div id="status">
          <table style="font-family: monospace; font-size: 1rem;">
            <tr><td>🌡 Ambient:</td>       <td><span id='ambient'>--</span> °C</td></tr>
            <tr><td>💧 Humidity:</td>      <td><span id='humidity'>--</span> %</td></tr>
            <tr><td>🔍 Glass:</td>         <td><span id='glass'>--</span> °C</td></tr>
            <tr><td>📏 Delta:</td>         <td><span id='delta'>--</span> °C</td></tr>
            <tr id="heaterRow"><td>🔌 Heater:</td> <td><span id='heater'>--</span></td></tr>
            <tr id="pwmRow"><td>🔥 Heating Power:</td> <td><span id='pwm'>--</span>%</td></tr>            
          </table>
        </div>
        <div id="settings">
          <form action="/settings" method="POST">
            <div class="inputrow">
              <label>Temperature Delta:</label>
              <input name="delta" value=")rawliteral" + String(targetDelta, 1) + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Humidity Threshold:</label>
              <input name="humidity" value=")rawliteral" + String(humidityThreshold) + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>SSID:</label>
              <input name="ssid" value=")rawliteral" + wifiSSID + R"rawliteral(">
            </div>
            <div class="inputrow">
              <label>Password:</label>
              <input name="password" type="password" value=")rawliteral" + wifiPass + R"rawliteral(">
            </div>
            <button type="submit">Update Settings</button>
          </form>
          <p>Calibration: T = )rawliteral" + String(calibScale, 2) + R"rawliteral( × V + )rawliteral" + String(calibOffset, 2) + R"rawliteral(</p>
        </div>
      </div>
      <div id="controls">
        <form action="/toggle" method="POST"><button>Toggle Heater</button></form>
        <form action="/calibrate" method="POST"><button>Toggle Calibration</button></form>
        <button type="button" onclick="showCalibration()">Show Calibration</button>
        <button type="button" onclick="clearLog()">Clear Log</button>
        <div style="display: flex; align-items: center; gap: 10px;">
          <label for="pwmTest">🧪 PWM Test:</label>
          <input type="range" id="pwmTest" min="0" max="100" value="0" oninput="updatePWMValue(this.value)">
          <span id="pwmValue">0%</span>
          <button onclick="sendPWMValue()" type="button">Apply</button>
        </div>
      </div>
      <div id="log">Loading...</div>
      </body></html>
    )rawliteral";
    server.send(200, "text/html; charset=UTF-8", html);
  });

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
    int pwmPercent = pwm * 100 / 255;

    String json = "{";
    json += "\"ambient\":" + String(t, 1) + ",";
    json += "\"humidity\":" + String(h, 1) + ",";
    json += "\"glass\":" + String(glassTemp, 1) + ",";
    json += "\"delta\":" + String(delta, 2) + ",";
    json += "\"heater\":" + String(heaterEnabled ? "true" : "false") + ",";
    json += "\"pwm\":" + String(pwmPercent);
    json += "}";

    server.send(200, "application/json; charset=UTF-8", json);
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
    sendLog(calibrating ? "🧪 Calibration STARTED" : "✅ Calibration STOPPED");
    if (!calibrating) computeCalibrationFromCSV();
    saveConfig();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/settings", HTTP_POST, []() {
    if (server.hasArg("delta")) targetDelta = server.arg("delta").toFloat();
    if (server.hasArg("humidity")) humidityThreshold = server.arg("humidity").toFloat();
    if (server.hasArg("ssid")) wifiSSID = server.arg("ssid");
    if (server.hasArg("password")) wifiPass = server.arg("password");
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
    sendLog("🧹 Log cleared.");
    server.send(200);
  });

  server.begin();
  sendLog("🌐 Web server started");
}


void sensorTask(void* parameter) {
  sendLog("🧵 Sensor task started on core " + String(xPortGetCoreID()));
  sendLog("🧪 SDA = " + String(I2C_SDA) + ", SCL = " + String(I2C_SCL));
  sendLog("🧪 Using I2C bus: 1");

  I2CBus.begin(I2C_SDA, I2C_SCL);
  delay(10);

  if (!aht.begin(&I2CBus)) {
    sendLog("❌ AHT20 init failed (dedicated bus)");
    vTaskDelete(NULL);
    return;
  }

  sendLog("✅ AHT20 initialized (dedicated bus)");

  for (;;) {
    sensors_event_t h, t;
    aht.getEvent(&h, &t);

    if (!isnan(t.temperature) && !isnan(h.relative_humidity)) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
        ambientTemp = t.temperature;
        humidity = h.relative_humidity;
        xSemaphoreGive(i2cMutex);
      }
//      sendLog("🧪 Task Read: T=" + String(ambientTemp, 1) + "°C, H=" + String(humidity, 1) + "%");
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

void setup() {
  // clear log
  logBuffer = "";

  Serial.begin(115200);
  delay(1200);  // Allow time for Serial to initialize
  sendLog("🚀 Sketch started");

  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 1000);
  analogWrite(PWM_PIN, 0);

  SPIFFS.begin(true);
  loadConfig();
  setupWiFi();
  setupWebServer();

  i2cMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle, 1
  );

  // Give the task time to initialize
  delay(500);  // or 750–1000 ms for extra safety
}


unsigned long lastUpdate = 0;

void loop() {
  server.handleClient();

  if (millis() - lastUpdate >= LOOP_INTERVAL_MS) {
    lastUpdate = millis();
  // Orange LED blinks when wifi in AP mode
    static unsigned long lastLED = 0;
    static bool ledState = false;
    if (WiFi.getMode() == WIFI_AP) {
      if (millis() - lastLED > 500) {
        lastLED = millis();
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
      }
    }

#if SIMULATE_HARDWARE
    ambientTemp = 17.0 + sin(millis() / 12000.0);
    humidity = 78.0 + sin(millis() / 8000.0);
#endif

    float t, h;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
      t = ambientTemp;
      h = humidity;
      xSemaphoreGive(i2cMutex);
    }

    // Now use t, h safely
    glassTemp = readGlassTemp();
    float delta = glassTemp - t;

    if (!pwmTest) {
      pwm = (heaterEnabled && humidity >= humidityThreshold && delta < targetDelta)
            ? map(constrain(targetDelta - delta, 0, 5), 0, 5, 64, 255)
            : 0;
      analogWrite(PWM_PIN, pwm);
    }

/*
    char line[128];
    snprintf(line, sizeof(line), "🌀 T=%5.1f°C | H=%5.1f%% | G=%5.1f°C | Δ=%+6.2f°C | PWM=%3d%% %s",
            t, h, glassTemp, delta, pwmPercent, pwm > 0 ? "🔥" : "🌙");
    sendLog(String(line));
*/
    if (calibrating && abs(t - lastSampledTemp) >= 0.1) {
      float v = readThermistorVoltage();
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_APPEND);
      if (f) {
        f.printf("%.2f,%.3f\n", t, v);
        f.close();
        sendLog("📌 Sample " + String(calCount + 1) + ": T=" + String(t, 2) + " V=" + String(v, 3));
      }
      lastSampledTemp = t;
      calCount++;
      if (calCount >= 40) {
        calibrating = false;
        sendLog("✅ Calibration STOPPED (40 samples)");
        computeCalibrationFromCSV();
      }
    }
  }
}
