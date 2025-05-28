#include <Wire.h>
#include <Adafruit_AHTX0.h>

// 🔧 Define I2C pins here
#define I2C_SDA 5  // D0 on Seeed XIAO ESP32S3
#define I2C_SCL 6  // D1 on Seeed XIAO ESP32S3

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(115200);
  delay(500);

  

  Serial.println("🔧 Initializing I2C and AHT20 sensor...");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  Serial.println("🔍 Scanning I2C bus...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("✅ Found device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("❌ No I2C devices found.");
  }

  Serial.println("📡 Attempting to initialize AHT20...");
  if (!aht.begin(&Wire)) {
    Serial.println("❌ AHT20 not found. Check wiring.");
    while (1) delay(10);
  } else {
    Serial.println("✅ AHT20 initialized successfully.");
  }
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  Serial.print("🌡 Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" °C, 💧 Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  delay(2000);
}
