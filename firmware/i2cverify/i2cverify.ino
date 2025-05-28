#include <Wire.h>

#define I2C_SDA 5  // D4, GPIO 5, Pin 5
#define I2C_SCL 6  // D5, GPIO 6, Pin 6

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("📡 Starting I2C Clock Pulse Test");
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  // Send dummy transmission to generate clock pulses
  Wire.beginTransmission(0x50);  // Arbitrary address, no device required
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("📶 Sent I2C start/stop sequence");
  delay(1000);  // Observe 1Hz pulse train on SCL with CRO
}
