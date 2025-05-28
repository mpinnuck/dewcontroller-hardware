#define TEST_PIN    7    // Set to the GPIO pin you want to probe (e.g. 6, 9, 10, 11)
#define IS_PWM      true    // Set true to output PWM, false for digital toggle
#define IS_ANALOG   false   // Set true to read analog input instead

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.printf("📡 Starting pin probe on GPIO %d\n", TEST_PIN);

  if (IS_ANALOG) {
    pinMode(TEST_PIN, INPUT);
  } else {
    pinMode(TEST_PIN, OUTPUT);
    if (IS_PWM) {
      analogWriteFrequency(TEST_PIN, 1000);  // 1 kHz PWM
      analogWrite(TEST_PIN, 127);            // 50% duty
      Serial.println("PWM enabled (1kHz, 50%)");
    }
  }
}

void loop() {
  if (IS_ANALOG) {
    int value = analogRead(TEST_PIN);
    float voltage = value * 3.3 / 4095.0;
    Serial.printf("🔍 Analog Read GPIO %d = %d → %.2f V\n", TEST_PIN, value, voltage);
    delay(1000);
  } else if (!IS_PWM) {
    digitalWrite(TEST_PIN, HIGH);
    delay(250);
    digitalWrite(TEST_PIN, LOW);
    delay(250);
  } else {
    // PWM mode: do nothing — pin is automatically driven
    delay(1000);
  }
}