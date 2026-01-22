#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("T-Beam S3 boot");
}

void loop() {
  delay(1000);
}
