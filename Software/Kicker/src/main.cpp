#include <Arduino.h>
#define KICKER_PIN 14

void setup() {
  pinMode(KICKER_PIN, OUTPUT);
}

void loop() {
  digitalWrite(KICKER_PIN, HIGH);
  delay(100);
  digitalWrite(KICKER_PIN, LOW);
  delay(3000);
}