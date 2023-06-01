#include <Arduino.h>

#define RELAY_PIN 19

void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    if (Serial.available() > 0) {
        int state = Serial.read();
        if (state == 1) {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("RELAY HIGH");
        }
        if (state == 0) {
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("RELAY LOW");
        }
    }
    delay(50);
}