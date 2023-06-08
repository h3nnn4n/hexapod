/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>
#include <PWMServoDriver.h>
#include <Wire.h>

int           ledState       = LOW;
const int     ledPin         = LED_BUILTIN;
const int32_t interval       = 100;
uint32_t      previousMillis = 0;

PWMServoDriver pca = PWMServoDriver(0x40);

void blinkenlights();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    pca.begin();
    pca.setPWMFreq(60);
}

void loop() {
    blinkenlights();

    for (int i = 0; i < 16; i++) {
        pca.writeMicroseconds(i, 1500);
    }
}

void blinkenlights() {
    uint32_t currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        if (ledState == LOW) {
            ledState = HIGH;
        } else {
            ledState = LOW;
        }

        digitalWrite(ledPin, ledState);
    }
}
