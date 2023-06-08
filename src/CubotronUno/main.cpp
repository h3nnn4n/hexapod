/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>
#include <Joint.h>
#include <PWMServoDriver.h>
#include <Wire.h>

int           ledState       = LOW;
const int     ledPin         = LED_BUILTIN;
const int32_t interval       = 100;
uint32_t      previousMillis = 0;
float         test_angle     = 0;
float         angle_step     = 0.5f;

PWMServoDriver pca = PWMServoDriver(0x40);

Joint tibia      = Joint(&pca, 13);
Joint femur      = Joint(&pca, 14);
Joint coxa       = Joint(&pca, 15);
Joint test_joint = Joint(&pca, 0, 1000 - 575, 2000 + 417);

void blinkenlights();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

#ifdef __DEBUG
    Serial.begin(9600);
    Serial.println("starting");
#endif

    pca.begin();
    pca.setPWMFreq(60);
}

void loop() {
    blinkenlights();

    tibia.set_angle(0);
    femur.set_angle(0);
    coxa.set_angle(0);

    test_angle += angle_step;

    if (fabs(test_angle) < 0.01f) {
        delay(1000);
    }

    if (test_angle > 90.0f || test_angle < -90.0f) {
        angle_step *= -1;
        delay(1000);
    }

    test_joint.set_angle(test_angle);
}

void blinkenlights() {
    // TODO(h3nnn4n): Move to a lib
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
