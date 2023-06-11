/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Config.h>

#include <Arduino.h>
#include <Joint.h>
#include <PWMServoDriver.h>
#include <Wire.h>

int           ledState       = LOW;
const int     ledPin         = LED_BUILTIN;
const int32_t interval       = 50;
uint32_t      previousMillis = 0;
float         test_angle     = 0;
float         angle_step     = 1.0f;

PWMServoDriver pca = PWMServoDriver(0x40);

Joint tibia = Joint(&pca, 6, 550, 2400);
Joint femur = Joint(&pca, 5, 550, 2400);
Joint coxa  = Joint(&pca, 4, 550, 2400);

void blinkenlights();
void update_angles();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

#ifdef __DEBUG
    Serial.begin(115200);
    Serial.println("starting");
#endif

    pca.reset();
    pca.begin();
    pca.setPWMFreq(60);

    // FIXME: Is this required?
    // Seems be behave weirdly if not set.
    for (int i = 0; i < 16; i++) {
        pca.writeMicroseconds(i, 1500);
    }
}

void loop() {
    blinkenlights();

    test_angle += angle_step;

    if (fabs(test_angle) <= 0.01f) {
        update_angles();
        delay(1000);
    }

    if (test_angle >= 90.0f || test_angle <= -90.0f) {
        update_angles();
        angle_step *= -1;
        delay(1000);
    }
}

void update_angles() {
    tibia.set_angle(test_angle * 0.85);
    femur.set_angle(-test_angle / 2.0);
    coxa.set_angle(test_angle / 3.0);
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
