/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Config.h>

#include <Arduino.h>
#include <BlinkenLights.h>
#include <Joint.h>
#include <PWMServoDriver.h>
#include <Wire.h>

float test_angle = 0;
float angle_step = 1.0f;

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca = PWMServoDriver(0x40);

Joint tibia = Joint(&pca, 6, 550, 2400);
Joint femur = Joint(&pca, 5, 550, 2400);
Joint coxa  = Joint(&pca, 4, 550, 2400);

void update_angles();

void setup() {
#ifdef __DEBUG
    Serial.begin(115200);
    Serial.println("starting");
#endif

    blinkenlights.init();

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
    blinkenlights.update();

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
