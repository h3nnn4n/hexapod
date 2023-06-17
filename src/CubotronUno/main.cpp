/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Config.h>

#include <Arduino.h>
#include <BlinkenLights.h>
#include <Joint.h>
#include <Leg.h>
#include <PWMServoDriver.h>
#include <VectorDatatype.h>
#include <Wire.h>

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca = PWMServoDriver(0x40);

Joint tibia = Joint(&pca, 6);
Joint femur = Joint(&pca, 5);
Joint coxa  = Joint(&pca, 4);

Leg leg = Leg(&coxa, &femur, &tibia);

char buffer[100];
char f_buffer[10];

void update_angles();

void setup() {
#ifdef __DEBUG
    Serial.begin(115200);
    Serial.println("starting");
#endif

    blinkenlights.init();
    blinkenlights.set_update_interval(100);

    pca.begin();
    pca.setPWMFreq(60);

    // FIXME: Is this required?
    // Seems be behave weirdly if not set.
    for (int i = 0; i < 16; i++) {
        pca.writeMicroseconds(i, 1500);
    }

    coxa.set_angle_range(-90.0f, 90.0f);
    femur.set_angle_range(-90.0f, 90.0f);
    tibia.set_angle_range(-90.0f, 90.0f);
}

void loop() {
    blinkenlights.update();

    float angle = sin(millis() / 1000.0f) * 90.0f;
    tibia.set_angle(angle);
    femur.set_angle(angle * 0.35f);
    coxa.set_angle(angle);
}
