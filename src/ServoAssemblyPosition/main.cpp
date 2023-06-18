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
#include <Utils.h>
#include <VectorDatatype.h>
#include <Wire.h>

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca = PWMServoDriver(0x40);

Joint coxa  = Joint(&pca, 4);
Joint femur = Joint(&pca, 5);
Joint tibia = Joint(&pca, 6);

Leg leg = Leg(&coxa, &femur, &tibia);

void setup() {
    blinkenlights.init();
    blinkenlights.set_update_interval(100);

    pca.begin();
    pca.setPWMFreq(60);

    coxa.set_angle_range(-90.0f, 90.0f);
    femur.set_angle_range(-90.0f, 90.0f);
    tibia.set_angle_range(0.0f, 180.0f);

    leg.init();
    leg.set_joint_angles(0, 90, 180);
    leg.enable_servos();
    leg.update();
}

void loop() { blinkenlights.update(); }
