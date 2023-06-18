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

Joint min_angle = Joint(&pca, 0);
Joint mid_angle = Joint(&pca, 1);
Joint max_angle = Joint(&pca, 2);

Leg leg        = Leg(&coxa, &femur, &tibia);
Leg leg_angles = Leg(&min_angle, &mid_angle, &max_angle);

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

    min_angle.set_angle_range(-90.0f, 90.0f);
    mid_angle.set_angle_range(-90.0f, 90.0f);
    max_angle.set_angle_range(-90.0f, 90.0f);

    leg_angles.init();
    leg_angles.set_joint_angles(-90, 0, 90);
    leg_angles.enable_servos();
    leg_angles.update();
}

void loop() { blinkenlights.update(); }
