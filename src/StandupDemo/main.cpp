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

const float coxa_start_angle  = 0.0f;
const float femur_start_angle = 80.0f;
const float tibia_start_angle = 160.0f;

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca = PWMServoDriver(0x40);

Joint coxa1  = Joint(&pca, 4);
Joint femur1 = Joint(&pca, 5);
Joint tibia1 = Joint(&pca, 6);

Joint coxa2  = Joint(&pca, 8);
Joint femur2 = Joint(&pca, 9);
Joint tibia2 = Joint(&pca, 10);

Joint coxa3  = Joint(&pca, 12);
Joint femur3 = Joint(&pca, 13);
Joint tibia3 = Joint(&pca, 14);

Leg leg1 = Leg(&coxa1, &femur1, &tibia1);
Leg leg2 = Leg(&coxa2, &femur2, &tibia2);
Leg leg3 = Leg(&coxa3, &femur3, &tibia3);

void setup() {
    blinkenlights.init();
    blinkenlights.set_update_interval(50);

    pca.begin();
    pca.setPWMFreq(60);

    coxa1.set_angle_range(-90.0f, 90.0f);
    femur1.set_angle_range(-90.0f, 90.0f);
    tibia1.set_angle_range(0.0f, 180.0f);

    coxa2.set_angle_range(-90.0f, 90.0f);
    femur2.set_angle_range(-90.0f, 90.0f);
    tibia2.set_angle_range(0.0f, 180.0f);

    coxa3.set_angle_range(-90.0f, 90.0f);
    femur3.set_angle_range(-90.0f, 90.0f);
    tibia3.set_angle_range(0.0f, 180.0f);

    leg1.init();
    leg1.enable_servos();
    leg1.set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
    leg1.update();

    leg2.init();
    leg2.enable_servos();
    leg2.set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
    leg2.update();

    leg3.init();
    leg3.enable_servos();
    leg3.set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
    leg3.update();
}

void loop() {
    blinkenlights.update();
    //
}
