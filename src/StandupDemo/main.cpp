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

const size_t buffer_size = 256;
char         buffer[buffer_size];
char         f_buffer[10];

unsigned long long last_time = 0;
unsigned long long now       = 0;

const float coxa_start_angle  = 0.0f;
const float femur_start_angle = 80.0f;
const float tibia_start_angle = 170.0f;

const int n_legs = 3;

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

Leg *legs[n_legs] = {&leg1, &leg2, &leg3};

vec3_t target_position;

void setup() {
    Serial.begin(115200);
    Serial.println("starting...");

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

    for (auto leg : legs) {
        leg->init();
        leg->set_tolerance(2.5);
        leg->set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
        leg->enable_servos();
        leg->update();
    }

    now       = millis();
    last_time = millis();
}

void loop() {
    blinkenlights.update();

    now         = millis();
    float delta = (now - last_time) / 1000.0f;
    last_time   = now;

    float f       = millis() / 1000.0f;
    float offset1 = sin(f) * 50.0f;
    float offset2 = cos(f) * 25.0f * 0.0;

    target_position = vec3_t(0.0f + offset2, 70.0f, -140.0f + offset1);

    leg1.set_target_foot_position(target_position);
    leg1.update();

    leg2.set_joint_angles(leg1.get_current_angles());
    leg3.set_joint_angles(leg1.get_current_angles());

    // for (auto leg : legs) {
    // leg->set_target_foot_position(target_position);
    // leg->update();
    //}

    snprintf(buffer, buffer_size, "dt:%4d ms", uint16_t(delta * 1000.0f));
    Serial.print(buffer);
    Serial.print(" ");
    Serial.print("  |  current: ");
    serial_print_vec3(leg1.get_current_position());
    Serial.print("  |  target: ");
    serial_print_vec3(leg1.get_target_position());
    Serial.print("  |  angles: ");
    serial_print_vec3(leg1.get_current_angles());
    Serial.print("  |  error: ");
    Serial.print(leg1.get_error());
    Serial.print("  |  reach: ");
    Serial.print(leg1.get_reach());
    Serial.println();
}
