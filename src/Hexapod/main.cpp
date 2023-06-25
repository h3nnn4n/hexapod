/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <cstdint>
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

const int n_legs = 6;

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca1 = PWMServoDriver(0x40);
PWMServoDriver pca2 = PWMServoDriver(0x41);

Joint coxa1  = Joint(&pca1, 0);
Joint femur1 = Joint(&pca1, 1);
Joint tibia1 = Joint(&pca1, 2);

Joint coxa2  = Joint(&pca1, 4);
Joint femur2 = Joint(&pca1, 5);
Joint tibia2 = Joint(&pca1, 6);

Joint coxa3  = Joint(&pca1, 8);
Joint femur3 = Joint(&pca1, 9);
Joint tibia3 = Joint(&pca1, 10);

Joint coxa4  = Joint(&pca2, 0);
Joint femur4 = Joint(&pca2, 1);
Joint tibia4 = Joint(&pca2, 2);

Joint coxa5  = Joint(&pca2, 4);
Joint femur5 = Joint(&pca2, 5);
Joint tibia5 = Joint(&pca2, 6);

Joint coxa6  = Joint(&pca2, 8);
Joint femur6 = Joint(&pca2, 9);
Joint tibia6 = Joint(&pca2, 10);

Leg leg1 = Leg(&coxa1, &femur1, &tibia1);
Leg leg2 = Leg(&coxa2, &femur2, &tibia2);
Leg leg3 = Leg(&coxa3, &femur3, &tibia3);
Leg leg4 = Leg(&coxa4, &femur4, &tibia4);
Leg leg5 = Leg(&coxa5, &femur5, &tibia5);
Leg leg6 = Leg(&coxa6, &femur6, &tibia6);

Leg *legs[n_legs] = {
    &leg1, &leg2, &leg3, &leg4, &leg5, &leg6,
};

vec3_t target_position;

void setup() {
    Serial.begin(115200);
    Serial.println("starting...");

    blinkenlights.init();
    blinkenlights.set_update_interval(50);

    pca1.begin();
    pca1.setPWMFreq(60);

    pca2.begin();
    pca2.setPWMFreq(60);

    coxa1.set_angle_range(-90.0f, 90.0f);
    femur1.set_angle_range(-90.0f, 90.0f);
    tibia1.set_angle_range(0.0f, 180.0f);

    coxa2.set_angle_range(-90.0f, 90.0f);
    femur2.set_angle_range(-90.0f, 90.0f);
    tibia2.set_angle_range(0.0f, 180.0f);

    coxa3.set_angle_range(-90.0f, 90.0f);
    femur3.set_angle_range(-90.0f, 90.0f);
    tibia3.set_angle_range(0.0f, 180.0f);

    coxa4.set_angle_range(-90.0f, 90.0f);
    femur4.set_angle_range(-90.0f, 90.0f);
    tibia4.set_angle_range(0.0f, 180.0f);

    coxa5.set_angle_range(-90.0f, 90.0f);
    femur5.set_angle_range(-90.0f, 90.0f);
    tibia5.set_angle_range(0.0f, 180.0f);

    coxa6.set_angle_range(-90.0f, 90.0f);
    femur6.set_angle_range(-90.0f, 90.0f);
    tibia6.set_angle_range(0.0f, 180.0f);

    // XXX: !?
    leg4.set_flip_axis(true);
    leg5.set_flip_axis(true);
    leg6.set_flip_axis(true);

    for (auto leg : legs) {
        leg->init();
        leg->set_tolerance(1.0f);
        leg->set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
        // leg->disable_servos();
        leg->enable_servos();
        leg->update();
    }

    // Two back legs
    // leg1.enable_servos();
    // leg4.enable_servos();

    // Two middle legs
    // leg2.enable_servos();
    // leg5.enable_servos();

    // Two front legs
    // leg3.enable_servos();
    // leg6.enable_servos();

    now       = millis();
    last_time = millis();
}

void loop() {
    blinkenlights.update();

    now         = millis();
    float delta = (now - last_time) / 1000.0f;
    last_time   = now;

    float f = millis() / 1000.0f;

    // float x = 0.0f + cos(f) * 25.0f * 1.0f;
    // float y = 70.0f + sin(f) * 25.0f * 0.0f;
    // float z = -140.0f + sin(f) * 50.0f * 0.0f;

    float x = 0.0f + cos(f) * 25.0f * 1.0f;
    float y = 250.0f + sin(f) * 50.0f * 0.0f;
    float z = 0.0f + sin(f) * 50.0f * 0.0f;

    target_position = vec3_t(x, y, z);
    // target_position = vec3_t(0, 250, 0);

    for (uint_fast8_t i = 0; i < n_legs; i++) {
        auto leg = legs[i];

        if (i == 2 || i == 5) {
            leg->set_target_foot_position(0, 250, 0);
        } else {
            leg->set_target_foot_position(target_position);
        }

        leg->set_target_foot_position(target_position);

        leg->update();
    }

    snprintf(buffer, buffer_size, "dt:%4d ms", uint16_t(delta * 1000.0f));
    Serial.print(buffer);
    Serial.print(" ");
    Serial.print("  |  current: ");
    serial_print_vec3(leg6.get_current_position());
    Serial.print("  |  target: ");
    serial_print_vec3(leg6.get_target_position());
    Serial.print("  |  angles: ");
    serial_print_vec3(leg6.get_current_angles());
    Serial.print("  |  error: ");
    Serial.print(leg6.get_error());
    Serial.print("  |  reach: ");
    Serial.print(leg6.get_reach());
    Serial.println();
}
