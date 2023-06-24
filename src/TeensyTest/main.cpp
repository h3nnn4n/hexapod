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

char f_buffer1[16];
char f_buffer2[16];

Joint coxa  = Joint(nullptr, 0);
Joint femur = Joint(nullptr, 1);
Joint tibia = Joint(nullptr, 2);

Leg leg = Leg(&coxa, &femur, &tibia);

void fk(uint32_t n_iters) {
    for (uint32_t i = 0; i < n_iters; i++) {
        leg.forward_kinematics(0, 0, 0);

        leg.forward_kinematics(-45, 0, 0);
        leg.forward_kinematics(90, 0, 0);
        leg.forward_kinematics(45, 0, 0);

        leg.forward_kinematics(0, 0, 90);
        leg.forward_kinematics(0, 90, 0);
        leg.forward_kinematics(0, -90, 90);

        leg.forward_kinematics(20, 0, 90);
        leg.forward_kinematics(20, 90, 0);
        leg.forward_kinematics(20, -90, 90);
    }
}

void ik(uint32_t n_iters) {
    for (uint32_t i = 0; i < n_iters; i++) {
        leg.inverse_kinematics(0, 200, 0);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("starting benchmark suite");
    Serial.print("test: ");
    Serial.println(10);

    coxa.set_angle_range(-90.0f, 90.0f);
    femur.set_angle_range(-90.0f, 90.0f);
    tibia.set_angle_range(0.0f, 180.0f);

    leg.init();
    leg.disable_servos();

    blinkenlights.init();
    blinkenlights.set_update_interval(50);
}

void loop() {
    blinkenlights.update();

    {
        uint32_t n_iters = 10;
        uint32_t start   = micros();
        fk(n_iters);
        uint32_t end       = micros();
        uint32_t elapsed   = end - start;
        float    elapsed_s = elapsed / 1000000.0f;
        float    ips       = float(n_iters * 10) / elapsed_s;

        Serial.print("fk took: ");
        Serial.print(elapsed_s);
        Serial.print("s  ips: ");
        Serial.print(ips);
        Serial.println();
    }

    {
        uint32_t n_iters = 10;
        uint32_t start   = micros();
        ik(n_iters);
        uint32_t end       = micros();
        uint32_t elapsed   = end - start;
        float    elapsed_s = elapsed / 1000000.0f;
        float    ips       = float(n_iters) / elapsed_s;

        Serial.print("ik took: ");
        Serial.print(elapsed_s);
        Serial.print("s  ips: ");
        Serial.print(ips);
        Serial.println();

        Serial.println();
    }
}
