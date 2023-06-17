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

char f_buffer1[16];
char f_buffer2[16];

Leg leg = Leg(nullptr, nullptr, nullptr);

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

    leg.disable_servos();
}

void loop() {
    {
        uint32_t n_iters = 10;
        uint32_t start   = micros();
        fk(n_iters);
        uint32_t end       = micros();
        uint32_t elapsed   = end - start;
        float    elapsed_s = elapsed / 1000000.0f;
        float    ips       = float(n_iters * 10) / elapsed_s;

        dtostrf(elapsed_s, 8, 3, f_buffer1);
        dtostrf(ips, 6, 2, f_buffer2);

        Serial.print("fk took: ");
        Serial.print(f_buffer1);
        Serial.print("s  ips: ");
        Serial.print(f_buffer2);
        Serial.println();
    }

    //

    {
        uint32_t n_iters = 10;
        uint32_t start   = micros();
        ik(n_iters);
        uint32_t end       = micros();
        uint32_t elapsed   = end - start;
        float    elapsed_s = elapsed / 1000000.0f;
        float    ips       = float(n_iters) / elapsed_s;

        dtostrf(elapsed_s, 8, 3, f_buffer1);
        dtostrf(ips, 6, 2, f_buffer2);

        Serial.print("ik took: ");
        Serial.print(f_buffer1);
        Serial.print("s  ips: ");
        Serial.print(f_buffer2);
        Serial.println();

        Serial.println();
    }
}
