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

#include <AUnit.h>

char buffer[100];

Leg leg = Leg(nullptr, nullptr, nullptr);

test(foo) { assertEqual(2 + 2, 4); }

test(fk) {
    vec3_t result = leg.forward_kinematics(0, 0, 0);

    serial_println_vec3(result);

    assertEqual(result.x, 0.0);
    assertEqual(result.y, 300.0);
    assertEqual(result.z, 0.0);
}

void setup() {
    Serial.begin(115200);

    leg.disable_servos();
}

void loop() { aunit::TestRunner::run(); }
