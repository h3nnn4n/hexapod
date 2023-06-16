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

#include <AUnit.h>

test(foo) {
    int x = 1;
    assertEqual(x, 1);
}

void setup() { Serial.begin(115200); }

void loop() { aunit::TestRunner::run(); }
