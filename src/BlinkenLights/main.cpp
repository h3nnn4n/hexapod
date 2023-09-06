/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <Arduino.h>

#include <BlinkenLights.h>

BlinkenLights blinkenlights = BlinkenLights();


void setup() {
    blinkenlights.init();
    blinkenlights.set_update_interval(100);
}

void loop() {
    blinkenlights.update();
}
