/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <Arduino.h>

#include <stdint.h>

#include "BlinkenLights.h"

BlinkenLights::BlinkenLights() { _pin = LED_BUILTIN; }

BlinkenLights::BlinkenLights(uint8_t led_pin) { _pin = led_pin; }

void BlinkenLights::init() { pinMode(_pin, OUTPUT); }

void BlinkenLights::set_update_interval(uint16_t interval) { _interval = interval; }

void BlinkenLights::update() {
    uint32_t current_millis = millis();
    if (current_millis - _last_update < _interval)
        return;

    state = !state;
    digitalWrite(_pin, state);

    _last_update = current_millis;
}
