/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_BLINKENLIGHTS_BLINKENLIGHTS_H_
#define LIB_BLINKENLIGHTS_BLINKENLIGHTS_H_

#include <Arduino.h>

#include <stdint.h>

class BlinkenLights {
  public:
    BlinkenLights();
    BlinkenLights(uint8_t led_pin);

    void init();
    void update();
    void set_update_interval(uint16_t interval);

  private:
    uint8_t _pin;
    bool    state = LOW;

    uint32_t _last_update = 0;
    uint16_t _interval    = 1000;
};

#endif  // LIB_BLINKENLIGHTS_BLINKENLIGHTS_H_
