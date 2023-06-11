/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_JOINT_JOINT_H_
#define LIB_JOINT_JOINT_H_

#include <Config.h>

#include <PWMServoDriver.h>

#define MIN_US_LIMIT 300
#define MAX_US_LIMIT 2750

class Joint {
  public:
    Joint(PWMServoDriver *servo_driver, uint8_t servo_id);
    Joint(PWMServoDriver *servo_driver, uint8_t servo_id, uint16_t min_us, uint16_t max_us);

    void set_angle(float angle);

  private:
    float _current_angle = 0;

    PWMServoDriver *_servo_driver;
    uint8_t         _servo_id;

    // Sane defaults that should always be safe
    uint16_t _min_us = 1000;
    uint16_t _max_us = 2000;

    // These are the angle limits in the servo, not an arbitrary design constaint
    const float _servo_min_angle = -90;
    const float _servo_max_angle = 90;

    void write_us_to_servo(uint16_t us);
};

#endif  // LIB_JOINT_JOINT_H_
