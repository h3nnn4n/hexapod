/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_JOINT_JOINT_H_
#define LIB_JOINT_JOINT_H_

#include <stdint.h>

#include <Config.h>

#include <PWMServoDriver.h>

// Values from mg995 servos that I have
#define MIN_US_LIMIT 550
#define MAX_US_LIMIT 2400

class Joint {
  public:
    Joint(PWMServoDriver *servo_driver, uint8_t servo_id);
    Joint(PWMServoDriver *servo_driver, uint8_t servo_id, uint16_t min_us, uint16_t max_us);

    void set_angle(float angle);

    void set_angle_range(float min_angle, float max_angle);

    float get_angle();

    float min_angle = -90;
    float max_angle = 90;

  private:
    float _current_angle = -999;

    PWMServoDriver *_servo_driver;
    uint8_t         _servo_id;

    uint16_t _min_us = MIN_US_LIMIT;
    uint16_t _max_us = MAX_US_LIMIT;

    void write_us_to_servo(uint16_t us);
};

#endif  // LIB_JOINT_JOINT_H_
