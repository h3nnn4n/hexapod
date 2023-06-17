/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include "Joint.h"

#ifdef __DEBUG
char buffer[100];
char f_buffer[10];
#endif

Joint::Joint(PWMServoDriver *servo_driver, uint8_t servo_id) {
    _servo_driver = servo_driver;
    _servo_id     = servo_id;
}

Joint::Joint(PWMServoDriver *servo_driver, uint8_t servo_id, uint16_t min_us, uint16_t max_us) {
    _servo_driver = servo_driver;
    _servo_id     = servo_id;

    _min_us = constrain(min_us, MIN_US_LIMIT, MAX_US_LIMIT);
    _max_us = constrain(max_us, MIN_US_LIMIT, MAX_US_LIMIT);
}

float Joint::get_angle() { return _current_angle; }

void Joint::set_angle_range(float min_angle, float max_angle) {
    this->min_angle = min_angle;
    this->max_angle = max_angle;
}

void Joint::set_angle(float angle) {
    angle = constrain(angle, min_angle, max_angle);

    // TODO(h3nnn4n): We could have a small delta so we don't send tiny updates
    // that are below the precision of the servo
    if (angle == _current_angle)
        return;

    uint16_t us = map(angle, min_angle, max_angle, _min_us, _max_us);

#ifdef __DEBUG
    snprintf(buffer, sizeof(buffer), "servo_id=%3d ", _servo_id);
    Serial.write(buffer);
    dtostrf(angle, 6, 2, f_buffer);
    snprintf(buffer, sizeof(buffer), "angle=%s ", f_buffer);
    Serial.write(buffer);
    dtostrf(min_angle, 6, 2, f_buffer);
    snprintf(buffer, sizeof(buffer), " min_angle=%s ", f_buffer);
    Serial.write(buffer);
    dtostrf(max_angle, 6, 2, f_buffer);
    snprintf(buffer, sizeof(buffer), "max_angle=%s  ", f_buffer);
    Serial.write(buffer);
    snprintf(buffer, sizeof(buffer), "us=%d\n", us);
    Serial.write(buffer);
#endif

    _current_angle = angle;

    write_us_to_servo(us);
}

void Joint::write_us_to_servo(uint16_t us) { _servo_driver->writeMicroseconds(_servo_id, us); }
