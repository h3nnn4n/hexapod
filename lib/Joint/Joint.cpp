/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "Joint.h"

#ifdef __DEBUG
char buffer[100];
char f_buffer[10];
#endif

Joint::Joint(PWMServoDriver *servo_driver, uint8_t servo_id) {
    _servo_driver = servo_driver;
    _servo_id     = servo_id;
}

void Joint::set_angle(float angle) {
    angle = constrain(angle, servo_min_angle, servo_max_angle);

    // TODO(h3nnn4n): We could have a small delta so we don't send tiny updates
    // that are below the precision of the servo
    if (angle == _current_angle)
        return;

    // FIXME: Many of these could be pre computed and stored
    uint16_t angle_range = servo_max_angle - servo_min_angle;
    uint16_t us_range    = max_us - min_us;
    uint16_t middle_us   = us_range / 2;

    double us_per_angle = us_range / angle_range;

    uint16_t us = min_us + middle_us + angle * us_per_angle;

#ifdef __DEBUG
    snprintf(buffer, sizeof(buffer), "servo_id=%3d ", _servo_id);
    Serial.write(buffer);
    dtostrf(angle, 6, 2, f_buffer);
    snprintf(buffer, sizeof(buffer), "angle=%s ", f_buffer);
    Serial.write(buffer);
    dtostrf(us_per_angle, 6, 2, f_buffer);
    snprintf(buffer, sizeof(buffer), "us_per_angle=%s us=%d\n", f_buffer, us);
    Serial.write(buffer);
#endif

    write_us_to_servo(us);
}

void Joint::write_us_to_servo(uint16_t us) { _servo_driver->writeMicroseconds(_servo_id, us); }
