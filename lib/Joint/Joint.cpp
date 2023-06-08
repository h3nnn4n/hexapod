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
    // FIXME: Many of these could be pre computed and store
    uint16_t angle_range = max_angle - min_angle;
    uint16_t us_range    = max_us - min_us;
    uint16_t middle_us   = us_range / 2;

    double us_per_angle = us_range / angle_range;

    uint16_t us = min_us + middle_us + angle * us_per_angle;

#ifdef __DEBUG
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