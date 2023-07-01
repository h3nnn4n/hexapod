/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_CONFIG_CONFIG_H_
#define LIB_CONFIG_CONFIG_H_

#include <stdint.h>

extern const float coxa_min_angle;
extern const float coxa_max_angle;
extern const float femur_min_angle;
extern const float femur_max_angle;
extern const float tibia_min_angle;
extern const float tibia_max_angle;

extern const float coxa_start_angle;
extern const float femur_start_angle;
extern const float tibia_start_angle;

extern const float default_tolerance;

extern const uint16_t pca_pwm_freq;

#define N_LEGS 6

#define PCA9685_ADDRESS_1 0x40
#define PCA9685_ADDRESS_2 0x41

#endif  // LIB_CONFIG_CONFIG_H_
