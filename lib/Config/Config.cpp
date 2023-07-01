/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "Config.h"

#include <stdint.h>

const float coxa_min_angle  = -90.0f;
const float coxa_max_angle  = 90.0f;
const float femur_min_angle = -90.0f;
const float femur_max_angle = 90.0f;
const float tibia_min_angle = 0.0f;
const float tibia_max_angle = 180.0f;

const float coxa_start_angle  = 0.0f;
const float femur_start_angle = 80.0f;
const float tibia_start_angle = 170.0f;

const float default_tolerance = 1.0f;

const uint16_t pca_pwm_freq = 60;
