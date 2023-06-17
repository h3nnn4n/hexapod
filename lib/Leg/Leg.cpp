/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "leg.h"

#include <math.h>

#include <Utils.h>
#include <VectorDatatype.h>

#define COXA_LENGTH  52
#define FEMUR_LENGTH 78.984
#define TIBIA_LENGTH 163.148

Leg::Leg(Joint *coxa, Joint *femur, Joint *tibia) {
    _coxa  = coxa;
    _femur = femur;
    _tibia = tibia;

    _joints[0] = _coxa;
    _joints[1] = _femur;
    _joints[2] = _tibia;
}

void Leg::enable_servos() { _servos_enabled = true; }

void Leg::disable_servos() { _servos_enabled = true; }

void Leg::init() {
    // Nothing?
}

void Leg::update() {
    // Nothing?
}

/**
 * @brief Calculates the forward kinematics for the leg.
 *
 * @param coxa_angle The coxa angle in degrees, from -90 to 90
 * @param femur_angle The femur angle in degrees, from -90 to 90
 * @param tibia_angle The tibia angle in degrees, from -90 to 90
 *
 * @return The feet position as a vec3_t
 */
vec3_t Leg::forward_kinematics(float coxa_angle, float femur_angle, float tibia_angle) {
    // From https://www.ros.org/reps/rep-0103.html
    //
    // x forward
    // y left
    // z up

    coxa_angle  = degree_to_radian(coxa_angle);
    femur_angle = degree_to_radian(femur_angle);
    tibia_angle = degree_to_radian(tibia_angle);

    vec3_t feet_position = vec3_t(0.0f, 0.0f, 0.0f);

    const float leg_length_top_view =
        COXA_LENGTH + cos(femur_angle) * FEMUR_LENGTH + cos(tibia_angle - femur_angle) * TIBIA_LENGTH;

    feet_position.x = sin(coxa_angle) * leg_length_top_view;
    feet_position.y = cos(coxa_angle) * leg_length_top_view;
    feet_position.z = sin(femur_angle) * FEMUR_LENGTH + sin(tibia_angle - femur_angle) * TIBIA_LENGTH;

    return feet_position;
}
