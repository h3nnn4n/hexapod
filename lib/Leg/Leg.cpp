/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "leg.h"

#include <VectorDatatype.h>

#define COXA_LENGTH  100
#define FEMUR_LENGTH 100
#define TIBIA_LENGTH 100

Leg::Leg(Joint *coxa, Joint *femur, Joint *tibia) {
    _coxa  = coxa;
    _femur = femur;
    _tibia = tibia;

    _joints[0] = _coxa;
    _joints[1] = _femur;
    _joints[2] = _tibia;
}

void Leg::init() {
    // Nothing?
}

void Leg::update() {
    // Nothing?
}

// From https://www.ros.org/reps/rep-0103.html
//
// x forward
// y left
// z up

vec3_t Leg::forward_kinematics(float coxa_angle, float femur_angle, float tibia_angle) {
    vec3_t origin = vec3_t(0, 0, 0);

    float top_view_length = COXA_LENGTH + FEMUR_LENGTH * cos(femur_angle) + TIBIA_LENGTH * cos(tibia_angle);

    vec3_t coxa_position = vec3_t(COXA_LENGTH * sin(coxa_angle), COXA_LENGTH * cos(coxa_angle), 0);

    vec3_t femur_position = vec3_t(coxa_position.x, coxa_position.y + FEMUR_LENGTH, FEMUR_LENGTH * sin(femur_angle));
    vec3_t tibia_position = vec3_t(0, 0, 0);

    float knee_height     = femur_position.z + FEMUR_LENGTH * sin(femur_angle);
    float fee_height_loss = 0;

    vec3_t feet_position = vec3_t(top_view_length * sin(coxa_angle), top_view_length * cos(coxa_angle), 0);

    return tibia_position;
}
