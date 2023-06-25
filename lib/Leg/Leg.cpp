/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "Leg.h"

#include <math.h>

#include <Utils.h>
#include <VectorDatatype.h>

#define COXA_LENGTH  52
#define FEMUR_LENGTH 78.984
#define TIBIA_LENGTH 163.148

float feet_position_error(vec3_t feet_position, vec3_t target_position);

Leg::Leg(Joint *coxa, Joint *femur, Joint *tibia) {
    _coxa  = coxa;
    _femur = femur;
    _tibia = tibia;
}

void Leg::enable_servos() {
    _servos_enabled = true;
    move_joints(_current_angles);
}

void Leg::disable_servos() { _servos_enabled = false; }

void Leg::init() {
    set_joint_angles(0.0f, 0.0f, 0.0f);
    _current_position = forward_kinematics(_current_angles);
}

void Leg::update() {
    bool needs_update = get_error() > _tolerance;

    if (needs_update)
        _current_angles = inverse_kinematics(_target_position);

    move_joints(_current_angles);

    if (needs_update)
        _current_position = forward_kinematics(_current_angles);
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
    // FIXME: There is something wrong with the z position (some flipped angle?)
    feet_position.z = sin(femur_angle) * FEMUR_LENGTH - sin(tibia_angle - femur_angle) * TIBIA_LENGTH;

    return feet_position;
}

vec3_t Leg::forward_kinematics(vec3_t angles) { return forward_kinematics(angles.x, angles.y, angles.z); }

/**
 * @brief Calculates the inverse kinematics for the leg.
 *
 * @param target_position The feet position as a vec3_t
 *
 * @return The angles for the coxa, femur and tibia as a vec3_t
 */
vec3_t Leg::inverse_kinematics(vec3_t target_position) {
    // TODO(h3nnn4n): We could store the last feet_position and angles and use it as a starting point

    // This is a very dumb approach to IK. It is basically a brute force search
    // where the Arduino works instead of me deriving the equations.

    // #define __IK_DEBUG

#ifdef __IK_DEBUG
    char f_buffer1[10];
    char f_buffer2[10];
    char f_buffer3[10];
    char _buffer[100];
#endif

    uint64_t start_time = millis();

    const uint16_t n_iters        = 25;
    const uint8_t  n_offsets      = 6;
    const uint8_t  n_nested_iters = 10;
    float          scale          = 0.5f;

    const vec3_t offsets[] = {
        vec3_t(1.0f, 0.0f, 0.0f), vec3_t(-1.0f, 0.0f, 0.0f),  // Coxa
        vec3_t(0.0f, 1.0f, 0.0f), vec3_t(0.0f, -1.0f, 0.0f),  // Femur
        vec3_t(0.0f, 0.0f, 1.0f), vec3_t(0.0f, 0.0f, -1.0f),  // Tibia
    };

    vec3_t angles      = _current_angles;
    vec3_t best_angles = _current_angles;

    // Flip the x axis if the leg is on the right side
    if (_flip_axis) {
        target_position.x = 0.0f - target_position.x;
    }

    float best_error = feet_position_error(_current_position, target_position);
    float error;

#ifdef __IK_DEBUG
    Serial.println("Starting IK");
    Serial.print("starting error: ");
    dtostrf(best_error, 4, 2, f_buffer3);
    Serial.println(f_buffer3);
    Serial.print("target position: ");
    serial_println_vec3(target_position);
    Serial.println();

    // Serial.print("n_iters: ");
    // Serial.print(n_iters);
    // Serial.print("  i_offset: ");
    // Serial.print(n_offsets);
    // Serial.print("  n_nested_iters: ");
    // Serial.println(n_nested_iters);
    // Serial.print("total: ");
    // Serial.println(n_iters * n_offsets * n_nested_iters);
#endif

    for (uint16_t i = 0; i < n_iters; i++) {
        for (uint16_t i_offset = 0; i_offset < n_offsets; i_offset++) {
            for (int j = 1; j <= n_nested_iters; j++) {
#ifdef __IK_DEBUG
                // Serial.print("i: ");
                // Serial.print(i);
                // Serial.print("  i_offset: ");
                // Serial.print(i_offset);
                // Serial.print("  j: ");
                // Serial.println(j);
#endif

                angles = best_angles + offsets[i_offset] * scale * j;

                if (angles.x < _coxa->min_angle || angles.x > _coxa->max_angle) {
                    continue;
                }
                if (angles.y < _femur->min_angle || angles.y > _femur->max_angle) {
                    continue;
                }
                if (angles.z < _tibia->min_angle || angles.z > _tibia->max_angle) {
                    continue;
                }

                vec3_t position = forward_kinematics(angles.x, angles.y, angles.z);
                error           = feet_position_error(position, target_position);

                if (error < best_error) {
#ifdef __IK_DEBUG
                    // Serial.println();
                    // serial_println_vec3(offsets[i_offset]);
                    // serial_println_vec3(position);
                    dtostrf(error, 4, 2, f_buffer1);
                    dtostrf(best_error, 4, 2, f_buffer2);
                    snprintf(_buffer, sizeof(_buffer), "%s %s", f_buffer1, f_buffer2);
                    Serial.println(_buffer);
#endif

                    best_error  = error;
                    best_angles = angles;
                } else {
                    break;
                }

                if (best_error <= _tolerance) {
                    goto end;
                }
            }
        }

        if (millis() - start_time > _timeout_ms)
            goto end;
    }

#ifdef __IK_DEBUG
    Serial.println("giving up");
#endif

end:

#ifdef __IK_DEBUG
    Serial.println();
    dtostrf(error, 4, 2, f_buffer1);
    snprintf(_buffer, sizeof(_buffer), "final error: %s", f_buffer1);
    Serial.print("best angles: ");
    serial_println_vec3(best_angles);
    Serial.println(_buffer);
    Serial.println();
#endif

    return best_angles;
}

/**
 * @brief Calculates the inverse kinematics for the leg.
 *
 * @param feet_x The feet position x position
 * @param feet_y The feet position y position
 * @param feet_z The feet position z position
 *
 * @return The angles for the coxa, femur and tibia as a vec3_t
 */
vec3_t Leg::inverse_kinematics(float feet_x, float feet_y, float feet_z) {
    vec3_t feet_position = vec3_t(feet_x, feet_y, feet_z);
    return inverse_kinematics(feet_position);
}

float feet_position_error(vec3_t feet_position, vec3_t target_position) {
    return sqrt(pow(feet_position.x - target_position.x, 2) + pow(feet_position.y - target_position.y, 2) +
                pow(feet_position.z - target_position.z, 2));
}

void Leg::set_target_foot_position(vec3_t feet_position) { _target_position = feet_position; }

void Leg::set_target_foot_position(float x, float y, float z) { _target_position = vec3_t(x, y, z); }

void Leg::set_joint_angles(vec3_t angles) {
    _current_angles   = angles;
    _target_position  = forward_kinematics(angles);
    _current_position = _target_position;

    move_joints(angles);
}

void Leg::set_joint_angles(float coxa_angle, float femur_angle, float tibia_angle) {
    vec3_t angles = vec3_t(coxa_angle, femur_angle, tibia_angle);
    set_joint_angles(angles);
}

void Leg::move_joints(float coxa_angle, float femur_angle, float tibia_angle) {
    vec3_t angles = vec3_t(coxa_angle, femur_angle, tibia_angle);
    move_joints(angles);
}

void Leg::move_joints(vec3_t angles) {
    if (!_servos_enabled)
        return;

    _coxa->set_angle(angles.x);
    _femur->set_angle(angles.y);
    _tibia->set_angle(angles.z);
}

vec3_t Leg::get_current_position() { return _current_position; }

vec3_t Leg::get_target_position() { return _target_position; }

vec3_t Leg::get_current_angles() { return _current_angles; }

float Leg::get_error() { return feet_position_error(_current_position, _target_position); }

float Leg::get_reach() { return _current_position.mag(); }

void Leg::set_tolerance(float tolerance) { _tolerance = tolerance; }

void Leg::set_timeout(uint16_t timeout_ms) { _timeout_ms = timeout_ms; }

void Leg::set_flip_axis(bool flip_axis) { _flip_axis = flip_axis; }
