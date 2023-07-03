/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_LEG_LEG_H_
#define LIB_LEG_LEG_H_

#include <stdint.h>

#include <Arduino.h>

#include <Joint.h>
#include <VectorDatatype.h>

enum LegMode {
    INSTANTANEOUS,   // Jump to the target position as fast as posible
    CONSTANT_SPEED,  // Moves to the target position at a constant speed
};

class Leg {
  public:
    Leg(Joint *coxa, Joint *femur, Joint *tibia);
    Leg(uint8_t id, Joint *coxa, Joint *femur, Joint *tibia);

    void init();
    void update();
    void enable_servos();
    void disable_servos();

    void set_leg_base_angle(float base_angle);
    void set_leg_mode(LegMode mode);
    void set_leg_speed(float speed);
    void set_leg_move_time(float move_time);

    void set_tolerance(float tolerance);
    void set_flip_axis(bool flip_axis);

    void set_target_foot_position(vec3_t feet_position);
    void set_target_foot_position(float x, float y, float z);

    void set_joint_angles(vec3_t angles);
    void set_joint_angles(float coxa_angle, float femur_angle, float tibia_angle);

    vec3_t forward_kinematics(vec3_t angles);
    vec3_t forward_kinematics(float coxa_angle, float femur_angle, float tibia_angle);

    vec3_t inverse_kinematics(vec3_t feet_position);
    vec3_t inverse_kinematics(float feet_x, float feet_y, float feet_z);

    vec3_t get_current_position();
    vec3_t get_target_position();
    vec3_t get_final_position();
    vec3_t get_current_angles();

    float get_error();
    float get_reach();
    float get_speed();
    float get_tolerance();

    LegMode get_mode();

    uint8_t get_id();

    void set_timeout(uint16_t timeout_ms);

  private:
    uint8_t _id = 1 << 8;

    Joint *_coxa;
    Joint *_femur;
    Joint *_tibia;

    bool _servos_enabled = false;
    bool _flip_axis      = false;

    float _base_angle = 0.0f;

    vec3_t _current_position;
    vec3_t _target_position;
    vec3_t _final_position;

    vec3_t _current_angles;

    LegMode _mode      = INSTANTANEOUS;
    float   _speed     = 0.0f;
    float   _move_time = 0.0f;

    float    _tolerance  = 1.0f;
    uint16_t _timeout_ms = 50;

    uint64_t _last_update  = 0;
    uint64_t _last_time    = 0;
    uint64_t _now          = 0;
    float    _delta        = 0.0f;
    uint16_t _update_timer = 25;

    void move_joints(vec3_t angles);
    void move_joints(float coxa_angle, float femur_angle, float tibia_angle);

    void _update_instantaneous();
    void _update_constant_speed();
};

#endif  // LIB_LEG_LEG_H_
