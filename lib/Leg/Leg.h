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

class Leg {
  public:
    Leg(Joint *coxa, Joint *femur, Joint *tibia);

    void init();
    void update();
    void enable_servos();
    void disable_servos();

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
    vec3_t get_current_angles();
    float  get_error();

  private:
    Joint *_coxa;
    Joint *_femur;
    Joint *_tibia;

    Joint *_joints[3];

    bool _servos_enabled = false;

    vec3_t _current_position;
    vec3_t _target_position;

    vec3_t _current_angles;

    void move_joints(vec3_t angles);
    void move_joints(float coxa_angle, float femur_angle, float tibia_angle);
};

#endif  // LIB_LEG_LEG_H_
