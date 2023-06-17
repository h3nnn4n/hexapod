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

    void move_feet_to(vec3_t feet_position);
    void move_feet_to(float x, float y, float z);

    void move_joints(vec3_t angles);
    void move_joints(float coxa_angle, float femur_angle, float tibia_angle);

    vec3_t forward_kinematics(float coxa_angle, float femur_angle, float tibia_angle);

    vec3_t inverse_kinematics(vec3_t feet_position);
    vec3_t inverse_kinematics(float feet_x, float feet_y, float feet_z);

  private:
    Joint *_coxa;
    Joint *_femur;
    Joint *_tibia;

    Joint *_joints[3];

    bool _servos_enabled = false;
};

#endif  // LIB_LEG_LEG_H_
