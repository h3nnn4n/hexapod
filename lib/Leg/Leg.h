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

    void   init();
    void   update();
    vec3_t forward_kinematics(float coxa_angle, float femur_angle, float tibia_angle);

  private:
    Joint *_coxa;
    Joint *_femur;
    Joint *_tibia;

    Joint *_joints[3];
};

#endif  // LIB_LEG_LEG_H_
