/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Config.h>

#include <Arduino.h>
#include <BlinkenLights.h>
#include <Joint.h>
#include <Leg.h>
#include <PWMServoDriver.h>
#include <Utils.h>
#include <VectorDatatype.h>
#include <Wire.h>

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca = PWMServoDriver(0x40);

Joint tibia = Joint(&pca, 6);
Joint femur = Joint(&pca, 5);
Joint coxa  = Joint(&pca, 4);

Leg leg = Leg(&coxa, &femur, &tibia);

char buffer[100];
char f_buffer[10];

float feet_x = 0;
float feet_y = 250;
float feet_z = 0;

float coxa_angle  = 0;
float femur_angle = 0;
float tibia_angle = 0;

bool button_down = false;
bool ik_mode     = true;

uint16_t val_x;
uint16_t val_y;
uint16_t val_z;

void setup() {
    Serial.begin(115200);
    Serial.println("starting");

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(2, INPUT);

    blinkenlights.init();
    blinkenlights.set_update_interval(50);

    pca.begin();
    pca.setPWMFreq(60);

    coxa.set_angle_range(-90.0f, 90.0f);
    femur.set_angle_range(-90.0f, 90.0f);
    tibia.set_angle_range(0.0f, 180.0f);

    leg.init();
    leg.enable_servos();

    delay(250);
}

void loop() {
    blinkenlights.update();

    vec3_t raw_readings;
    vec3_t target_position = vec3_t(0, 0, 0);
    vec3_t target_angles   = vec3_t(0, 0, 0);

    raw_readings.x = analogRead(A0);
    raw_readings.y = analogRead(A1);
    raw_readings.z = analogRead(A2);

    button_down = digitalRead(2);

    // Serial.print(button_down);

    if (ik_mode) {
        target_position.x = map(raw_readings.x, 0, 1023, -300, 300);
        target_position.y = map(raw_readings.y, 0, 1023, 0, 300);
        target_position.z = map(raw_readings.z, 0, 1023, -300, 300);
    } else {
        target_angles.x = map(raw_readings.z, 0, 1023, -90, 90);
        target_angles.y = map(raw_readings.x, 0, 1023, -90, 90);
        target_angles.z = map(raw_readings.y, 0, 1023, 0, 180);
    }

    Serial.print("current: ");
    serial_print_vec3(leg.get_current_position());
    Serial.print("  |  target: ");
    serial_print_vec3(leg.get_target_position());
    Serial.print("  |  angles: ");
    serial_print_vec3(leg.get_current_angles());
    Serial.print("  |  error: ");
    Serial.print(leg.get_error());

    Serial.println();

    if (button_down) {
        if (ik_mode) {
            leg.set_target_foot_position(target_position);
        } else {
            leg.set_joint_angles(target_angles);
        }
    } else {
        leg.set_joint_angles(0, 0, 0);
    }

    leg.update();
}
