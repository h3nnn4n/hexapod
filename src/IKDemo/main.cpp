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

const size_t buffer_size = 256;

char buffer[buffer_size];
char f_buffer[10];

vec3_t positions[] = {
    // Vertical Square
    //{100, 210, 50},    //
    //{-100, 210, 50},   //
    //{-100, 210, -75},  //
    //{100, 210, -75},   //

    // Triangle walk cycle
    {0, 135, -65},     //
    {-100, 135, -90},  //
    {100, 135, -90},   //
};

uint16_t current_position_i = 0;
uint16_t target_position_i  = 1;

vec3_t current_position = {0, 0, 0};
vec3_t position_delta   = {0, 0, 0};

const float speed              = 100.0f;
float       distance_to_target = 0.0f;

unsigned long long last_time = 0;
unsigned long long now       = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("starting");

    blinkenlights.init();
    blinkenlights.set_update_interval(50);

    pca.begin();
    pca.setPWMFreq(60);

    coxa.set_angle_range(-90.0f, 90.0f);
    femur.set_angle_range(-90.0f, 90.0f);
    tibia.set_angle_range(0.0f, 180.0f);

    leg.init();
    leg.set_tolerance(2.5);
    leg.enable_servos();
    leg.set_joint_angles(0, 90, 180);
    leg.update();

    delay(250);

    now                = millis();
    last_time          = millis();
    target_position_i  = current_position_i + 1;
    current_position   = positions[current_position_i];
    position_delta     = (positions[target_position_i] - positions[current_position_i]).norm() * speed;
    distance_to_target = (positions[target_position_i] - positions[current_position_i]).mag();
}

void loop() {
    blinkenlights.update();

    now         = millis();
    float delta = (now - last_time) / 1000.0f;
    last_time   = now;

    if (distance_to_target <= 0) {
        Serial.println("\nmoving to new position");

        current_position_i++;

        if (current_position_i >= sizeof(positions) / sizeof(vec3_t)) {
            current_position_i = 0;
        }

        target_position_i = current_position_i + 1;

        if (target_position_i >= sizeof(positions) / sizeof(vec3_t)) {
            target_position_i = 0;
        }

        position_delta     = (positions[target_position_i] - positions[current_position_i]).norm() * speed;
        current_position   = positions[current_position_i];  // To fix any drift
        distance_to_target = (positions[target_position_i] - positions[current_position_i]).mag();
    }

    vec3_t step = position_delta * delta;
    distance_to_target -= step.mag();
    current_position += step;

    leg.set_target_foot_position(current_position);
    leg.update();

    snprintf(buffer, buffer_size, "dt:%4d ms", uint16_t(delta * 1000.0f));
    Serial.print(buffer);
    Serial.print(" ");
    Serial.print(current_position_i);
    Serial.print("/");
    Serial.print(target_position_i);
    Serial.print("  |  step: ");
    serial_print_vec3(step);
    Serial.print("  |  current: ");
    serial_print_vec3(leg.get_current_position());
    // Serial.print("  |  target: ");
    // serial_print_vec3(leg.get_target_position());
    // Serial.print("  |  angles: ");
    // serial_print_vec3(leg.get_current_angles());
    Serial.print("  |  error: ");
    Serial.print(leg.get_error());
    Serial.print("  |  reach: ");
    Serial.print(leg.get_reach());
    Serial.println();
}
