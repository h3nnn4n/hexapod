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

char buffer[400];
char f_buffer[10];

vec3_t positions[] = {
    {0, 290, 0}, {0, 250, 0}, {0, 250, 50}, {0, 250, -50}, {0, 250, 0},
};

uint16_t current_position_i = 0;
uint16_t target_position_i  = 1;

unsigned long long last_time = 0;
unsigned long long now       = 0;

int32_t  timer                  = 0;
uint32_t time_between_positions = 2000;

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

    now               = millis();
    last_time         = millis();
    timer             = time_between_positions;
    target_position_i = current_position_i + 1;
}

void loop() {
    blinkenlights.update();

    now        = millis();
    auto delta = now - last_time;
    last_time  = now;

    timer -= delta;

    // snprintf(buffer, sizeof(buffer), "timer: %4d  delta: %4d  now: %ld  last_time:  %ld", timer, delta, now,
    // last_time); Serial.println(buffer);

    if (timer <= 0) {
        Serial.println("\nmoving to new position");

        timer = time_between_positions;

        current_position_i++;

        if (current_position_i >= sizeof(positions) / sizeof(vec3_t)) {
            current_position_i = 0;
        }

        target_position_i = current_position_i + 1;

        if (target_position_i >= sizeof(positions) / sizeof(vec3_t)) {
            target_position_i = 0;
        }
    }

    float p = (float)timer / (float)time_between_positions;
    p       = 1.0f - p;

    vec3_t target_position = vec3_lerp(positions[current_position_i], positions[target_position_i], p);

    leg.set_target_foot_position(target_position);
    leg.update();

    snprintf(buffer, sizeof(buffer), "timer: %4d  ", timer);
    Serial.print(buffer);
    Serial.print("  ");
    Serial.print("p: ");
    Serial.print(p);
    Serial.print("  ");
    Serial.print(current_position_i);
    Serial.print("/");
    Serial.print(target_position_i);
    Serial.print("  |  current: ");
    serial_print_vec3(leg.get_current_position());
    Serial.print("  |  target: ");
    serial_print_vec3(leg.get_target_position());
    Serial.print("  |  angles: ");
    serial_print_vec3(leg.get_current_angles());
    Serial.print("  |  error: ");
    Serial.print(leg.get_error());
    Serial.print("  |  reach: ");
    Serial.print(leg.get_reach());
    Serial.println();
}
