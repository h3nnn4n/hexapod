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

    leg.enable_servos();
    leg.init();

    delay(250);
}

void loop() {
    blinkenlights.update();

    button_down = digitalRead(2);
    val_x       = analogRead(A0);
    val_y       = analogRead(A1);
    val_z       = analogRead(A2);

    Serial.print(button_down);

    if (ik_mode) {
        feet_x = map(val_x, 0, 1023, -300, 300);
        feet_y = map(val_y, 0, 1023, 0, 300);
        feet_z = map(val_z, 0, 1023, -300, 300);

        Serial.print(" ");
        Serial.print(feet_x);
        Serial.print(" ");
        Serial.print(feet_y);
        Serial.print(" ");
        Serial.print(feet_z);
    } else {
        coxa_angle  = map(val_z, 0, 1023, -90, 90);
        femur_angle = map(val_x, 0, 1023, -90, 90);
        tibia_angle = map(val_y, 0, 1023, 0, 180);

        Serial.print(" ");
        Serial.print(coxa_angle);
        Serial.print(" ");
        Serial.print(femur_angle);
        Serial.print(" ");
        Serial.print(tibia_angle);
    }

    Serial.println();

    if (button_down) {
        if (ik_mode) {
            leg.move_feet_to(feet_x, feet_y, feet_z);
        } else {
            leg.move_joints(coxa_angle, femur_angle, tibia_angle);
        }
    } else {
        leg.move_feet_to(0, 300, 0);
    }
}
