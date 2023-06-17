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

bool button_down = false;

uint16_t val;

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
    tibia.set_angle_range(-90.0f, 90.0f);

    leg.enable_servos();
    leg.init();

    delay(250);
}

void loop() {
    blinkenlights.update();

    val    = analogRead(A0);
    feet_x = map(val, 0, 1023, -300, 300);

    val    = analogRead(A1);
    feet_y = map(val, 0, 1023, 0, 300);

    val    = analogRead(A2);
    feet_z = map(val, 0, 1023, -300, 300);

    button_down = digitalRead(2);

    Serial.print(button_down);
    Serial.print(" ");
    Serial.print(feet_x);
    Serial.print(" ");
    Serial.print(feet_y);
    Serial.print(" ");
    Serial.print(feet_z);
    Serial.println();

    if (button_down) {
        leg.move_feet_to(feet_x, feet_y, feet_z);
    } else {
        leg.move_feet_to(0, 250, 0);
    }
}
