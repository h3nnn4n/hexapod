/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

#define base_angle   90
#define angle_offset 25

const int     ledPin           = LED_BUILTIN;
const int32_t interval         = 100;
uint32_t      previousMillis   = 0;
uint32_t      servoMillis      = 0;
int           ledState         = LOW;
int           servo_direction  = LOW;
float         servo_angle      = base_angle;
float         servo_angle_step = 0.5;

const int min_angle          = base_angle - angle_offset;
const int max_angle          = base_angle + angle_offset;
const int servo_update_delay = 10;

const int servo_min_pulse_width = 500;
const int servo_max_pulse_width = 2500;

Servo servo_femur;
Servo servo_tibia;
Servo servo_coxa;

void blinkenlights();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    servo_femur.attach(9, servo_min_pulse_width, servo_max_pulse_width);
    servo_tibia.attach(10, servo_min_pulse_width, servo_max_pulse_width);
    servo_coxa.attach(11, servo_min_pulse_width, servo_max_pulse_width);

    servo_femur.write(servo_angle);
    servo_tibia.write(servo_angle);
    servo_coxa.write(servo_angle);
}

void loop() {
    uint32_t currentMillis = millis();

    if (currentMillis - servoMillis >= servo_update_delay) {
        servoMillis = currentMillis;

        if (servo_direction == LOW) {
            servo_angle += servo_angle_step;
        } else {
            servo_angle -= servo_angle_step;
        }

        if (servo_angle < min_angle || servo_angle > max_angle) {
            if (servo_direction == LOW) {
                servo_direction = HIGH;
                servo_angle += servo_angle_step;
                servo_angle += servo_angle_step;
            } else {
                servo_direction = LOW;
                servo_angle -= servo_angle_step;
                servo_angle -= servo_angle_step;
            }
        }

        servo_femur.write(servo_angle);
        servo_tibia.write(servo_angle);
        servo_coxa.write(servo_angle);
    }

    blinkenlights();
}

void blinkenlights() {
    uint32_t currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        if (ledState == LOW) {
            ledState = HIGH;
        } else {
            ledState = LOW;
        }

        digitalWrite(ledPin, ledState);
    }
}
