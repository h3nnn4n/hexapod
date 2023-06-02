/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const int     ledPin         = LED_BUILTIN;
const int32_t interval       = 100;
uint32_t      previousMillis = 0;
int           ledState       = LOW;
int           pos            = 0;

const int min_angle          = 0;
const int max_angle          = 180;
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

    servo_femur.write(90);
    servo_tibia.write(90);
    servo_coxa.write(90);
}

void loop() {
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
