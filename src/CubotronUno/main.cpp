/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const int     servoPin       = 9;
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

Servo servo;

void blinkenlights();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    servo.attach(servoPin, servo_min_pulse_width, servo_max_pulse_width);
}

void loop() {
    blinkenlights();

    for (pos = min_angle; pos <= max_angle; pos += 1) {
        servo.write(pos);
        delay(servo_update_delay);
    }
    delay(500);

    for (pos = max_angle; pos >= min_angle; pos -= 1) {
        servo.write(pos);
        delay(servo_update_delay);
    }
    delay(500);
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
