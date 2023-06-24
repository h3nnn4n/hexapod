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

#define WIRE Wire

BlinkenLights blinkenlights = BlinkenLights();

void setup() {
    WIRE.begin();

    Serial.begin(9600);

    while (!Serial) {
        delay(10);
    }

    Serial.println("\nI2C Scanner");

    blinkenlights.init();
    blinkenlights.set_update_interval(50);
}

void loop() {
    blinkenlights.update();

    int nDevices = 0;

    Serial.println("Scanning...");

    for (byte address = 1; address < 127; address++) {
        WIRE.beginTransmission(address);
        byte error = WIRE.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }

            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" I2C devices\n");
    }

    delay(1000);
}
