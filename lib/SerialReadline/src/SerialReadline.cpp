/*
 * serial-readline.cpp - Library for buffered serial line reading
 * Created by MSZ, March 3, 2022.
 * Released into the public domain under MIT Licence.
 * Copyright (c) 2022 Zawisza (MSZ98)
 */

#include "SerialReadline.h"

#include <Arduino.h>

int a = 0;

void SerialLineReader::poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (buffer_len >= buffer_limit) {
            Serial.println("WARN: Buffer full");
            return;
        }

        buffer[buffer_len++] = c;
        if (c == '\n') {
            buffer[buffer_len - 1] = 0;
            char *line             = new char[buffer_len];
            strcpy(line, buffer);
            queue.add(line);
            buffer_len = 0;
        }

        if (!queue.isEmpty() && isr != NULL) {
            char *line = queue.get();
            isr(line);
            delete line;
        }
    }
}

void SerialLineReader::read(char *line) {
    char *l = queue.get();
    strcpy(line, l);
    delete l;
}
