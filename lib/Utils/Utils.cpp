/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include "Utils.h"

#include <math.h>

#include <VectorDatatype.h>

const size_t BUFFER_SIZE = 100;

char f_buffer_x[10];
char f_buffer_y[10];
char f_buffer_z[10];

char _buffer[BUFFER_SIZE];

void print_vec3(char *buffer, size_t n, vec3_t v) {
    dtostrf(v.x, 7, 2, f_buffer_x);
    dtostrf(v.y, 7, 2, f_buffer_y);
    dtostrf(v.z, 7, 2, f_buffer_z);

    snprintf(buffer, n, "x=%s y=%s z=%s", f_buffer_x, f_buffer_y, f_buffer_z);
}

void serial_print_vec3(vec3_t v) {
    print_vec3(_buffer, BUFFER_SIZE, v);
    Serial.write(_buffer);
}

void serial_println_vec3(vec3_t v) {
    serial_print_vec3(v);
    Serial.write("\n");
}

float degree_to_radian(float degree) { return degree * (M_PI / 180.0f); }

float radian_to_degree(float radian) { return radian * (180.0f / M_PI); }

uint16_t count_char_in_string(char *str, char c) {
    uint16_t count = 0;

    for (uint16_t i = 0; i < strlen(str); i++) {
        if (str[i] == c) {
            count++;
        }
    }

    return count;
}

uint16_t count_char_in_string(String *str, char c) {
    uint16_t count = 0;

    for (uint16_t i = 0; i < str->length(); i++) {
        if (str[i] == c) {
            count++;
        }
    }

    return count;
}
