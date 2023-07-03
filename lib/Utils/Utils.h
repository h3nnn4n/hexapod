/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#ifndef LIB_UTILS_UTILS_H_
#define LIB_UTILS_UTILS_H_

#include <VectorDatatype.h>

void print_vec3(char *buffer, vec3_t v);
void serial_print_vec3(vec3_t v);
void serial_println_vec3(vec3_t v);

float degree_to_radian(float degree);
float radian_to_degree(float radian);

uint16_t count_char_in_string(char *str, char c);
uint16_t count_char_in_string(String *str, char c);

#endif  // LIB_UTILS_UTILS_H_
