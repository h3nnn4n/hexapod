/*
 * Copyright (C) 2023 Renan S Silva (aka h3nnn4n)
 *
 */

#include <stdint.h>

#include <Arduino.h>

#include <Config.h>

#include <BlinkenLights.h>
#include <Joint.h>
#include <Leg.h>
#include <PWMServoDriver.h>
#include <SerialReadline.h>
#include <Utils.h>
#include <VectorDatatype.h>
#include <Wire.h>

const size_t buffer_size = 256;
char         buffer[buffer_size];
char         f_buffer[10];

unsigned long long last_time = 0;
unsigned long long now       = 0;

BlinkenLights blinkenlights = BlinkenLights();

PWMServoDriver pca1 = PWMServoDriver(0x40);
PWMServoDriver pca2 = PWMServoDriver(0x41);

Joint coxa1  = Joint(&pca1, 0);
Joint femur1 = Joint(&pca1, 1);
Joint tibia1 = Joint(&pca1, 2);

Joint coxa2  = Joint(&pca1, 4);
Joint femur2 = Joint(&pca1, 5);
Joint tibia2 = Joint(&pca1, 6);

Joint coxa3  = Joint(&pca1, 8);
Joint femur3 = Joint(&pca1, 9);
Joint tibia3 = Joint(&pca1, 10);

Joint coxa4  = Joint(&pca2, 0);
Joint femur4 = Joint(&pca2, 1);
Joint tibia4 = Joint(&pca2, 2);

Joint coxa5  = Joint(&pca2, 4);
Joint femur5 = Joint(&pca2, 5);
Joint tibia5 = Joint(&pca2, 6);

Joint coxa6  = Joint(&pca2, 8);
Joint femur6 = Joint(&pca2, 9);
Joint tibia6 = Joint(&pca2, 10);

Leg leg1 = Leg(&coxa1, &femur1, &tibia1);
Leg leg2 = Leg(&coxa2, &femur2, &tibia2);
Leg leg3 = Leg(&coxa3, &femur3, &tibia3);
Leg leg4 = Leg(&coxa4, &femur4, &tibia4);
Leg leg5 = Leg(&coxa5, &femur5, &tibia5);
Leg leg6 = Leg(&coxa6, &femur6, &tibia6);

Leg *legs[N_LEGS] = {
    &leg1, &leg2, &leg3, &leg4, &leg5, &leg6,
};

vec3_t target_position;

void set_cmd(String cmd);
void parse_read_leg_info_cmd(String cmd);
void parse_leg_position_cmd(String cmd);
void parse_leg_angles_cmd(String cmd);
void parse_leg_speed_cmd(String cmd);
void parse_serial(char *cmd);

SerialLineReader serial_reader(parse_serial);

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    blinkenlights.init();
    blinkenlights.set_update_interval(50);

    pca1.begin();
    pca1.setPWMFreq(pca_pwm_freq);

    pca2.begin();
    pca2.setPWMFreq(pca_pwm_freq);

    coxa1.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur1.set_angle_range(femur_min_angle, femur_max_angle);
    tibia1.set_angle_range(tibia_min_angle, tibia_max_angle);

    coxa2.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur2.set_angle_range(femur_min_angle, femur_max_angle);
    tibia2.set_angle_range(tibia_min_angle, tibia_max_angle);

    coxa3.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur3.set_angle_range(femur_min_angle, femur_max_angle);
    tibia3.set_angle_range(tibia_min_angle, tibia_max_angle);

    coxa4.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur4.set_angle_range(femur_min_angle, femur_max_angle);
    tibia4.set_angle_range(tibia_min_angle, tibia_max_angle);

    coxa5.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur5.set_angle_range(femur_min_angle, femur_max_angle);
    tibia5.set_angle_range(tibia_min_angle, tibia_max_angle);

    coxa6.set_angle_range(coxa_min_angle, coxa_max_angle);
    femur6.set_angle_range(femur_min_angle, femur_max_angle);
    tibia6.set_angle_range(tibia_min_angle, tibia_max_angle);

    // XXX: !?
    leg4.set_flip_axis(true);
    leg5.set_flip_axis(true);
    leg6.set_flip_axis(true);

    leg1.set_leg_base_angle(60.0f);
    leg4.set_leg_base_angle(-60.0f);

    leg3.set_leg_base_angle(-60.0f);
    leg6.set_leg_base_angle(60.0f);

    for (auto leg : legs) {
        leg->init();
        leg->set_tolerance(1.0f);
        leg->set_joint_angles(coxa_start_angle, femur_start_angle, tibia_start_angle);
        leg->disable_servos();
        leg->update();
    }

    now       = millis();
    last_time = millis();
}

void loop() {
    blinkenlights.update();

    now         = millis();
    float delta = (now - last_time) / 1000.0f;
    last_time   = now;

    float f = millis() / 1000.0f;

    serial_reader.poll();

    for (auto leg : legs) {
        leg->update();
    }
}

void parse_serial(char *cmd_str) {
    String cmd = String(cmd_str);
    cmd.trim();

    Serial.println(cmd);

    if (cmd == "ENABLE_SERVOS") {
        for (auto leg : legs) {
            leg->enable_servos();
        }

        Serial.println("OK");
    } else if (cmd == "DISABLE_SERVOS") {
        for (auto leg : legs) {
            leg->disable_servos();
        }

        Serial.println("OK");
    } else if (cmd == "UPDATE") {
        for (auto leg : legs) {
            leg->update();
        }

        Serial.println("OK");
    } else if (cmd == "PING") {
        Serial.println("PONG");
    } else if (cmd.startsWith("READ_LEG_INFO ")) {
        parse_read_leg_info_cmd(cmd.substring(14));
    } else if (cmd.startsWith("SET_LEG_POSITION ")) {
        parse_leg_position_cmd(cmd.substring(17));
    } else if (cmd.startsWith("SET_LEG_ANGLES ")) {
        parse_leg_angles_cmd(cmd.substring(15));
    } else if (cmd.startsWith("SET_LEG_SPEED ")) {
        parse_leg_speed_cmd(cmd.substring(14));
    } else if (cmd.startsWith("SET ")) {
        set_cmd(cmd.substring(4));
    } else {
        Serial.print("unknown command: ");
        Serial.println(cmd);

        Serial.println("ERROR");
    }
}

void parse_read_leg_info_cmd(String cmd) {
    uint_fast8_t leg_index = cmd.toInt();
    auto         leg       = legs[leg_index];

    Serial.print("current_position ");
    serial_println_vec3(leg->get_current_position());

    Serial.print("target_position ");
    serial_println_vec3(leg->get_target_position());

    Serial.print("current_angles ");
    serial_println_vec3(leg->get_current_angles());

    Serial.print("error ");
    Serial.println(leg->get_error());

    Serial.print("current_reach ");
    Serial.println(leg->get_reach());

    Serial.print("mode ");
    Serial.println(leg->get_mode());

    Serial.print("speed ");
    Serial.println(leg->get_speed());

    Serial.println("OK");
}

void parse_leg_position_cmd(String cmd) {
    uint_fast8_t token_start = 0;

    int_fast8_t leg_index = -1;
    vec3_t      position  = {0, 0, 0};

    for (int i = 0; i < 4; i++) {
        uint_fast8_t token_end = cmd.indexOf(" ", token_start);
        String       value_str = cmd.substring(token_start, token_end);

        if (i == 0) {
            leg_index = value_str.toInt();
        } else if (i == 1) {
            position.x = value_str.toFloat();
        } else if (i == 2) {
            position.y = value_str.toFloat();
        } else if (i == 3) {
            position.z = value_str.toFloat();
        }

        token_start = token_end + 1;
    }

    Serial.print("SET_LEG_POSITION ");
    Serial.print(leg_index);
    Serial.print(" ");
    serial_print_vec3(position);
    Serial.println();

    legs[leg_index]->set_target_foot_position(position);

    Serial.println("OK");
}

void parse_leg_angles_cmd(String cmd) {
    uint_fast8_t token_start = 0;

    int_fast8_t leg_index = -1;
    vec3_t      angles    = {0, 0, 0};

    for (int i = 0; i < 4; i++) {
        uint_fast8_t token_end = cmd.indexOf(" ", token_start);
        String       value_str = cmd.substring(token_start, token_end);

        if (i == 0) {
            leg_index = value_str.toInt();
        } else if (i == 1) {
            angles.x = value_str.toFloat();
        } else if (i == 2) {
            angles.y = value_str.toFloat();
        } else if (i == 3) {
            angles.z = value_str.toFloat();
        }

        token_start = token_end + 1;
    }

    Serial.print("SET_LEG_ANGLES ");
    Serial.print(leg_index);
    Serial.print(" ");
    serial_print_vec3(angles);
    Serial.println();

    legs[leg_index]->set_joint_angles(angles.x, angles.y, angles.z);

    Serial.println("OK");
}

void parse_leg_speed_cmd(String cmd) {
    uint_fast8_t token_start = 0;

    int_fast8_t leg_index = -1;
    float       leg_speed = 0.0f;

    for (int i = 0; i < 2; i++) {
        uint_fast8_t token_end = cmd.indexOf(" ", token_start);
        String       value_str = cmd.substring(token_start, token_end);

        if (i == 0) {
            leg_index = value_str.toInt();
        } else if (i == 1) {
            leg_speed = value_str.toFloat();
        }

        token_start = token_end + 1;
    }

    Serial.print("SET_LEG_SPEED ");
    Serial.print(leg_index);
    Serial.print(" ");
    Serial.print(leg_speed);
    Serial.println();

    legs[leg_index]->set_leg_speed(leg_speed);

    Serial.println("OK");
}

void set_cmd(String cmd) {
    int    separator_index = cmd.indexOf("=");
    String key             = cmd.substring(0, separator_index);
    String value_str       = cmd.substring(separator_index + 1);

    if (key == "TOLERANCE") {
        float value = value_str.toFloat();

        for (auto leg : legs) {
            leg->set_tolerance(value);
        }
    } else if (key == "LEG_SPEED") {
        float value = value_str.toFloat();

        for (auto leg : legs) {
            leg->set_leg_speed(value);
        }
    } else if (key == "MODE") {
        LegMode leg_mode;

        if (value_str == "INSTANTANEOUS") {
            leg_mode = LegMode::INSTANTANEOUS;
        } else if (value_str == "CONSTANT_SPEED") {
            leg_mode = LegMode::CONSTANT_SPEED;
        } else {
            Serial.print("ERROR: unknown mode: ");
            Serial.println(value_str);

            return;
        }

        for (auto leg : legs) {
            leg->set_leg_mode(leg_mode);
        }
    } else {
        Serial.print("ERROR: unknown key: ");
        Serial.println(key);

        return;
    }

    Serial.print("SET ");
    Serial.print(key);
    Serial.print("=");
    Serial.print(value_str);
    Serial.println();

    Serial.println("OK");
}
