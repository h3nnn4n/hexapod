#include <AUnit.h>
#include <Utils.h>
#include <VectorDatatype.h>

#include "common.h"

test(fk_resting_position) {
    vec3_t result = leg.forward_kinematics(0, 0, 0);

    // serial_println_vec3(result);

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 294.13f, eps);
    assertNear(result.z, 0.0f, eps);
}

test(fk_full_front) {
    vec3_t result = leg.forward_kinematics(90, 0, 0);

    // serial_println_vec3(result);

    assertNear(result.x, 294.13f, eps);
    assertNear(result.y, 0.0f, eps);
    assertNear(result.z, 0.0f, eps);
}

test(fk_45_forward) {
    vec3_t result = leg.forward_kinematics(45, 0, 0);

    // serial_println_vec3(result);

    assertNear(result.x, 207.98f, eps);
    assertNear(result.y, 207.98f, eps);
    assertNear(result.z, 0.0f, eps);
}

test(fk_tibia_full_down) {
    vec3_t result = leg.forward_kinematics(0, 0, 90);

    // serial_println_vec3(result);

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 130.98f, eps);
    assertNear(result.z, -163.15f, eps);
}

test(fk_femur_full_down) {
    vec3_t result = leg.forward_kinematics(0, 90, 0);

    // serial_println_vec3(result);

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 52.0f, eps);
    assertNear(result.z, -(78.984f + 163.148f), eps);
}

test(fk_z_pose) {
    vec3_t result = leg.forward_kinematics(0, -90, 90);

    // serial_println_vec3(result);

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 52.0f, eps);
    assertNear(result.z, -(78.984f + 163.148f), eps);
}
