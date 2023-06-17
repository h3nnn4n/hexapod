#include <AUnit.h>
#include <Utils.h>
#include <VectorDatatype.h>

#include "common.h"

test(ik_resting_position) {
    vec3_t result = leg.inverse_kinematics(0.0f, 294.13f, 0.0f);

    // serial_println_vec3(result);

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 0.0f, eps);
    assertNear(result.z, 0.0f, eps);
}

test(ik_45_forward) {
    vec3_t result = leg.inverse_kinematics(207.98f, 207.98f, 0.0f);

    // Serial.println("\n");
    // Serial.print("..actual: ");
    // serial_println_vec3(result);

    // Serial.print("expected: ");
    // serial_println_vec3(vec3_t(45.0f, 0.0f, 0.0f));
    // Serial.println("\n");

    assertNear(result.x, 45.0f, eps);
    assertNear(result.y, 0.0f, eps);
    assertNear(result.z, 0.0f, eps);
}

test(ik_z_pose) {
    vec3_t result = leg.inverse_kinematics(0, 52.0f, -(78.984f + 163.148f));

    // Serial.println("\n");
    // Serial.print("..actual: ");
    // serial_println_vec3(result);

    // Serial.print("expected: ");
    // serial_println_vec3(vec3_t(0.0f, 90.0f, 90.0f));
    // Serial.println("\n");

    assertNear(result.x, 0.0f, eps);
    assertNear(result.y, 90.0f, eps);
    assertNear(result.z, 90.0f, eps);
}
