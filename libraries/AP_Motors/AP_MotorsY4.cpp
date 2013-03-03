/*
 *       AP_MotorsY4.cpp - ArduCopter motors library
 *       Code by John Stäck / RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsY4.h"

// setup_motors - configures the motors for a coaxial-tail Y4 frame
void AP_MotorsY4::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // MultiWii set-up
    add_motor_raw(AP_MOTORS_MOT_1, -1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CCW);
    add_motor_raw(AP_MOTORS_MOT_2,  1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CW);
    add_motor_raw(AP_MOTORS_MOT_3,  0.0, -1.333, AP_MOTORS_MATRIX_MOTOR_CW);
    add_motor_raw(AP_MOTORS_MOT_4,  0.0, -1.333, AP_MOTORS_MATRIX_MOTOR_CCW);
}
