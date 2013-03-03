/*
 *       AP_MotorsY4.cpp - ArduCopter motors library
 *       Code by John Stäck / RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsY4V.h"


/*

These defines are set in config.h, can be overridden in APM_Config.h

Y4V_TAIL - Direction of the V-tail (up or down)
Y4V_V_TAIL - Tail motors tilted inward ("V"-shaped)
Y4V_A_TAIL - Tail motors tilted outward (upside-down V is an "A")


Y4V_TAIL_WIDTH - Width between tail rotors as fraction of "main width"


Y4V_TAIL_ANGLE - Tail motor tilt angle from vertical, in degrees

Y4V_YAW_FACTOR - Yaw effect of a propellers momentum as fraction of its "pull force"
Assuming some proportional relation between frame and prop size, this should not vary too much.

*/

#ifndef Y4V_TAIL
 # define Y4V_TAIL Y4V_V_TAIL
#endif
#ifndef Y4V_TAIL_ANGLE
 # define Y4V_TAIL_ANGLE 15.0
#endif
#ifndef Y4V_TAIL_WIDTH
 # define Y4V_TAIL_WIDTH 0.3
#endif
#ifndef Y4V_YAW_FACTOR
 # define Y4V_YAW_FACTOR 0.05
#endif



// setup_motors - configures the motors for a V-tail Y4 frame
void AP_MotorsY4V::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    //Motor tilt angle in radians is useful
    float tilt_angle = ToRad(Y4V_TAIL_ANGLE);

    //A tail motors efficiency in vertical lift
    float lift_efficiency = cos(tilt_angle);

    //How much a tail motor contributes to roll. The actual "v-boom" length (which is perpendicular to roll axis)
    //is width / cos(tilt_angle)
    float roll_force = Y4V_TAIL_WIDTH / lift_efficiency;

    //Yaw force of a tail motor is from two different factors, the rotational momentum of the prop (just as when yawing a quad),
    //and the "sideways pull" from the tilt. This is scaled up so 1.0 equals a horizontal prop. Tilted motors will have lots more.
    //The frames are set up so these two factors work together in the tail motors to yaw the copter, so they're summed up.
    //For a 0 degree tilt angle, you will only have the "normal yaw", for 90 degree you would get the full pull of the motor sideways.

    float yaw_force = lift_efficiency + sin(tilt_angle) / Y4V_YAW_FACTOR;


    //The front motors will actually contribute to yaw as well (1.0 of it). While you might want to have the software see this
    //effect as zero (to only have the tail do the yaw action, such as with regular tricopters), the front motors will have to
    //work anyway to compensate for the roll of the tail so it might as well be included.

    //Tail rotors directions are different for "V" and "A" tails, and the front rotors are set to match.

#if Y4V_TAIL == Y4V_V_TAIL
    add_motor_raw(AP_MOTORS_MOT_1,  1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CCW); //Front left, CCW
    add_motor_raw(AP_MOTORS_MOT_2, -1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CW);  //Front right, CW
    add_motor_raw(AP_MOTORS_MOT_3, -roll_force, -1.333*lift_efficiency, AP_MOTORS_MATRIX_MOTOR_CW*yaw_force);  //Tail right, CCW
    add_motor_raw(AP_MOTORS_MOT_4,  roll_force, -1.333*lift_efficiency, AP_MOTORS_MATRIX_MOTOR_CCW*yaw_force); //Tail left, CW
#else //A tail
    add_motor_raw(AP_MOTORS_MOT_1,  1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CW); //Front left, CW
    add_motor_raw(AP_MOTORS_MOT_2, -1.0,  0.666, AP_MOTORS_MATRIX_MOTOR_CCW);  //Front right, CCW
    add_motor_raw(AP_MOTORS_MOT_3, -roll_force, -1.333*lift_efficiency, AP_MOTORS_MATRIX_MOTOR_CCW*yaw_force);  //Tail right, CW
    add_motor_raw(AP_MOTORS_MOT_4,  roll_force, -1.333*lift_efficiency, AP_MOTORS_MATRIX_MOTOR_CW*yaw_force); //Tail left, CCW
#endif

}
