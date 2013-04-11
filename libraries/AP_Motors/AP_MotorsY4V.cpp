/*
 *       AP_MotorsY4.cpp - ArduCopter motors library
 *       Code by John Stäck
 *
 *       This library is free software; you can redistribute it and/or
 *       modify it under the terms of the GNU Lesser General Public
 *       License as published by the Free Software Foundation; either
 *       version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsY4V.h"

/*

FRAME_ORIENTATION  Direction of the V-tail (up or down)
                Y4V_V_TAIL  Tail motors tilted inward ("V"-shaped)
                Y4V_A_TAIL  Tail motors tilted outward (upside-down V is an "A") 

Frame parameters that determine motor behaviour. For anything not too far off from the defaults, it will behave well
without changing these values, but they do matter for "perfect tuning"

Y4V_TAIL_WIDTH  Width between tail props as fraction of "main width" (distance between front props),
                measured perpendicular to the motor axis. This value will depend a lot on tail design (height, width, tilt).
                It is used to determine the tail props leverage on roll.

Y4V_TAIL_ANGLE  Tail motor tilt angle from vertical, in degrees (0 - 90)

Y4V_YAW_FACTOR  Yaw effect of a propellers rotation as fraction of its "pull force"
                Assuming some proportional relation between frame and prop size, this should not vary too much.

Y4V_COG         Center of gravity. The crafts center of gravity along the main axis. 0 = all the way forward,
                1 = all the way back. 1/3rd is the default for tricopters and is used here too, but since the
                two tail props will usually have more lift than a single one (but less than two horizontal props),
                it can be moved back a bit.

*/

#ifndef FRAME_ORIENTATION
 #define FRAME_ORIENTATION Y4V_A_TAIL
#endif
#ifndef Y4V_TAIL_ANGLE
 #define Y4V_TAIL_ANGLE 30.0
#endif
#ifndef Y4V_TAIL_WIDTH
 #define Y4V_TAIL_WIDTH 0.25
#endif
#ifndef Y4V_YAW_FACTOR
 #define Y4V_YAW_FACTOR 0.08
#endif
#ifndef Y4V_COG
 #define Y4V_COG 0.4
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

    //Pitch effect of front and rear motors
    float front_pitch = 1.0 * (1 - Y4V_COG);
    float rear_pitch  = -lift_efficiency * (1 + Y4V_COG); //Frontward pitch is negative

    //Yaw force of a tail motor is from two different factors, the rotational momentum of the prop (just as when yawing a quad),
    //and the "sideways pull" from the tilt. This is scaled so 1.0 equals a tilted tail motor, and the front motors will have a
    //fraction of that.
    //The frames are set up so these two forces work together in the tail motors to yaw the copter, so they're summed up.
    //For a 0 degree tilt angle, you will only have the "normal yaw", for 90 degree you would get the full pull of the motor sideways.
    //For small tail tilt angles, this effect is significant, from both front and rear.
    float yaw_force = sin(tilt_angle) + cos(tilt_angle) / Y4V_YAW_FACTOR;


    //Tail prop directions are different for "V" and "A" tails, and the front props are set to match.

#if FRAME_ORIENTATION == Y4V_V_TAIL
    add_motor_raw(AP_MOTORS_MOT_1,  1.0,            front_pitch, AP_MOTORS_MATRIX_MOTOR_CCW / yaw_force, 4); //Front left, CCW
    add_motor_raw(AP_MOTORS_MOT_2, -1.0,            front_pitch, AP_MOTORS_MATRIX_MOTOR_CW / yaw_force, 1);  //Front right, CW
    add_motor_raw(AP_MOTORS_MOT_3, -Y4V_TAIL_WIDTH, rear_pitch,  AP_MOTORS_MATRIX_MOTOR_CW, 2);  //Tail right, CW
    add_motor_raw(AP_MOTORS_MOT_4,  Y4V_TAIL_WIDTH, rear_pitch,  AP_MOTORS_MATRIX_MOTOR_CCW, 3); //Tail left, CCW

#else //A tail

//    add_motor_raw(AP_MOTORS_MOT_1,  1.0,            front_pitch, AP_MOTORS_MATRIX_MOTOR_CCW / yaw_force, 4); //Front left, CCW
//    add_motor_raw(AP_MOTORS_MOT_2, -1.0,            front_pitch, AP_MOTORS_MATRIX_MOTOR_CW / yaw_force, 1);  //Front right, CW
    add_motor_raw(AP_MOTORS_MOT_1,  1.0,            front_pitch, 0, 4); //Front left, CCW
    add_motor_raw(AP_MOTORS_MOT_2, -1.0,            front_pitch, 0, 1);  //Front right, CW
    add_motor_raw(AP_MOTORS_MOT_3, -Y4V_TAIL_WIDTH, rear_pitch,  AP_MOTORS_MATRIX_MOTOR_CCW, 2);  //Tail right, CCW
    add_motor_raw(AP_MOTORS_MOT_4,  Y4V_TAIL_WIDTH, rear_pitch,  AP_MOTORS_MATRIX_MOTOR_CW, 3); //Tail left, CW

#endif

}
