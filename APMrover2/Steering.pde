// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(g.channel_throttle.radio_max - g.channel_throttle.radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        g.channel_throttle.radio_out = constrain_int16(g.channel_throttle.radio_out, last_throttle - temp, last_throttle + temp);
    }
}

/*
  calculate the throtte for auto-throttle modes
 */
static void calc_throttle(float target_speed)
{  
    if (target_speed <= 0) {
        // cope with zero requested speed
        g.channel_throttle.servo_out = g.throttle_min.get();
        return;
    }

    int throttle_target = g.throttle_cruise + throttle_nudge;  

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf((nav_steer/nav_gain_scaler) / (float)SERVO_MAX);
    steer_rate = constrain(steer_rate, 0.0, 1.0);
    float reduction = 1.0 - steer_rate*(100 - g.speed_turn_gain)*0.01;
    
    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0 - (100-g.speed_turn_gain)*0.01*((g.speed_turn_dist - wp_distance)/g.speed_turn_dist);
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }
    
    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = target_speed - ground_speed; 
    
    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    g.channel_throttle.servo_out = constrain_int16(throttle, g.throttle_min.get(), g.throttle_max.get());
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

static void calc_nav_steer()
{
	// Adjust gain based on ground speed
	nav_gain_scaler = (float)ground_speed / g.speed_cruise;
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// Calculate the required turn of the wheels rover
	// ----------------------------------------

    // negative error = left turn
	// positive error = right turn
	nav_steer = g.pidNavSteer.get_pid(bearing_error_cd, nav_gain_scaler);

    if (obstacle) {  // obstacle avoidance 
	    nav_steer += g.sonar_turn_angle*100;
    }

    g.channel_steer.servo_out = nav_steer;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = g.channel_throttle.radio_out;

	if ((control_mode == MANUAL || control_mode == LEARNING) &&
        (g.skid_steer_out == g.skid_steer_in)) {
        // do a direct pass through of radio values
        g.channel_steer.radio_out       = hal.rcin->read(CH_STEER);
        g.channel_throttle.radio_out    = hal.rcin->read(CH_THROTTLE);
	} else {       
        g.channel_steer.calc_pwm();
		g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, 
                                                       g.throttle_min.get(), 
                                                       g.throttle_max.get());
        // convert 0 to 100% into PWM
        g.channel_throttle.calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);

        if (g.skid_steer_out) {
            // convert the two radio_out values to skid steering values
            /*
              mixing rule:
              steering = motor1 - motor2
              throttle = 0.5*(motor1 + motor2)
              motor1 = throttle + 0.5*steering
              motor2 = throttle - 0.5*steering
            */          
            float steering_scaled = g.channel_steer.norm_output();
            float throttle_scaled = g.channel_throttle.norm_output();
            float motor1 = throttle_scaled + 0.5*steering_scaled;
            float motor2 = throttle_scaled - 0.5*steering_scaled;
            g.channel_steer.servo_out = 4500*motor1;
            g.channel_throttle.servo_out = 100*motor2;
            g.channel_steer.calc_pwm();
            g.channel_throttle.calc_pwm();
        }
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    hal.rcout->write(CH_1, g.channel_steer.radio_out);     // send to Servos
    hal.rcout->write(CH_3, g.channel_throttle.radio_out);     // send to Servos

	// Route configurable aux. functions to their respective servos
	g.rc_2.output_ch(CH_2);
	g.rc_4.output_ch(CH_4);
	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);

#endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {

    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
        hal.rcout->write(1, 1400);
        mavlink_delay(400);
        hal.rcout->write(1, 1600);
        mavlink_delay(200);
        hal.rcout->write(1, 1500);
#endif
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}
