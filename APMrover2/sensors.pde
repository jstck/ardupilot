// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_sonar(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    sonar.Init(&adc);
#else
    sonar.Init(NULL);
#endif
}

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

void ReadSCP1000(void) {}

#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void read_battery(void)
{
	if(g.battery_monitoring == 0) {
		battery_voltage1 = 0;
		return;
	}
	
    if(g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_volt_pin->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_pin);
    }
    if(g.battery_monitoring == 4) {
        // this copes with changing the pin at runtime
        batt_curr_pin->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_pin);
        current_total1   += current_amps1 * (float)delta_ms_medium_loop * 0.0002778;                                    // .0002778 is 1/3600 (conversion to hours)
    }
}


// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->read_average();
    receiver_rssi = constrain_int16(ret, 0, 255);
}
