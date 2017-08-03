/*
 * attitude.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include "copter.h"

//get_yaw_rate_stabilized_ef(g.rc_4.control_in);
void
Copter::get_stabilize_roll(int32_t target_angle)
{

}


void
Copter::get_stabilize_pitch(int32_t target_angle)
{

}

void
Copter::get_stabilize_yaw(int32_t target_angle)
{

}

void
Copter::get_stabilize_rate_yaw(int32_t target_rate)
{

}

void
Copter::get_acro_roll(int32_t target_rate)
{

}

void
Copter::get_acro_pitch(int32_t target_rate)
{

}

void
Copter::get_acro_yaw(int32_t target_rate)
{

}

// Roll with rate input and stabilized in the earth frame
void
Copter::get_roll_rate_stabilized_ef(int32_t stick_angle)
{

}

// Pitch with rate input and stabilized in the earth frame
void
Copter::get_pitch_rate_stabilized_ef(int32_t stick_angle)
{

}

// Yaw with rate input and stabilized in the earth frame
void
Copter::get_yaw_rate_stabilized_ef(int32_t stick_angle)
{


}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void
Copter::set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{

}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void
Copter::set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{

}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void
Copter::set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{

}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
Copter::update_rate_contoller_targets()
{

}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
Copter::run_rate_controllers()
{

}

int16_t
Copter::get_rate_roll(int32_t target_rate)
{

	return 0;
}

int16_t
Copter::get_rate_pitch(int32_t target_rate)
{
	return 0;

}

int16_t
Copter::get_rate_yaw(int32_t target_rate)
{

	return 0;

}

int16_t
Copter::get_throttle_rate(int16_t z_target_speed)
{
	return 0;
}


/*
 *  reset all I integrators
 */
void
Copter::reset_I_all(void)
{

}

void
Copter::reset_rate_I()
{

}

void
Copter::reset_throttle_I(void)
{
}

void
Copter::reset_stability_I(void)
{
}

