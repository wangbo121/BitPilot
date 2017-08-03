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
	   // angle error
	    target_angle            = wrap_180(target_angle - ahrs->roll_sensor);

	    // convert to desired Rate:
	    int32_t target_rate = g.pi_stabilize_roll.get_p(target_angle);

	    int16_t i_stab;
	    if(labs(ahrs->roll_sensor) < 500)
	    {
	        target_angle            = constrain_value(target_angle, -500, 500);
	        i_stab                          = g.pi_stabilize_roll.get_i(target_angle, G_Dt);
	    }
	    else
	    {
	        i_stab                          = g.pi_stabilize_roll.get_integrator();
	    }

	    // set targets for rate controller
	    set_roll_rate_target(target_rate+i_stab, EARTH_FRAME);
}


void
Copter::get_stabilize_pitch(int32_t target_angle)
{
	 // angle error
	    target_angle            = wrap_180(target_angle - ahrs->pitch_sensor);


	    // convert to desired Rate:
	    int32_t target_rate = g.pi_stabilize_pitch.get_p(target_angle);

	    int16_t i_stab;
	    if(labs(ahrs->pitch_sensor) < 500) {
	        target_angle            = constrain_value(target_angle, -500, 500);
	        i_stab                          = g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
	    }else{
	        i_stab                          = g.pi_stabilize_pitch.get_integrator();
	    }

	    // set targets for rate controller
	    set_pitch_rate_target(target_rate + i_stab, EARTH_FRAME);


}

void
Copter::get_stabilize_yaw(int32_t target_angle)
{
	int32_t target_rate,i_term;
	int32_t angle_error;
	int32_t output = 0;

	// angle error
	angle_error             = wrap_180(target_angle - ahrs->yaw_sensor,100.0);

	// limit the error we're feeding to the PID

	angle_error             = constrain_value(angle_error, -4000, 4000);

	// convert angle error to desired Rate:
	target_rate = g.pi_stabilize_yaw.get_p(angle_error);
	i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

	// set targets for rate controller
	set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);

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
	rate_targets_frame = earth_or_body_frame;
	    if( earth_or_body_frame == BODY_FRAME )
	    {
	        roll_rate_target_bf = desired_rate;
	    }
	    else
	    {
	        roll_rate_target_ef = desired_rate;
	    }

}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void
Copter::set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{
	 rate_targets_frame = earth_or_body_frame;
	    if( earth_or_body_frame == BODY_FRAME )
	    {
	        pitch_rate_target_bf = desired_rate;
	    }
	    else
	    {
	        pitch_rate_target_ef = desired_rate;
	    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void
Copter::set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame )
{
	//uint8_t rate_targets_frame;
	rate_targets_frame = earth_or_body_frame;
	if( earth_or_body_frame == BODY_FRAME )
	{
		yaw_rate_target_bf = desired_rate;
	}
	else
	{
		yaw_rate_target_ef = desired_rate;
	}
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
Copter::update_rate_contoller_targets()
{
	Vector2f yawvector;
	Matrix3f temp   = ahrs->get_dcm_matrix();

	yawvector.x     = temp.a.x;     // sin
	yawvector.y     = temp.b.x;         // cos
	yawvector.normalize();

	cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
	cos_roll_x          = temp.c.z / cos_pitch_x;                       // level = 1

	cos_pitch_x = constrain_value(cos_pitch_x, 0.0f, 1.0f);
	// this relies on constrain() of infinity doing the right thing,
	// which it does do in avr-libc
	cos_roll_x  = constrain_value(cos_roll_x, -1.0f, 1.0f);

	sin_yaw_y               = yawvector.x;                                              // 1y = north
	cos_yaw_x               = yawvector.y;                                              // 0x = north

	// added to convert earth frame to body frame for rate controllers
	sin_pitch = -temp.c.x;
	sin_roll = temp.c.y / cos_pitch_x;

	 if( rate_targets_frame == EARTH_FRAME )
	 {
	        // convert earth frame rates to body frame rates
	        roll_rate_target_bf = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
	        pitch_rate_target_bf = cos_roll_x * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
	        yaw_rate_target_bf = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
	    }

}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
Copter::run_rate_controllers()
{
    // call rate controllers
    g.channel_roll.servo_out = get_rate_roll(roll_rate_target_bf);
    g.channel_pitch.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.channel_rudder.servo_out = get_rate_yaw(yaw_rate_target_bf);

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

void
Copter::update_yaw_mode(void)
{
	switch(yaw_mode)
	{
	case YAW_STABILE:
		get_stabilize_yaw(g.channel_rudder.control_in);
		break;
	default:
		break;
	}
}

void
Copter::update_roll_pitch_mode(void)
{
	switch(roll_pitch_mode)
	{
	case ROLL_PITCH_ACRO:

			get_roll_rate_stabilized_ef(g.channel_roll.control_in);
			get_pitch_rate_stabilized_ef(g.channel_pitch.control_in);

			/* 下面的这两个貌似是特技模式，但是也是角速度模式 */
			//get_acro_roll(g.rc_1.control_in);
			//get_acro_pitch(g.rc_2.control_in);
		break;
	case ROLL_PITCH_STABLE:
		control_roll            = g.channel_roll.control_in;
		control_pitch           = g.channel_roll.control_in;

		get_stabilize_roll(control_roll);
		get_stabilize_pitch(control_pitch);
		break;

	default:
		break;
	}
}

