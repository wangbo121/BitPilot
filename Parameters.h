/*
 * Parameters.h
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

/*
 * 这个文件需要定义Parameter这个结构变量
 * 同时包含所有的
 * 宏定义的值放在哪个文件呢？
 * 把宏定义都放在global.h文件吧，这个文件是我自己根据我自己的习惯建立的，跟apm没有关系
 * 主要是为了在整个工程中除了库文件都可以用到，进行值的传递或者仿真
 */
class Parameters
{

public:

	int16_t sysid_this_mav;
	int16_t sysid_my_gcs;
	int8_t telem_delay;

	int16_t    format_version;

	// Feed-forward gains
	//
	float    kff_pitch_compensation;
	float    kff_rudder_mix;
	float    kff_pitch_to_throttle;
	float    kff_throttle_to_pitch;

	// Crosstrack navigation
	//
	float    crosstrack_gain;
	int16_t    crosstrack_entry_angle;

	// Estimation
	//
	float    altitude_mix;
	float    airspeed_ratio;

	// Waypoints
	//
	int8_t     waypoint_mode;
	int8_t     waypoint_total;
	int8_t     waypoint_index;
	int8_t     waypoint_radius;//不管apm2.3是啥了，这个单位应该是米20170919
	int8_t     loiter_radius;

	// Waypoints
	//
	//int8_t         waypoint_mode;
	int8_t         command_total;
	int8_t         command_index;
	int8_t         command_nav_index;
	//int16_t        waypoint_radius;
	//int16_t        loiter_radius;
	int16_t        waypoint_speed_max;
	//float        crosstrack_gain;
	int32_t        auto_land_timeout;

	int8_t         tilt_comp;

	// Fly-by-wire
	//
	int8_t     flybywire_airspeed_min;
	int8_t     flybywire_airspeed_max;

	// Throttle
	//
	int8_t     throttle_min;
	int8_t     throttle_max;
	int8_t     throttle_fs_enabled;
	int8_t     throttle_fs_action;
	int16_t    throttle_fs_value;
	//int8_t     throttle_cruise;
	int32_t     throttle_cruise;

	// Flight modes
	//
	int8_t     flight_mode_channel;
	int8_t  flight_modes;

	// Navigational maneuvering limits
	//
	int16_t    roll_limit;
	int16_t    pitch_limit_max;
	int16_t    pitch_limit_min;

	// Misc
	//
	int8_t     auto_trim;
	int8_t     switch_enable;
	int16_t    log_bitmask;
	int16_t    airspeed_cruise;
	int16_t    pitch_trim;
	int16_t    RTL_altitude;
	int16_t    ground_temperature;
	int32_t    ground_pressure;
	int8_t		compass_enabled;
	int16_t    angle_of_attack;
	int8_t		battery_monitoring;	// 0=disabled, 1=3 cell lipo, 2=4 cell lipo, 3=total voltage only, 4=total voltage and current
	int16_t	pack_capacity;		// Battery pack capacity less reserve

	AP_RC _rc;
	// RC channels
	AP_RC_Channel  channel_roll;
	AP_RC_Channel  channel_pitch;
	AP_RC_Channel  channel_throttle;
	AP_RC_Channel  channel_rudder;
	AP_RC_Channel	rc_5;
	AP_RC_Channel	rc_6;
	AP_RC_Channel	rc_7;
	AP_RC_Channel	rc_8;

	// AP_PID controllers
	//
	AP_PID         pidNavRoll;
	AP_PID         pidServoRoll;
	AP_PID         pidServoPitch;
	AP_PID         pidNavPitchAirspeed;
	AP_PID         pidServoRudder;
	AP_PID         pidTeThrottle;
	AP_PID         pidNavPitchAltitude;

	AP_PID                  pid_rate_roll;
	AP_PID                  pid_rate_pitch;
	AP_PID                  pid_rate_yaw;
	AP_PID                  pid_loiter_rate_lat;
	AP_PID                  pid_loiter_rate_lon;
	AP_PID                  pid_nav_lat;
	AP_PID                  pid_nav_lon;

	AP_PID                  pid_throttle;
	AP_PID                  pid_optflow_roll;
	AP_PID                  pid_optflow_pitch;

	AP_PID                  pi_loiter_lat;
	AP_PID                  pi_loiter_lon;
	AP_PID                  pi_stabilize_roll;
	AP_PID                  pi_stabilize_pitch;
	AP_PID                  pi_stabilize_yaw;
	AP_PID                  pi_alt_hold;

	//		// PI/D controllers
	//		AP_PID		pid_rate_roll;
	//		AP_PID		pid_rate_pitch;
	//		AP_PID		pid_rate_yaw;
	//		AP_PID		pid_nav_lat;
	//		AP_PID		pid_nav_lon;

	int16_t        auto_slew_rate;

	uint8_t     junk;

	int8_t		sonar_enabled;
	int8_t		airspeed_enabled;
	int8_t		flap_1_percent;
	int8_t		flap_1_speed;
	int8_t		flap_2_percent;
	int8_t		flap_2_speed;

	Parameters() {}
};


#endif /* PARAMETERS_H_ */
