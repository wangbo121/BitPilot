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

	// The version of the layout as described by the parameter enum.
	//
	// When changing the parameter enum in an incompatible fashion, this
	// value should be incremented by one.
	//
	// The increment will prevent old parameters from being used incorrectly
	// by newer code.
	//
	static const uint16_t k_format_version = 113;

	// The parameter software_type is set up solely for ground station use
	// and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
	// GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
	// values within that range to identify different branches.
	//
	static const uint16_t k_software_type = 10;		// 0 for APM trunk







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


//
	// Parameter identities.
	//
	// The enumeration defined here is used to ensure that every parameter
	// or parameter group has a unique ID number.	This number is used by
	// AP_Var to store and locate parameters in EEPROM.
	//
	// Note that entries without a number are assigned the next number after
	// the entry preceding them.	When adding new entries, ensure that they
	// don't overlap.
	//
	// Try to group related variables together, and assign them a set
	// range in the enumeration.	Place these groups in numerical order
	// at the end of the enumeration.
	//
	// WARNING: Care should be taken when editing this enumeration as the
	//			AP_Var load/save code depends on the values here to identify
	//			variables saved in EEPROM.
	//
	//
	enum {
	// Layout version number, always key zero.
	//
	k_param_format_version = 0,
	k_param_software_type,
	k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
	k_param_ins,                            // libraries/AP_InertialSensor variables

	// simulation
	k_param_sitl = 10,

	// barometer object (needed for SITL)
	k_param_barometer,

	// scheduler object (for debugging)
	k_param_scheduler,

	// relay object
	k_param_relay,

	// EPM object
	k_param_epm,

	// BoardConfig object
	k_param_BoardConfig,

	// GPS object
	k_param_gps,

	// Parachute object
	k_param_parachute,

	// Landing gear object
	k_param_landinggear,    // 18

	// Input Management object
	k_param_input_manager,  // 19 FULL!

	// Misc
	//
	k_param_log_bitmask_old = 20,           // Deprecated
	k_param_log_last_filenumber,            // *** Deprecated - remove
											// with next eeprom number
											// change
	k_param_toy_yaw_rate,                   // deprecated - remove
	k_param_crosstrack_min_distance,	// deprecated - remove with next eeprom number change
	k_param_rssi_pin,
	k_param_throttle_accel_enabled,     // deprecated - remove
	k_param_wp_yaw_behavior,
	k_param_acro_trainer,
	k_param_pilot_velocity_z_max,
	k_param_circle_rate,                // deprecated - remove
	k_param_sonar_gain,
	k_param_ch8_option,
	k_param_arming_check,
	k_param_sprayer,
	k_param_angle_max,
	k_param_gps_hdop_good,
	k_param_battery,
	k_param_fs_batt_mah,
	k_param_angle_rate_max,         // remove
	k_param_rssi_range,
	k_param_rc_feel_rp,
	k_param_NavEKF,                 // Extended Kalman Filter Inertial Navigation Group
	k_param_mission,                // mission library
	k_param_rc_13,
	k_param_rc_14,
	k_param_rally,
	k_param_poshold_brake_rate,
	k_param_poshold_brake_angle_max,
	k_param_pilot_accel_z,
	k_param_serial0_baud,           // deprecated - remove
	k_param_serial1_baud,           // deprecated - remove
	k_param_serial2_baud,           // deprecated - remove
	k_param_land_repositioning,
	k_param_sonar, // sonar object
	k_param_fs_ekf_thresh,
	k_param_terrain,
	k_param_acro_expo,
	k_param_throttle_deadzone,
	k_param_optflow,
	k_param_dcmcheck_thresh,        // deprecated - remove
	k_param_log_bitmask,
	k_param_cli_enabled,
	k_param_throttle_filt,
	k_param_throttle_behavior,
	k_param_pilot_takeoff_alt, // 64

	// 65: AP_Limits Library
	k_param_limits = 65,            // deprecated - remove
	k_param_gpslock_limit,          // deprecated - remove
	k_param_geofence_limit,         // deprecated - remove
	k_param_altitude_limit,         // deprecated - remove
	k_param_fence,
	k_param_gps_glitch,             // deprecated
	k_param_baro_glitch,            // 71 - deprecated

	//
	// 75: Singlecopter, CoaxCopter
	//
	k_param_single_servo_1 = 75,
	k_param_single_servo_2,
	k_param_single_servo_3,
	k_param_single_servo_4, // 78

	//
	// 80: Heli
	//
	k_param_heli_servo_1 = 80,
	k_param_heli_servo_2,
	k_param_heli_servo_3,
	k_param_heli_servo_4,
	k_param_heli_pitch_ff,      // remove
	k_param_heli_roll_ff,       // remove
	k_param_heli_yaw_ff,        // remove
	k_param_heli_stab_col_min,  // remove
	k_param_heli_stab_col_max,  // remove
	k_param_heli_servo_rsc,     // 89 = full!

	//
	// 90: Motors
	//
	k_param_motors = 90,
	k_param_disarm_delay,

	//
	// 100: Inertial Nav
	//
	k_param_inertial_nav = 100, // deprecated
	k_param_wp_nav,
	k_param_attitude_control,
	k_param_pos_control,
	k_param_circle_nav,     // 104

	// 110: Telemetry control
	//
	k_param_gcs0 = 110,
	k_param_gcs1,
	k_param_sysid_this_mav,
	k_param_sysid_my_gcs,
	k_param_serial1_baud_old, // deprecated
	k_param_telem_delay,
	k_param_gcs2,
	k_param_serial2_baud_old, // deprecated
	k_param_serial2_protocol, // deprecated
	k_param_serial_manager,
	k_param_ch9_option,
	k_param_ch10_option,
	k_param_ch11_option,
	k_param_ch12_option,
	k_param_takeoff_trigger_dz,
	k_param_gcs3,
	k_param_gcs_pid_mask,    // 126

	//
	// 135 : reserved for Solo until features merged with master
	//
	k_param_rtl_speed_cms = 135,
	k_param_fs_batt_curr_rtl, // 136

	//
	// 140: Sensor parameters
	//
	k_param_imu = 140, // deprecated - can be deleted
	k_param_battery_monitoring = 141,   // deprecated - can be deleted
	k_param_volt_div_ratio, // deprecated - can be deleted
	k_param_curr_amp_per_volt,  // deprecated - can be deleted
	k_param_input_voltage,  // deprecated - can be deleted
	k_param_pack_capacity,  // deprecated - can be deleted
	k_param_compass_enabled,
	k_param_compass,
	k_param_sonar_enabled_old, // deprecated
	k_param_frame_orientation,
	k_param_optflow_enabled,    // deprecated
	k_param_fs_batt_voltage,
	k_param_ch7_option,
	k_param_auto_slew_rate,     // deprecated - can be deleted
	k_param_sonar_type_old,     // deprecated
	k_param_super_simple = 155,
	k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
	k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
	k_param_ahrs, // AHRS group // 159

	//
	// 160: Navigation parameters
	//
	k_param_rtl_altitude = 160,
	k_param_crosstrack_gain,	// deprecated - remove with next eeprom number change
	k_param_rtl_loiter_time,
	k_param_rtl_alt_final,
	k_param_tilt_comp, 	//164	deprecated - remove with next eeprom number change


	//
	// Camera and mount parameters
	//
	k_param_camera = 165,
	k_param_camera_mount,
	k_param_camera_mount2,      // deprecated

	//
	// Batery monitoring parameters
	//
	k_param_battery_volt_pin = 168, // deprecated - can be deleted
	k_param_battery_curr_pin,   // 169 deprecated - can be deleted

	//
	// 170: Radio settings
	//
	k_param_rc_1 = 170,
	k_param_rc_2,
	k_param_rc_3,
	k_param_rc_4,
	k_param_rc_5,
	k_param_rc_6,
	k_param_rc_7,
	k_param_rc_8,
	k_param_rc_10,
	k_param_rc_11,
	k_param_throttle_min,
	k_param_throttle_max,           // remove
	k_param_failsafe_throttle,
	k_param_throttle_fs_action,     // remove
	k_param_failsafe_throttle_value,
	k_param_throttle_trim,          // remove
	k_param_esc_calibrate,
	k_param_radio_tuning,
	k_param_radio_tuning_high,
	k_param_radio_tuning_low,
	k_param_rc_speed = 192,
	k_param_failsafe_battery_enabled,
	k_param_throttle_mid,
	k_param_failsafe_gps_enabled,   // remove
	k_param_rc_9,
	k_param_rc_12,
	k_param_failsafe_gcs,
	k_param_rcmap, // 199

	//
	// 200: flight modes
	//
	k_param_flight_mode1 = 200,
	k_param_flight_mode2,
	k_param_flight_mode3,
	k_param_flight_mode4,
	k_param_flight_mode5,
	k_param_flight_mode6,
	k_param_simple_modes,

	//
	// 210: Waypoint data
	//
	k_param_waypoint_mode = 210, // remove
	k_param_command_total,       // remove
	k_param_command_index,       // remove
	k_param_command_nav_index,   // remove
	k_param_waypoint_radius,     // remove
	k_param_circle_radius,       // remove
	k_param_waypoint_speed_max,  // remove
	k_param_land_speed,
	k_param_auto_velocity_z_min, // remove
	k_param_auto_velocity_z_max, // remove - 219

	//
	// 220: PI/D Controllers
	//
	k_param_acro_rp_p = 221,
	k_param_axis_lock_p,    // remove

	k_param_p_stabilize_roll,
	k_param_p_stabilize_pitch,
	k_param_p_stabilize_yaw,

	k_param_pid_rate_roll_p,
	k_param_pid_rate_roll_i,
	k_param_pid_rate_roll_d,

	k_param_pid_rate_pitch_p,
	k_param_pid_rate_pitch_i,
	k_param_pid_rate_pitch_d,

	k_param_pid_rate_yaw_p,
	k_param_pid_rate_yaw_i,
	k_param_pid_rate_yaw_d,

	k_param_pid_nav_lat_p,
	k_param_pid_nav_lat_i,
	k_param_pid_nav_lat_d,

	k_param_pid_nav_lon_p,
	k_param_pid_nav_lon_i,
	k_param_pid_nav_lon_d,





	k_param_p_alt_hold,
	k_param_p_vel_z,
	k_param_pid_optflow_roll,       // remove
	k_param_pid_optflow_pitch,      // remove
	k_param_acro_balance_roll_old,  // remove
	k_param_acro_balance_pitch_old, // remove
	k_param_pid_accel_z,
	k_param_acro_balance_roll,
	k_param_acro_balance_pitch,
	k_param_acro_yaw_p,
	k_param_autotune_axis_bitmask,
	k_param_autotune_aggressiveness,
	k_param_pi_vel_xy,
	k_param_fs_ekf_action,
	k_param_rtl_climb_min,
	k_param_rpm_sensor,
	k_param_autotune_min_d, // 251

	// 254,255: reserved


/*
 * 因为下面有一些是group变量，pid一个参数其实是3个参数的整合，但是我只摘出来了1个参数的
 * 所以把pid分成了p  i   d  3个参数
 */



//	//
//	// 220: PI/D Controllers
//	//
//	k_param_acro_rp_p = 221,
//	k_param_axis_lock_p,    // remove
//	k_param_pid_rate_roll,
//	k_param_pid_rate_pitch,
//	k_param_pid_rate_yaw,
//	k_param_p_stabilize_roll,
//	k_param_p_stabilize_pitch,
//	k_param_p_stabilize_yaw,
//	k_param_p_pos_xy,
//	k_param_p_loiter_lon,       // remove
//	k_param_pid_loiter_rate_lat,    // remove
//	k_param_pid_loiter_rate_lon,    // remove
//	k_param_pid_nav_lat,        // remove
//	k_param_pid_nav_lon,        // remove
//	k_param_p_alt_hold,
//	k_param_p_vel_z,
//	k_param_pid_optflow_roll,       // remove
//	k_param_pid_optflow_pitch,      // remove
//	k_param_acro_balance_roll_old,  // remove
//	k_param_acro_balance_pitch_old, // remove
//	k_param_pid_accel_z,
//	k_param_acro_balance_roll,
//	k_param_acro_balance_pitch,
//	k_param_acro_yaw_p,
//	k_param_autotune_axis_bitmask,
//	k_param_autotune_aggressiveness,
//	k_param_pi_vel_xy,
//	k_param_fs_ekf_action,
//	k_param_rtl_climb_min,
//	k_param_rpm_sensor,
//	k_param_autotune_min_d, // 251
//
//	// 254,255: reserved
};

typedef struct PARAM
{
	float value;
	uint8_t key;
	char name[16];
	struct PARAM *next_param;
}T_PARAM;

extern T_PARAM param_all[];

extern uint8_t param_all_cnt;




#endif /* PARAMETERS_H_ */
