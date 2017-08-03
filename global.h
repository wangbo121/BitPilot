/*
 * global.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>
#include <inttypes.h>
#include "rc_channel.h"
#include "pid.h"

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define DEBUG 0

// Radio channels
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4

/*
 * Radio values
 *			Channel assignments
 *			1   Ailerons (rudder if no ailerons)
 *			2   Elevator
 *			3   Throttle
 *			4   Rudder (if we have ailerons)
 *			5   Mode
 *			6   TBD  to be done
 *			7   TBD
 *			8   TBD
 */

/*
 * 飞机的控制模式 从0开始是手动情况下的不同模式
 *  从10开始是自动情况下的不同模式
 */
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABILIZE 2

#define AUTO_OMISSION 10
#define AUTO_RTL 11
#define AUTO_LOITER 12
#define AUTO_TAKEOFF 13
#define AUTO_LAND 14

#ifndef LOW_VOLTAGE
# define LOW_VOLTAGE			9.6
#endif












class Global_Pilot{

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
	int8_t     waypoint_radius;
	int8_t     loiter_radius;

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
	int8_t     throttle_cruise;

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

	// RC channels
	AP_RC_Channel  channel_roll;
	AP_RC_Channel  channel_pitch;
	AP_RC_Channel  channel_throttle;
	AP_RC_Channel  channel_rudder;
	AP_RC_Channel	rc_5;
	AP_RC_Channel	rc_6;
	AP_RC_Channel	rc_7;
	AP_RC_Channel	rc_8;

      // BIT_PID controllers
      //
      BIT_PID         pidNavRoll;
      BIT_PID         pidServoRoll;
      BIT_PID         pidServoPitch;
      BIT_PID         pidNavPitchAirspeed;
      BIT_PID         pidServoRudder;
      BIT_PID         pidTeThrottle;
      BIT_PID         pidNavPitchAltitude;

      uint8_t     junk;
};



#endif /* GLOBAL_H_ */
