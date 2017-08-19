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
#include "rc.h"

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
#if 0
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABLE 2
#define ACRO 3

#define AUTO_OMISSION 10
#define AUTO_RTL 11
#define AUTO_LOITER 12
#define AUTO_TAKEOFF 13
#define AUTO_LAND 14
#endif

#ifndef LOW_VOLTAGE
# define LOW_VOLTAGE			9.6
#endif

// YAW debug
// ---------
#define YAW_STABILE 0
#define YAW_ACRO 1
//#define YAW_HOLD 0
//#define YAW_BRAKE 1
#define YAW_RATE 2

#define YAW_MANUAL 3
// Flight modes
// ------------
#define YAW_HOLD                        0
#define YAW_ACRO                        1
#define YAW_AUTO                        2
#define YAW_LOOK_AT_HOME    		    3
#define YAW_TOY                         4       // THOR This is the Yaw mode





#define ROLL_PITCH_STABLE       0
#define ROLL_PITCH_ACRO         1
#define ROLL_PITCH_AUTO         2
#define ROLL_PITCH_STABLE_OF    3
#define ROLL_PITCH_TOY          4       // THOR This is the Roll and Pitch
                                        // mode
#define ROLL_PITCH_MANUAL 5

#define THROTTLE_STABLE 0
#define THROTTLE_ACRO 1
#define THROTTLE_MANUAL         0
#define THROTTLE_HOLD           1
#define THROTTLE_AUTO           2




#define LOITER_MODE 1
#define WP_MODE 2
#define CIRCLE_MODE 3
#define NO_NAV_MODE 4
#define TOY_MODE 5                      // THOR This mode defines the Virtual
                                        // WP following mode

#define	MAV_ROI_NONE 0 /* No region of interest. | */
	#define MAV_ROI_WPNEXT    1 /* Point toward next MISSION. | */
	#define MAV_ROI_WPINDEX   2 /* Point toward given MISSION. | */
	#define MAV_ROI_LOCATION   3 /* Point toward fixed location. | */
	#define MAV_ROI_TARGET   4 /* Point toward of given id. | */
	#define MAV_ROI_ENUM_END  5 /*  | */



// definitions for earth frame and body frame
// used to specify frame to rate controllers
#define EARTH_FRAME     0
#define BODY_FRAME      1

// RADIANS
#define RADX100 0.000174532925
#define DEGX100 5729.57795

// DEFAULT PIDS

// roll
#define STABILIZE_ROLL_P 0.70
#define STABILIZE_ROLL_I 0.025
#define STABILIZE_ROLL_D 0.04
#define STABILIZE_ROLL_IMAX 7

//pitch
#define STABILIZE_PITCH_P 0.70
#define STABILIZE_PITCH_I 0.025
#define STABILIZE_PITCH_D 0.04
#define STABILIZE_PITCH_IMAX 7

// yaw stablise
#define STABILIZE_YAW_P  0.7
#define STABILIZE_YAW_I  0.02
#define STABILIZE_YAW_D  0.0

// yaw rate
#define RATE_YAW_P  0.135
#define RATE_YAW_I  0.0
#define RATE_YAW_D  0.0

// throttle
#define THROTTLE_P 0.2
#define THROTTLE_I 0.001
#define THROTTLE_IMAX 100

// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control  比例控制，其实也就是纯手动控制
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10            // Hold a single location using optical flow
                                // sensor
#define TOY_A 11                                // THOR Enum for Toy mode
#define TOY_M 12                                // THOR Enum for Toy mode
#define NUM_MODES 13

#define SIMPLE_1 1
#define SIMPLE_2 2
#define SIMPLE_3 4
#define SIMPLE_4 8
#define SIMPLE_5 16
#define SIMPLE_6 32





#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#ifndef WAYPOINT_SPEED_MIN
 # define WAYPOINT_SPEED_MIN             150                    // 1m/s
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


	  int16_t        auto_slew_rate;

      uint8_t     junk;
};





#define QUAD_MOTORS 4


typedef struct tagGLOBAL
{
	double cnt;

}T_GLOBAL;

typedef struct tagAP2FG
{
	double throttle0;//[0..1], 0-3为四个电机的控制量
	double throttle1;
	double throttle2;
	double throttle3;

	double rpm0;//[0..1], 0-3为四个电机的控制量
	double rpm1;
	double rpm2;
	double rpm3;


#if 0
  double latitude_deg;//[deg],飞行器当前纬度坐标
  double longitude_deg;//[deg],飞行器当前经度坐标
  double altitude_ft;//[ft],飞行器当前飞行高度
  double altitude_agl_ft;//[ft],
  double roll_deg;//[deg]滚转角
  double pitch_deg;//[deg]俯仰角
  double heading_deg;//[deg]机头朝向
#endif
}T_AP2FG;

typedef struct tagFG2AP
{
  double latitude_deg;//机场的纬度坐标，只在初始化时使用一次
  double longitude_deg;//机场的经度坐标，只在初始化时使用一次
  double pitch_deg;
  double heading_deg;//机头初始朝向
}T_FG2AP;


extern T_GLOBAL  gblState;
extern T_AP2FG  ap2fg;
extern T_FG2AP fg2ap;
extern T_AP2FG  ap2fg_send;


extern T_AP2FG  ap2fg_recv;




#endif /* GLOBAL_H_ */
