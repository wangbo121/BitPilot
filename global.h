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



// Commands - Note that APM now uses a subset of the MAVLink protocol
// commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location
                    // requested
#define NO_COMMAND 0

enum MAV_CMD
{
	MAV_CMD_NAV_WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_NAV_LAND=21, /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
	MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
	MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
	MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
	MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
	MAV_CMD_DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
	MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
	MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
	MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
	MAV_CMD_OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
	MAV_CMD_MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
	MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
	MAV_CMD_ENUM_END=401, /*  | */
};


// Waypoint options
#define MASK_OPTIONS_RELATIVE_ALT               1
#define WP_OPTION_ALT_CHANGE                    2
#define WP_OPTION_YAW                                   4
#define WP_OPTION_ALT_REQUIRED                  8
#define WP_OPTION_RELATIVE                              16
//#define WP_OPTION_					32
//#define WP_OPTION_					64
#define WP_OPTION_NEXT_CMD                              128

#ifndef WAYPOINT_SPEED_MAX
# define WAYPOINT_SPEED_MAX		600			// 6m/s error = 13mph
#endif



#ifndef AUTO_SLEW_RATE
 # define AUTO_SLEW_RATE         30                     // degrees
#endif


#ifndef WAYPOINT_SPEED_MAX
 # define WAYPOINT_SPEED_MAX             500                    // 6m/s error = 13mph
#endif

#ifndef WAYPOINT_SPEED_MIN
 # define WAYPOINT_SPEED_MIN             150                    // 1m/s
#endif




// nav byte mask
// -------------
#define NAV_LOCATION 1
#define NAV_ALTITUDE 2
#define NAV_DELAY    4

#define ASCENDING			1
#define DESCENDING			-1
#define REACHED_ALT			0









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
