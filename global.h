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

#ifndef LINUX_OS
#define LINUX_OS //这个是在linux上测试时用的，比如udp和串口通信
#endif

#ifdef LINUX_OS
extern struct T_UART_DEVICE uart_device_ap2gcs;
#endif

#include "location.h"
extern struct Location wp_total_array_temp[255];

/**
 * 简单打印调试信息
 */
#define DEBUG_SWITCH        1
#ifdef    DEBUG_SWITCH
//#define printf_debug(fmt,args...) printf(fmt, ##args)
#define DEBUG_PRINTF(fmt,args...) printf(fmt, ##args)
#else
#define DEBUG_PRINTF(fmt,args...) /*do nothing */
#endif

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

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

#ifdef LINUX_OS


#else
//#define	MAV_ROI_NONE 0 /* No region of interest. | */
//	#define MAV_ROI_WPNEXT    1 /* Point toward next MISSION. | */
//	#define MAV_ROI_WPINDEX   2 /* Point toward given MISSION. | */
//	#define MAV_ROI_LOCATION   3 /* Point toward fixed location. | */
//	#define MAV_ROI_TARGET   4 /* Point toward of given id. | */
//	#define MAV_ROI_ENUM_END  5 /*  | */
//
//#define MAV_CMD_NAV_WAYPOINT 16

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
   MAV_CMD_NAV_WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_LAND=21, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
   MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
   MAV_CMD_NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
   MAV_CMD_NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
   MAV_CMD_NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
   MAV_CMD_DO_FOLLOW=32, /* Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  */
   MAV_CMD_DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  */
   MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
   MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
   MAV_CMD_NAV_SPLINE_WAYPOINT=82, /* Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
   MAV_CMD_NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode |Empty| Front transition heading, see VTOL_TRANSITION_HEADING enum.| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
   MAV_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
   MAV_CMD_NAV_PAYLOAD_PLACE=94, /* Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload |Maximum distance to descend (meters)| Empty| Empty| Empty| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
   MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
   MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
   MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
   MAV_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
   MAV_CMD_DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_GO_AROUND=191, /* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
   MAV_CMD_DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
   MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
   MAV_CMD_DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
   MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| roll input (0 = angle, 1 = angular rate)| pitch input (0 = angle, 1 = angular rate)| yaw input (0 = angle, 1 = angular rate)|  */
   MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch depending on mount mode (degrees or degrees/second depending on pitch input).| roll depending on mount mode (degrees or degrees/second depending on roll input).| yaw depending on mount mode (degrees or degrees/second depending on yaw input).| alt in meters depending on mount mode.| latitude in degrees * 1E7, set if appropriate mount mode.| longitude in degrees * 1E7, set if appropriate mount mode.| MAV_MOUNT_MODE enum value|  */
   MAV_CMD_DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is trigerred each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance (meters). 0 to stop triggering.| Camera shutter integration time (milliseconds). -1 or 0 to ignore| Trigger camera once immediately. (0 = no trigger, 1 = trigger)| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  */
   MAV_CMD_DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change |yaw angle to adjust steering by in centidegress| speed - normalized to 0 .. 1| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time (milliseconds). -1 or 0 to ignore.| Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
   MAV_CMD_DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_GUIDED_LIMITS=222, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
   MAV_CMD_DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
   MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
   MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
   MAV_CMD_PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
   MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  */
   MAV_CMD_OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
   MAV_CMD_MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
   MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
   MAV_CMD_GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
   MAV_CMD_GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  */
   MAV_CMD_SET_MESSAGE_INTERVAL=511, /* Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  */
   MAV_CMD_REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility |1: Request supported protocol versions by all nodes on the network| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_CAMERA_INFORMATION=521, /* WIP: Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_CAMERA_SETTINGS=522, /* WIP: Request camera settings (CAMERA_SETTINGS). |0: No Action 1: Request camera settings| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_STORAGE_INFORMATION=525, /* WIP: Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| 0: No Action 1: Request storage information| Reserved (all remaining params)|  */
   MAV_CMD_STORAGE_FORMAT=526, /* WIP: Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| 0: No action 1: Format storage| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS=527, /* WIP: Request camera capture status (CAMERA_CAPTURE_STATUS) |0: No Action 1: Request camera capture status| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_FLIGHT_INFORMATION=528, /* WIP: Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)|  */
   MAV_CMD_RESET_CAMERA_SETTINGS=529, /* WIP: Reset all camera settings to Factory Default |0: No Action 1: Reset all settings| Reserved (all remaining params)|  */
   MAV_CMD_SET_CAMERA_MODE=530, /* Set camera running mode. Use NAN for reserved values. |Reserved (Set to 0)| Camera mode (see CAMERA_MODE enum)| Reserved (all remaining params)|  */
   MAV_CMD_IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NAN for reserved values. |Reserved (Set to 0)| Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Reserved (all remaining params)|  */
   MAV_CMD_IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence Use NAN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)|  */
   MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* WIP: Re-request a CAMERA_IMAGE_CAPTURE packet. Use NAN for reserved values. |Sequence number for missing CAMERA_IMAGE_CAPTURE packet| Reserved (all remaining params)|  */
   MAV_CMD_DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore|  */
   MAV_CMD_VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). Use NAN for reserved values. |Reserved (Set to 0)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)| Reserved (all remaining params)|  */
   MAV_CMD_VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). Use NAN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)|  */
   MAV_CMD_VIDEO_START_STREAMING=2502, /* WIP: Start video streaming |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| Reserved|  */
   MAV_CMD_VIDEO_STOP_STREAMING=2503, /* WIP: Stop the current video streaming |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| Reserved|  */
   MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION=2504, /* WIP: Request video stream information (VIDEO_STREAM_INFORMATION) |Camera ID (0 for all cameras, 1 for first, 2 for second, etc.)| 0: No Action 1: Request video stream information| Reserved (all remaining params)|  */
   MAV_CMD_LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NAN for no change)| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN|  */
   MAV_CMD_PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
   MAV_CMD_DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  */
   MAV_CMD_ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is resposible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle|  */
   MAV_CMD_SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
                   | */
   MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  */
   MAV_CMD_CONDITION_GATE=4501, /* WIP: Delay mission state machine until gate has been reached. |Geometry: 0: orthogonal to path between previous and next waypoint.| Altitude: 0: ignore altitude| Empty| Empty| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
   MAV_CMD_NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
   MAV_CMD_UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
   MAV_CMD_PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
   MAV_CMD_PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
   MAV_CMD_WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
   MAV_CMD_USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
   MAV_CMD_ENUM_END=31015, /*  | */
} MAV_CMD;
#endif

#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI
{
   MAV_ROI_NONE=0, /* No region of interest. | */
   MAV_ROI_WPNEXT=1, /* Point toward next MISSION. | */
   MAV_ROI_WPINDEX=2, /* Point toward given MISSION. | */
   MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
   MAV_ROI_TARGET=4, /* Point toward of given id. | */
   MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

#endif


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

#if 0
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
#endif

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



//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_STATUS,
    MSG_GPS_RAW,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_RETRY_DEFERRED // this must be last
};

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL
};












#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#ifndef WAYPOINT_SPEED_MIN
 # define WAYPOINT_SPEED_MIN             150                    // 1m/s
#endif

//typedef struct tagAll_SENSORS_IN
//{
//	//gps
//	int longitude;//度
//	int latitude;//度
//	int altitude;//米
//	int v_north;//米每秒
//	int v_east;//米每秒
//	int v_down;//米每秒
//
//	//imu
//	float _gyro_x;//弧度每秒
//	float _gyro_y;//弧度每秒
//	float _gyro_z;//弧度每秒
//	float _accel_x;//米每二次方秒
//	float _accel_y;//米每二次方秒
//	float _accel_z;//米每二次方秒
//
//	//rc 遥控器信号输入
//	float rc_raw_in_0;//1000～2000
//	float rc_raw_in_1;
//	float rc_raw_in_2;
//	float rc_raw_in_3;
//	float rc_raw_in_4;
//	float rc_raw_in_5;
//	float rc_raw_in_6;
//	float rc_raw_in_7;
//	float rc_raw_in_8;
//
//
//
//
//
//
//}T_ALL_SENSORS_IN;
//
//typedef struct tagAll_SENSORS_OUT
//{
//
//	float rc_raw_out_0;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_1;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_2;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_3;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_4;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_5;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_6;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_7;//这个给到ucos系统中的pwm任务，输出pwm波
//	float rc_raw_out_8;//这个给到ucos系统中的pwm任务，输出pwm波
//
//}T_ALL_SENSORS_OUT;



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

     // struct   Location current_loc;

//      T_ALL_SENSORS_IN all_sensors_in;//从飞控模块的外部获取的传感器数据，本来应该是从hal中获取，但是我们现在利用ucos中的一个全局变量获取
//
//      T_ALL_SENSORS_OUT all_sensors_out;//从飞控模块的输出给外部的传感器，本来应该是从hal中输出，但是我们现在利用ucos中的一个全局变量输出，跟飞控核心的代码就没什么关系了




};





#define QUAD_MOTORS 4




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

extern T_AP2FG  ap2fg;
extern T_FG2AP fg2ap;
extern T_AP2FG  ap2fg_send;

struct T_GLOBAL_BOOL_BOATPILOT
{
	unsigned char bool_get_gcs2ap_cmd;//电台获取到地面站的命令包
	unsigned char bool_get_gcs2ap_waypoint;//电台获取到地面站的航点包
    unsigned char bool_gcs2ap_beidou;//0:不解析北斗数据，1:解析北斗接收的地面站数据包，并采用
    unsigned char bool_generator_on;//发电机 0:不工作，1:工作
    unsigned char bool_is_sending_wp_ap2gcs;//电台 在回传全部航点时，作为正在回传的标志
    unsigned char bool_beidou_get_gcs2ap_cmd;
    unsigned char bool_beidou_get_gcs2ap_waypoint;
    unsigned char bool_loiter_mode;//停留模式=true
    unsigned char bool_shutdown_master;//主控重启
    unsigned char bool_shutdown_slave;//副控重启
    unsigned char bool_rudder_calib_success;//方向舵极限值标定成功
    unsigned char bool_is_calib_rudder;//正在标定状态，方向舵可以随意摆动
    unsigned char turn_mode;//转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯
    unsigned char s2m_generator_onoff_req_previous;//上一次副控向主控请求发电机的工作状态
    unsigned char radio_recv_packet_cnt;//电台 地面站发送给驾驶仪的数据包的计数，计数不同时，驾驶仪才接收地面站的数据，否则则舍弃
    unsigned char radio_recv_packet_cnt_previous;
    unsigned char udp_recv_packet_cnt;//只是用来记录主控接收到副控的udp的数据包计数，没有用作别的用途
    unsigned char wp_total_num;//wp_data[]数组记录了航点，wp_total_num则用来表示每次自动驾驶时地面站发给驾驶仪的航点总个数，最小是1
    unsigned char send_ap2gcs_wp_req;//驾驶仪 发送航点请求
    unsigned char ap2gcs_wp_cnt_previous;//驾驶仪 发送航点计数
    unsigned char ap2gcs_wp_cnt;
    unsigned char send_ap2gcs_real_req;//驾驶仪 发送实时数据请求
    unsigned char ap2gcs_real_cnt_previous;//驾驶仪 发送实时数据计数
    unsigned char ap2gcs_real_cnt;
    unsigned char send_ap2gcs_cmd_req;//驾驶仪 发送(回传)命令请求
    unsigned char ap2gcs_cmd_cnt_previous;//驾驶仪 发送(回传)命令计数
    unsigned char ap2gcs_cmd_cnt;
    unsigned char send_m2s_udp_req;//主控给副控发送udp数据包请求
    unsigned char m2s_udp_cnt_previous;//主控给副控发送udp数据包计数
    unsigned char m2s_udp_cnt;
    unsigned char bd_send_ap2gcs_wp_req;
    unsigned char bd_ap2gcs_wp_cnt_previous;
    unsigned char bd_ap2gcs_wp_cnt;
    unsigned char bd_send_ap2gcs_real_req;
    unsigned char bd_ap2gcs_real_cnt_previous;
    unsigned char bd_ap2gcs_real_cnt;
    unsigned char bd_send_ap2gcs_cmd_req;
    unsigned char bd_ap2gcs_cmd_cnt_previous;
    unsigned char bd_ap2gcs_cmd_cnt;
    unsigned char rudder_calib_cnt_previous;//标定方向舵的左极限右极限和中间值
    unsigned char launch_req_ack_cnt_previous;//副控确认火箭可发射后，向主控提出发射火箭请求
    unsigned char save_boatpilot_log_req;//保存无人船日志请求，这个是定时保存，每秒存储一个日志结构
    unsigned char wp_packet_cnt;//地面站发送给驾驶仪的第wp_packet_cnt个航点数据包
    unsigned char assign_config_req;//当驾驶仪接收到地面站的命令包时，驾驶仪认为需要更新配置(也就是一些参数设置)
    unsigned char assign_config_cnt_previous;
    unsigned char assign_config_cnt;
    unsigned char save_config_req;//驾驶仪中的config结构发生变化时，就提出保存config请求
    unsigned char set_switch_channel_previous;//切换器 上一次的通道
    unsigned char voltage_llim_previous;//切换器 放电电压最低值
    unsigned char voltage_hlim_previous;//切换器 放电电压最高值
    unsigned char bat0_is_discharing;//切换器 电池0通道正在放电
    unsigned char bat1_is_discharing;//切换器 电池1通道正在放电
    unsigned char charger_set_channel_previous;//充电机 上一次的通道
    unsigned char charger_set_voltage_previous;//充电机 上一次的电压
    unsigned char charger_set_current_previous;//充电机 上一次的电流
    unsigned char charge_start_previous;//充电机 上次的开机关机状态，只有检测到与当前状态不同时才改变
    unsigned char wp_next;//自动驾驶时，由地面站设置的或者改变的当前目标航点，最小值0
    unsigned char send_ap2gcs_wp_start_num;//发送航点数据包时，起始航点数，最小值0
    unsigned char send_ap2gcs_wp_end_num;//发送航点数据包时，航点数木，最小值1
    unsigned char send_ap2gcs_specific_wp_req;//地面站请求驾驶仪回传特定的某几个航点，不是回传全部航点
    unsigned char master_state;//主控的状态 D7:电台等待超时 D6:gps等待超时 D5:modbus继电器模拟量等待超时 D4:modbus码盘等待超时 D3:udp等待超时 D2:北斗等待超时 D1: D0:
    unsigned char slave_state;//副控的状态
    unsigned char slave_config_previous;//地面站配置副控，现在用来控制是否读取485的电流通道
    unsigned char spare0;//64字节

    short left_motor_voltage;//单位[0.1伏特]
    short right_motor_voltage;//单位[0.1伏特]
    short rudder_angle_degree;//单位[度] 范围[-45-+45度]这是通过码盘读回来的真实的方向舵的角度值，通过实时数据返回给地面站
    short cte_error_check_radian;//psi_r根据偏航距得到的修正方向舵角[-3.14*1000-+3.14*1000]
    short current_to_target_radian;//[-180*1000-+180*1000][0.001弧度]
    short command_radian;//[-180*1000-+180*1000][0.001弧度]
    short dir_target_degree;//[0.01度]当前位置与目标航点位置的方位角bearing angle
    short dir_nav_degree;//80字节//[0.01度]制导算法得到的导航目标航迹角course angle或者航向角heading angle

    int cte_distance_error;//84字节//[0.01米]

#if 0
    //float rudder_middle_position;//[0-720]方向舵标定时，定义的方向舵的中位的2倍，初始值设置为360线(驾驶仪认为是180度)，码盘的360度对应的是720线
    float rudder_middle_position;//方向舵处于中间位置时对应的码盘的读数
    float rudder_left_limit_position;//[0-360]方向舵标定时的左极限，初始默认180-45=135度
    float rudder_right_limit_position;//[0-360]方向舵标定时的右极限，初始默认180+45=225度
    float rudder_delta_fabs;//[0-90]20170508    [0-45]方向舵标定时，为保证向左和向右转动时左右极限值对称，故采用此变化范围
#else
    short rudder_middle_position;//方向舵处于中间位置时对应的码盘的读数
    short rudder_left_limit_position;//[0-720]方向舵标定时的左极限位置，对应的码盘读数
    short rudder_right_limit_position;//[0-720]方向舵标定时的右极限位置，对应的码盘读数
    short rudder_delta_fabs;//92字节//[0-90]方向舵标定时，为保证向左和向右转动时左右极限值对称，故有此变量
#endif

    /*
     * 以后改变global变量结构
     * 必须从这个以后添加，否则需要修改日志记录那里
     * 20170518
     */
    unsigned char gcs2ap_wp_cnt;//电台--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char gcs2ap_cmd_cnt;//电台--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char bd_gcs2ap_wp_cnt;//北斗--接收到的地面站发送给驾驶仪的航点包计数
    unsigned char bd_gcs2ap_cmd_cnt;//北斗--接收到的地面站发送给驾驶仪的命令包计数

    float radio_send_time;
    float radio_need_to_send_time;
    float radio_send_delta_time;
    float radio_get_data_previous_time_s;//电台获取到数据的前一时间单位秒[s]
    float radio_lose_data_time_s;//电台获取到数据的前一时间单位秒[s]

    float radio_wait_time;
    float gps_wait_time;
    float modbus_wait_time;
    float modbus_rotary_wait_time;
    float udp_wait_time;
    float bd_wait_time;
};

typedef struct T_DateTime
{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char stuffing;//填充字节，保证数据包字节数为4的整数倍
}T_DATETIME;

#define GPS_LOG_FILE "gps.log"
#define REALTIME_LOG_FILE "realtime.log"
#define BOATPILOT_LOG_FILE "boatpilot.log"
#define WAY_POINT_FILE "waypoint.log"
#define CONFIG_FILE "config.log"

#define BEIDOU_LOG_FILE "BD.dat"

struct AP2GCS_REAL
{
    unsigned char head1;
    unsigned char head2;
    unsigned char len;
    unsigned char cnt;
    unsigned char sysid;
    unsigned char type;
    unsigned char commu_method;
    unsigned char ack_req;//8字节

    unsigned char pack_func_flag;//包功能标志，暂时固定为0
    unsigned char pack_func_info1;//接收到的命令包计数
    unsigned char pack_func_info2;//接收到的航点包计数
    unsigned char pack_func_info3;//包功能辅助信息，在接收到航点包时，作为航点包的计数
    unsigned int lng;//[度*0.00001]，GPS经度坐标，整型，精确到米
    unsigned int lat;//[度*0.00001]，GPS纬度坐标，整型，精确到米
    unsigned short spd;//[Knot*0.01]，实时航速
    short dir_gps;//24个字节//[度*0.01]，地速航向，GPS航向
    short dir_heading;//[度*0.01]，机头朝向
    short dir_target;//[度*0.01]，目标点朝向
    short dir_nav;//[度*0.01]，导航航向
    short roll;//32个字节//[度*0.01]，滚转
    short pitch;//[度*0.01]，俯仰
    short yaw;//[度*0.01]，偏航
    unsigned char codedisc;//码盘实时采集返回值 0-360
    unsigned char da_out1;//[0.1V]电调给定值1
    unsigned char da_out2;//[0.1V]电调给定值2
    unsigned char rudder_pos;//32个字节//[度],方向舵角度值,中位为45
    unsigned char rc_thruster;//[0-255]RC推进器通道值
    unsigned char rc_rudder;//[0-255]RC方向舵通道值
    unsigned char rud_p;//[0.1],转弯参数P
    unsigned char cte_p;//[0.1],CTE参数P
    unsigned char boat_temp1;//[C],船内温度1
    unsigned char boat_temp2;//[C],船内温度2
    unsigned char boat_humi;//[%],船内湿度,预留,可作它用
    unsigned char voltage_bat1;//48//[V],电池组1实时电压(切换器上的电压)
    unsigned char voltage_bat2;//[V],电池组2实时电压(切换器上的电压)
    unsigned char current_bat1;//[0.1A],电池组1实时放电电流(电路互感器上检测的电流)
    unsigned char current_bat2;//[0.1A],电池组2实时放电电流(电路互感器上检测的电流)
    unsigned char toggle_state;//切换器状态,D1D0:当前放电通道,01:通道1,10:通道2; D3D2:通道1工作状态:01:0x55正在放电,10:0x5A请求放电;11:0xAA停止放电; D5D4:通道2工作状态; D7D6:预留
    unsigned char charge_state;//充电机状态,D1D0:当前充电通道,01:通道1,10:通道2,11:通道3; D2:开关机状态,0:开机,1:关机; D3: 手动自动状态, 0:自动,1:手动; D6-4:保留; D7:发电机状态,0:停止,1:工作
    unsigned char temp;//气象站数据：温度1
    unsigned char humi;//气象站数据：湿度
    unsigned char windspeed;//56//气象站数据：风速
    unsigned char winddir;//气象站数据：风向
    unsigned char airpress;//气象站数据：气压
    unsigned char seasault;//气象站数据：海盐
    unsigned char elec_cond;//气象站数据：电导率
    unsigned char seatemp1;//气象站数据：海温1
    unsigned char seatemp2;//气象站数据：海温2
    unsigned char seatemp3;//气象站数据：海温3
    unsigned char alt;//56//气象站数据：高度
    unsigned char radiation;//气象站数据：辐射
    unsigned char launch_req_ack;//火箭发射请求
    unsigned char rocket_state;//火箭系统状态
    unsigned char rktnumber;//火箭编号
    unsigned char rkt_alt;//[10m]火箭升空高度
    unsigned char work_mode;
    unsigned char charger_voltage;
    unsigned char charger_current;
    unsigned char wp_next;
    unsigned char master_state;
    unsigned char slave_state;
    unsigned char checksum;//76个字节

};

extern struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;


#endif /* GLOBAL_H_ */
