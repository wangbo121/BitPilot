/*
 * copter.h
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

#ifndef COPTER_H_
#define COPTER_H_


////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include "location.h"



// Common dependencies


// Application dependencies
#include <iostream>
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>

#include "BIT_HAL.h"

// Libraries
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "utility.h"

#include "gps.h"        // ArduPilot GPS library
#include "gps_nmea.h"
#include "compass.h"     // ArduPilot Mega Magnetometer Library
#include "compass_hmc5843.h"
#include "BIT_MATH.h"        // ArduPilot Mega Vector/Matrix math Library
#include "imu.h"         // ArduPilot Mega IMU Library
//#include "imu_oilpan.h"
#include "ahrs_DCM.h"         // ArduPilot Mega DCM Library
#include "pid.h"            // PID library
#include "rc.h"         // ArduPilot Mega RC Library
#include "rc_channel.h"     // RC Channel Library
#include "motors.h"

// Local modules
#include "global.h"
//#include "GCS.h"

class Copter :public AP_HAL::HAL::Callbacks{
public:

    Copter(void)
	{
    	/*
    	 * 在构造函数的开始就初始化一些内部变量
    	 */
    	control_mode            = STABILIZE;
	}



    void setup() override;
    void loop() override;

public:
    void mavlink_delay_cb();
    void failsafe_check();

    int8_t reboot_board();

    /**
	 * 我自己添加了一些
	 */
	void navigate();
	//void get_stabilize_roll(int32_t target_angle);


private:

	Vector3f omega;
    // Global parameters are all contained within the 'g' class.
    Global_Pilot g;

    AP_RC ap_rc;

    // primary input control channels
    AP_RC_Channel *channel_roll;
    AP_RC_Channel *channel_pitch;
    AP_RC_Channel *channel_throttle;
    AP_RC_Channel *channel_yaw;




    // flight modes convenience array
    int8_t *flight_modes;

    //AP_Baro barometer;
    //AP_Compass_HMC5843 compass;

    // Location & Navigation
    int32_t wp_bearing;
    // The location of home in relation to the copter in centi-degrees
    int32_t home_bearing;
    // distance between plane and home in cm
    int32_t home_distance;
    // distance between plane and next waypoint in cm.
    uint32_t wp_distance;

    // Circle
    bool circle_pilot_yaw_override; // true if pilot is overriding yaw

    // SIMPLE Mode
    // Used to track the orientation of the copter for Simple mode. This value is reset at each arming
    // or in SuperSimple mode when the copter leaves a 20m radius from home.
    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;

    // Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
    int32_t initial_armed_bearing;

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    // Brake
    uint32_t brake_timeout_start;
    uint32_t brake_timeout_ms;

    // Delay the next navigation command
    int32_t nav_delay_time_max;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start;

    // Flip
    Vector3f flip_orig_attitude;         // original copter attitude before flip


    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;   // desired altitude in cm above the ground
    int32_t baro_alt;            // barometer altitude in cm above home
    float baro_climbrate;        // barometer climbrate in cm/s

    // Navigation Yaw control
    // auto flight mode's yaw mode
    uint8_t auto_yaw_mode;

    // Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
    Vector3f roi_WP;

    // bearing from current location to the yaw_look_at_WP
    float yaw_look_at_WP_bearing;

    // yaw used for YAW_LOOK_AT_HEADING yaw_mode
    int32_t yaw_look_at_heading;

    // Deg/s we should turn
    int16_t yaw_look_at_heading_slew;

    // heading when in yaw_look_ahead_bearing
    float yaw_look_ahead_bearing;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // IMU variables
    // Integration time (in seconds) for the gyros (DCM algorithm)
    // Updated with the fast loop
    float G_Dt;

    // Inertial Navigation


    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
    //AC_AttitudeControl *attitude_control;
    //AC_PosControl *pos_control;
    //AC_WPNav *wp_nav;
    //AC_Circle *circle_nav;

    // Performance monitoring
    int16_t pmTest1;

    // System Timers
    // --------------
    // Time in microseconds of main control loop
    //uint32_t fast_loopTimer;
    float fast_loopTimer;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;
    // Loiter timer - Records how long we have been in loiter
    uint32_t rtl_loiter_start_time;
    // arm_time_ms - Records when vehicle was armed. Will be Zero if we are disarmed.
    uint32_t arm_time_ms;

    // Used to exit the roll and pitch auto trim function
    uint8_t auto_trim_counter;

    // All GPS access should be through this pointer.

    AP_GPS_NMEA gps;
    /*
     * 下面这些都是真实传感器的实例化
     */
    // real sensors
    //AP_ADC_ADS7844          adc;
    //APM_BMP085_Class        barometer;
    //AP_Compass_HMC5843      compass();

    //AP_DCM              dcm;
    //AP_IMU_Oilpan *imu;

    IMU imu;

//    AP_DCM dcm;

    //AP_DCM_Oilpan *ahrs;

    AP_Compass_HMC5843 compass;
    AP_Motors motors;



   // AP_DCM dcm;
	AP_DCM ahrs{imu,gps,compass};
	//AP_DCM ahrs(&imu,&gps,&compass);
#if 0
	AP_DCM ahrs():
			_imu(imu),_gps(gps),_compass(compass)
	{

	};
#endif
    /*
     * 这里得读取gps数据
     */
    // real GPS selection


    //AP_RangeFinder_MaxsonarXL sonar;

    ////////////////////////////////////////////////////////////////////////////////
    // Global variables
    ////////////////////////////////////////////////////////////////////////////////

    int8_t    control_mode;
    int8_t    oldSwitchPosition;              // for remembering the control mode switch




    // Radio
    // -----
    uint16_t elevon1_trim  = 1500; 	// TODO: handle in EEProm
    uint16_t elevon2_trim  = 1500;
    uint16_t ch1_temp      = 1500;     // Used for elevon mixing
    uint16_t ch2_temp  	= 1500;


    // GPS variables
    // -------------
    const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
    float 	scaleLongUp			= 1;			// used to reverse longtitude scaling
    float 	scaleLongDown 		= 1;			// used to reverse longtitude scaling
    int8_t 	ground_start_count	= 5;			// have we achieved first lock and set Home?
    int     ground_start_avg;					// 5 samples to avg speed for ground start
    int8_t ground_start;    					// have we started on the ground?
    bool	GPS_enabled 	= false;			// used to quit "looking" for gps with auto-detect if none present

    // Location & Navigation
    // ---------------------
    const	float radius_of_earth 	= 6378100;	// meters
    const	float gravity 			= 9.81;		// meters/ sec^2
    long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
    int32_t	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
    long	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
    float	nav_gain_scaler 		= 1;		// Gain scaling for headwind/tailwind TODO: why does this variable need to be initialized to 1?
    long    hold_course       	 	= -1;		// deg * 100 dir of plane

    int8_t	command_must_index;					// current command memory location
    int8_t	command_may_index;					// current command memory location
    int8_t	command_must_ID;					// current command ID
    int8_t	command_may_ID;						// current command ID

    // Airspeed
    // --------
    int		airspeed;							// m/s * 100
    int     airspeed_nudge;  					// m/s * 100 : additional airspeed based on throttle stick position in top 1/2 of range
    float   airspeed_error;						// m/s * 100
    long    energy_error;                       // energy state error (kinetic + potential) for altitude hold
    long    airspeed_energy_error;              // kinetic portion of energy error
    int8_t airspeed_enabled = false;

    // Location Errors
    // ---------------
    long	bearing_error;						// deg * 100 : 0 to 36000
    long	altitude_error;						// meters * 100 we are off in altitude
    float	crosstrack_error;					// meters we are off trackline

    // Battery Sensors
    // ---------------
    float	battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
    float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
    float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2, initialized above threshold for filter
    float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3, initialized above threshold for filter
    float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3 + 4, initialized above threshold for filter

    float	current_amps;
    float	current_total;

    // Airspeed Sensors
    // ----------------
    float   airspeed_raw;                       // Airspeed Sensor - is a float to better handle filtering
    int     airspeed_offset;					// analog air pressure sensor while still
    int     airspeed_pressure;					// airspeed as a pressure value

    // Barometer Sensor variables
    // --------------------------
    unsigned long 	abs_pressure;

    // Altitude Sensor variables
    // ----------------------
    //int8_t 	altitude_sensor = BARO;				// used to know which sensor is active, BARO or SONAR

    // flight mode specific
    // --------------------
    int8_t takeoff_complete    = true;         // Flag for using gps ground course instead of IMU yaw.  Set false when takeoff command processes.
    int8_t	land_complete;
    long	takeoff_altitude;
    int			landing_distance;					// meters;
    int			landing_pitch;						// pitch for landing set by commands
    int			takeoff_pitch;

    // Loiter management
    // -----------------
    long 	old_target_bearing;					// deg * 100
    int		loiter_total; 						// deg : how many times to loiter * 360
    int 	loiter_delta;						// deg : how far we just turned
    int		loiter_sum;							// deg : how far we have turned around a waypoint

    // these are the values for navigation control functions
    // ----------------------------------------------------
    long	nav_roll;							// deg * 100 : target roll angle
    long	nav_pitch;							// deg * 100 : target pitch angle
    int     throttle_nudge = 0;                 // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel

    // Waypoints
    // ---------
    //long	wp_distance;						// meters - distance between plane and next waypoint
    long	wp_totalDistance;					// meters - distance between old and next waypoint
    int8_t	next_wp_index;						// Current active command index

    // repeating event control
    // -----------------------
    int8_t 		event_id; 							// what to do - see defines
    long 		event_timer; 						// when the event was asked for in ms
    uint16_t 	event_delay; 						// how long to delay the next firing of event in millis
    int 		event_repeat = 0;					// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
    int 		event_value; 						// per command value, such as PWM for servos
    int 		event_undo_value;					// the value used to cycle events (alternate value to event_value)

    // delay command
    // --------------
    //long 	condition_value;						// used in condition commands (eg delay, change alt, etc.)
    //long 	condition_start;
    int 	condition_rate;

    #if 0
    // 3D Location vectors
    // -------------------
    struct 	Location home;						// home location
    struct 	Location prev_WP;					// last waypoint
    struct 	Location current_loc;				// current location
    struct 	Location next_WP;					// next waypoint
    struct 	Location next_command;				// command preloaded
    long 	target_altitude;					// used for altitude management between waypoints
    long 	offset_altitude;					// used for altitude management between waypoints
    int8_t	home_is_set; 						// Flag for if we have g_gps lock and have set the home location
    #endif

    // IMU variables
    // -------------
    //float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)


    // Performance monitoring
    // ----------------------
    long 	perf_mon_timer;					// Metric based on accel gain deweighting
    int 	G_Dt_max;							// Max main loop cycle time in milliseconds
    int 	gps_fix_count;
    int8_t	gcs_messages_sent;


    // GCS
    // ---
    char GCS_buffer[53];
    char display_PID = -1;						// Flag used by DebugTerminal to indicate that the next PID calculation with this index should be displayed

    // System Timers
    // --------------
    //unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
    unsigned long 	fast_loopTimeStamp;			// Time Stamp when fast loop was complete
    uint8_t 		delta_ms_fast_loop; 		// Delta Time in miliseconds
    //int 			mainLoop_count;

    unsigned long 	medium_loopTimer;			// Time in miliseconds of medium loop
    int8_t 			medium_loopCounter;			// Counters for branching from main control loop to slower loops
    uint8_t			delta_ms_medium_loop;

    int8_t 			slow_loopCounter;
    int8_t 			superslow_loopCounter;
    int8_t			counter_one_herz;

    unsigned long 	nav_loopTimer;				// used to track the elapsed ime for GPS nav

    unsigned long 	dTnav;						// Delta Time in milliseconds for navigation computations
    unsigned long 	elapsedTime;				// for doing custom events
    float 			load;						// % MCU cycles used

   uint8_t yaw_mode;
    // The current desired control scheme for roll and pitch / navigation
   uint8_t roll_pitch_mode;
    // The current desired control scheme for altitude hold
    uint8_t throttle_mode;

    // Attitude control variables
    float command_rx_roll=0;        // User commands
    float command_rx_roll_old;
    float command_rx_roll_diff;
    float command_rx_pitch=0;
    float command_rx_pitch_old;
    float command_rx_pitch_diff;
    float command_rx_yaw=0;
    float command_rx_yaw_diff;
    int control_roll;           // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
    int control_pitch; // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
    int control_yaw; // PID control results 这个在完全手控情况下直接就是rc_channel的rc_in，所以范围有两种1是-4500～4500 2是0～1000
    //float K_aux;

	  ////////////////////////////////////////////////////////////////////////////////
	  // Rate contoller targets
	  ////////////////////////////////////////////////////////////////////////////////
	  uint8_t rate_targets_frame;    // indicates whether rate targets provided in earth or body frame
	   int32_t roll_rate_target_ef;
	  int32_t pitch_rate_target_ef;
	  int32_t yaw_rate_target_ef;
	  int32_t roll_rate_target_bf ;     // body frame roll rate target
	  int32_t pitch_rate_target_bf ;    // body frame pitch rate target
	  int32_t yaw_rate_target_bf;      // body frame yaw rate target

	  ////////////////////////////////////////////////////////////////////////////////
	  // Orientation
	  ////////////////////////////////////////////////////////////////////////////////
	  // Convienience accessors for commonly used trig functions. These values are generated
	  // by the DCM through a few simple equations. They are used throughout the code where cos and sin
	  // would normally be used.
	  // The cos values are defaulted to 1 to get a decent initial value for a level state
	  float cos_roll_x;
	  float cos_pitch_x;
	  float cos_yaw_x ;
	  float sin_yaw_y;
	  float sin_roll;
	  float sin_pitch;

	  // An additional throttle added to keep the copter at the same altitude when banking
	  int16_t angle_boost;

	  ////////////////////////////////////////////////////////////////////////////////
	  // PIDs
	  ////////////////////////////////////////////////////////////////////////////////
	  // This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
	  // and not the adjusted omega rates, but the name is stuck

	  // This is used to hold radio tuning values for in-flight CH6 tuning
	  float tuning_value;
	  // This will keep track of the percent of roll or pitch the user is applying
	  float roll_scale_d, pitch_scale_d;


	  /*
	   * 20170819为了实现航点飞行 添加的变量
	   */

	  ////////////////////////////////////////////////////////////////////////////////
	  // Navigation general
	  ////////////////////////////////////////////////////////////////////////////////
	  // The location of the copter in relation to home, updated every GPS read
	   int32_t home_to_copter_bearing;
	  // distance between plane and home in cm
	   //int32_t home_distance;
	  // distance between plane and next_WP in cm
	  // is not  because AP_Camera uses it
	  //int32_t wp_distance;

	  ////////////////////////////////////////////////////////////////////////////////
	  // 3D Location vectors
	  ////////////////////////////////////////////////////////////////////////////////
	  // home location is stored when we have a good GPS lock and arm the copter
	  // Can be reset each the copter is re-armed
	   struct   Location home;
	  // Flag for if we have g_gps lock and have set the home location
	   uint8_t home_is_set;
	  // Current location of the copter
	   struct   Location current_loc;
	  // lead filtered loc
	   struct   Location filtered_loc;
	  // Next WP is the desired location of the copter - the next waypoint or loiter location
	   struct   Location next_WP;
	  // Prev WP is used to get the optimum path from one WP to the next
	   struct   Location prev_WP;
	  // Holds the current loaded command from the EEPROM for navigation
	   struct   Location command_nav_queue;
	  // Holds the current loaded command from the EEPROM for conditional scripts
	   struct   Location command_cond_queue;
	  // Holds the current loaded command from the EEPROM for guided mode
	   struct   Location guided_WP;



	  ////////////////////////////////////////////////////////////////////////////////
	  // Navigation Roll/Pitch functions
	  ////////////////////////////////////////////////////////////////////////////////
	  // all angles are deg * 100 : target yaw angle
	  // The Commanded ROll from the autopilot.
	   //int32_t nav_roll;
	  // The Commanded pitch from the autopilot. negative Pitch means go forward.
	   //int32_t nav_pitch;
	  // The desired bank towards North (Positive) or South (Negative)
	   int32_t auto_roll;
	   int32_t auto_pitch;

	  // Don't be fooled by the fact that Pitch is reversed from Roll in its sign!
	   int16_t nav_lat;
	  // The desired bank towards East (Positive) or West (Negative)
	   int16_t nav_lon;
	  // The Commanded ROll from the autopilot based on optical flow sensor.
	   int32_t of_roll;
	  // The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
	   int32_t of_pitch;
	   bool slow_wp ;


	  ////////////////////////////////////////////////////////////////////////////////
	  // Navigation Throttle control
	  ////////////////////////////////////////////////////////////////////////////////
	  // The Commanded Throttle from the autopilot.
	   int16_t nav_throttle;                                            // 0-1000 for throttle control
	  // This is a simple counter to track the amount of throttle used during flight
	  // This could be useful later in determining and debuging current usage and predicting battery life
	   uint32_t throttle_integrator;

	  ////////////////////////////////////////////////////////////////////////////////
	  // Climb rate control
	  ////////////////////////////////////////////////////////////////////////////////
	  // Time when we intiated command in millis - used for controlling decent rate
	  // Used to track the altitude offset for climbrate control
	   int8_t alt_change_flag;

	  ////////////////////////////////////////////////////////////////////////////////
	  // Navigation Yaw control
	  ////////////////////////////////////////////////////////////////////////////////
	  // The Commanded Yaw from the autopilot.
	   int32_t nav_yaw;
	  // A speed governer for Yaw control - limits the rate the quad can be turned by the autopilot
	   int32_t auto_yaw;
	  // Used to manage the Yaw hold capabilities -
	   bool yaw_stopped;
	   uint8_t yaw_timer;
	  // Options include: no tracking, point at next wp, or at a target
	//   byte yaw_tracking = MAV_ROI_WPNEXT;
	  // In AP Mission scripting we have a fine level of control for Yaw
	  // This is our initial angle for relative Yaw movements
	   int32_t command_yaw_start;
	  // Timer values used to control the speed of Yaw movements
	   uint32_t command_yaw_start_time;
	   uint16_t command_yaw_time;                                       // how long we are turning
	   int32_t command_yaw_end;                                         // what angle are we trying to be
	  // how many degrees will we turn
	   int32_t command_yaw_delta;
	  // Deg/s we should turn
	   int16_t command_yaw_speed;
	  // Direction we will turn –  1 = CW, 0 or -1 = CCW
	   //byte command_yaw_dir;
	   uint8_t command_yaw_dir;
	  // Direction we will turn – 1 = relative, 0 = Absolute
	   uint8_t command_yaw_relative;
	  // Yaw will point at this location if yaw_tracking is set to MAV_ROI_LOCATION
	   struct   Location target_WP;
	   uint8_t wp_control;
	   //uint8_t   yaw_tracking = MAV_ROI_WPNEXT;
	   uint8_t   yaw_tracking ;


	   ////////////////////////////////////////////////////////////////////////////////
	   // Crosstrack
	   ////////////////////////////////////////////////////////////////////////////////
	   // deg * 100, The original angle to the next_WP when the next_WP was set
	   // Also used to check when we pass a WP
	   int32_t original_target_bearing;
	   // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
	   // int16_t crosstrack_error;
	   // should we take the waypoint quickly or slow down?
	   uint8_t fast_corner;
	   ////////////////////////////////////////////////////////////////////////////////
	   // The GPS based velocity calculated by offsetting the Latitude and Longitude
	   // updated after GPS read - 5-10hz
	   int16_t x_actual_speed;
	   int16_t y_actual_speed;


	   ////////////////////////////////////////////////////////////////////////////////
	   // Location & Navigation
	   ////////////////////////////////////////////////////////////////////////////////
	   // Status flag indicating we have data that can be used to navigate
	   // Set by a GPS read with 3D fix, or an optical flow read
	    uint8_t nav_ok;
	   // This is the angle from the copter to the "next_WP" location in degrees * 100
	    //int32_t target_bearing;
	   // Status of the Waypoint tracking mode. Options include:
	   // NO_NAV_MODE, WP_MODE, LOITER_MODE, CIRCLE_MODE
	    //uint8_t wp_control;
	   // Register containing the index of the current navigation command in the mission script
	    int16_t command_nav_index;
	   // Register containing the index of the previous navigation command in the mission script
	   // Used to manage the execution of conditional commands
	    uint8_t prev_nav_index;
	   // Register containing the index of the current conditional command in the mission script
	    uint8_t command_cond_index;
	   // Used to track the required WP navigation information
	   // options include
	   // NAV_ALTITUDE - have we reached the desired altitude?
	   // NAV_LOCATION - have we reached the desired location?
	   // NAV_DELAY    - have we waited at the waypoint the desired time?
	    uint8_t wp_verify_byte;                                                  // used for tracking state of navigating waypoints
	   // used to limit the speed ramp up of WP navigation
	   // Acceleration is limited to .5m/s/s
	    int16_t waypoint_speed_gov;
	   // Used to track how many cm we are from the "next_WP" location
	    int32_t long_error, lat_error;
	   // Are we navigating while holding a positon? This is set to false once the speed drops below 1m/s
	    uint8_t loiter_override;
	    int16_t waypoint_radius;




	    // The difference between the desired rate of travel and the actual rate of travel
	    // updated after GPS read - 5-10hz
	     int16_t x_rate_error;
	     int16_t y_rate_error;


	     ////////////////////////////////////////////////////////////////////////////////
	     // Orientation
	     ////////////////////////////////////////////////////////////////////////////////
	     // Convienience accessors for commonly used trig functions. These values are generated
	     // by the DCM through a few simple equations. They are used throughout the code where cos and sin
	     // would normally be used.
	     // The cos values are defaulted to 1 to get a decent initial value for a level state
//	    float cos_roll_x ;
//	    float cos_pitch_x 	;
//	    float cos_yaw_x 		;
	    float sin_pitch_y;
	    //float sin_yaw_y,
	    float sin_roll_y;






	    struct Location wp_total_array[255];



	    ////////////////////////////////////////////////////////////////////////////////
	    // flight specific
	    ////////////////////////////////////////////////////////////////////////////////
	    // Flag for monitoring the status of flight
	    // We must be in the air with throttle for 5 seconds before this flag is true
	    // This flag is reset when we are in a manual throttle mode with 0 throttle or disarmed
	     //bool	takeoff_complete;
	    // Used to record the most recent time since we enaged the throttle to take off
	     int32_t	takeoff_timer;
	    // Used to see if we have landed and if we should shut our engines - not fully implemented
	     //bool	land_complete = true;
	    // used to manually override throttle in interactive Alt hold modes
	     int16_t 	manual_boost;
	    // An additional throttle added to keep the copter at the same altitude when banking
	   //  int16_t 	angle_boost;
	    // Push copter down for clean landing
	     int16_t 	landing_boost;
	    // for controlling the landing throttle curve
	    //verifies landings
	     int16_t ground_detector;

	     ////////////////////////////////////////////////////////////////////////////////
	     // Altitude
	     ////////////////////////////////////////////////////////////////////////////////
	     // The pressure at home location - calibrated at arming
	      int32_t 	ground_pressure;
	     // The ground temperature at home location - calibrated at arming
	      int16_t 	ground_temperature;
	     // The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
	      //int32_t		altitude_error;
	     // The cm/s we are moving up or down - Positive = UP
	  //    int16_t		climb_rate;
	     // The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
	      int16_t		sonar_alt;
	     // The climb_rate as reported by sonar in cm/s
	      int16_t		sonar_rate;
	     // The altitude as reported by Baro in cm – Values can be quite high
	     // int32_t		baro_alt;
	     // The climb_rate as reported by Baro in cm/s
	      int16_t		baro_rate;
	     //
	      bool 		reset_throttle_flag;

	      ////////////////////////////////////////////////////////////////////////////////
	      // Climb rate control
	      ////////////////////////////////////////////////////////////////////////////////
	      // Time when we intiated command in millis - used for controlling decent rate
	      // The orginal altitude used to base our new altitude during decent
	       int32_t 	original_altitude;
	      // Used to track the altitude offset for climbrate control
	       int32_t 	target_altitude;
	       uint32_t alt_change_timer;
	      // int8_t 	alt_change_flag;
	       uint32_t alt_change;

	      ////////////////////////////////////////////////////////////////////////////////
	      // Navigation Throttle control
	      ////////////////////////////////////////////////////////////////////////////////
	      // The Commanded Throttle from the autopilot.
	      // int16_t	nav_throttle;						// 0-1000 for throttle control
	      // This is a simple counter to track the amount of throttle used during flight
	      // This could be useful later in determining and debuging current usage and predicting battery life
	       ///uint32_t throttle_integrator;
	      // This is a future value for replacing the throttle_cruise setup procedure. It's an average of throttle control
	      // that is generated when the climb rate is within a certain threshold
	      // float	throttle_avg = THROTTLE_CRUISE;
	      // This is a flag used to trigger the updating of nav_throttle at 10hz
	       bool 	invalid_throttle;
	      // Used to track the altitude offset for climbrate control
	      // int32_t 	target_altitude;








    void compass_accumulate(void);
    void compass_cal_update(void);
    void barometer_accumulate(void);
    void perf_update(void);
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_mount();
    void update_trigger(void);
    void update_batt_compass(void);
    void fourhundred_hz_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();
   // bool home_is_set();
    void set_auto_armed(bool b);
    void set_simple_mode(uint8_t b);
    void set_failsafe_radio(bool b);
    void set_failsafe_battery(bool b);
    void set_failsafe_gcs(bool b);
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    void update_using_interlock();
    void set_motor_emergency_stop(bool b);
    float get_smoothing_gain();
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    void check_ekf_reset();
    float get_roi_yaw();
    float get_look_ahead_yaw();
    void update_throttle_hover();
    void set_throttle_takeoff();
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f);
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle();
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
    float get_avoidance_adjusted_climbrate(float target_rate);
    void auto_takeoff_set_start_alt(void);
    void auto_takeoff_attitude_run(float target_yaw_rate);
    void set_accel_throttle_I_from_pilot_throttle();
    void rotate_body_frame_to_NE(float &x, float &y);
    void gcs_send_heartbeat(void);
    void gcs_send_deferred(void);

    void rpm_update();
    void button_update();
    void init_proximity();
    void update_proximity();
    void stats_update();
    void init_beacon();
    void update_beacon();
    void init_visual_odom();
    void update_visual_odom();

    void gcs_send_mission_item_reached_message(uint16_t mission_index);
    void gcs_data_stream_send(void);
    void gcs_check_input(void);
    void do_erase_logs(void);
    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
    void Log_Write_Current();
    void Log_Write_Optflow();
    void Log_Write_Nav_Tuning();
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_EKF_POS();
    void Log_Write_MotBatt();
    void Log_Write_Event(uint8_t id);
    void Log_Write_Data(uint8_t id, int32_t value);
    void Log_Write_Data(uint8_t id, uint32_t value);
    void Log_Write_Data(uint8_t id, int16_t value);
    void Log_Write_Data(uint8_t id, uint16_t value);
    void Log_Write_Data(uint8_t id, float value);
    void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
    void Log_Write_Baro(void);
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high);
    void Log_Write_Home_And_Origin();
    void Log_Sensor_Health();
#if FRAME_CONFIG == HELI_FRAME
    void Log_Write_Heli(void);
#endif
    void Log_Write_Precland();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);

    void Log_Write_Proximity();
    void Log_Write_Beacon();
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
    void start_logging() ;
    void load_parameters(void);
    void convert_pid_parameters(void);
    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock);

    void set_system_time_from_GPS();
    void exit_mission();
    //void do_RTL(void);
   // bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
  //  bool verify_loiter_time();
    //bool verify_RTL();
    //bool verify_wait_delay();
   // bool verify_within_distance();
   // bool verify_yaw();
    void do_take_picture();
    void log_picture();

    void delay(uint32_t ms);
    bool acro_init(bool ignore_checks);
    void acro_run();
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);
    bool althold_init(bool ignore_checks);
    void althold_run();
    bool auto_init(bool ignore_checks);
    void auto_run();

    void auto_wp_run();
    void auto_spline_run();
    void auto_land_start();
    void auto_land_start(const Vector3f& destination);
    void auto_land_run();

    void auto_payload_place_start();
    void auto_payload_place_start(const Vector3f& destination);
    void auto_payload_place_run();
    bool auto_payload_place_run_should_run();
    void auto_payload_place_run_loiter();
    void auto_payload_place_run_descend();
    void auto_payload_place_run_release();
    void auto_rtl_start();
    void auto_rtl_run();

    void auto_circle_start();
    void auto_circle_run();
    void auto_nav_guided_start();
    void auto_nav_guided_run();
    bool auto_loiter_start();
    void auto_loiter_run();
    uint8_t get_default_auto_yaw_mode(bool rtl);
    void set_auto_yaw_mode(uint8_t yaw_mode);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    float get_auto_heading(void);
    bool autotune_init(bool ignore_checks);
    void autotune_stop();
    bool autotune_start(bool ignore_checks);
    void autotune_run();
    bool autotune_currently_level();
    void autotune_attitude_control();
    void autotune_backup_gains_and_initialise();
    void autotune_load_orig_gains();
    void autotune_load_tuned_gains();
    void autotune_load_intra_test_gains();
    void autotune_load_twitch_gains();
    void autotune_save_tuning_gains();
    void autotune_update_gcs(uint8_t message_id);
    bool autotune_roll_enabled();
    bool autotune_pitch_enabled();
    bool autotune_yaw_enabled();
    void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max);
    void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
    void autotune_get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);
    void avoidance_adsb_update(void);
    void autotune_send_step_string();
    const char *autotune_level_issue_string() const;
    const char * autotune_type_string() const;
    void autotune_announce_state_to_gcs();
    void autotune_do_gcs_announcements();

#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif
    bool brake_init(bool ignore_checks);
    void brake_run();
    void brake_timeout_to_loiter_ms(uint32_t timeout_ms);
    bool circle_init(bool ignore_checks);
    void circle_run();
    bool drift_init(bool ignore_checks);
    void drift_run();
    float get_throttle_assist(float velz, float pilot_throttle_scaled);
    bool flip_init(bool ignore_checks);
    void flip_run();
    bool guided_init(bool ignore_checks);
    bool guided_takeoff_start(float final_alt_above_home);
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();

    void guided_run();
    void guided_takeoff_run();
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);
    void guided_limit_clear();
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    void guided_limit_init_time_and_pos();
    bool guided_limit_check();
    bool guided_nogps_init(bool ignore_checks);
    void guided_nogps_run();
    bool land_init(bool ignore_checks);
    void land_run();
    void land_gps_run();
    void land_nogps_run();
    int32_t land_get_alt_above_ground(void);
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horizontal_control();
    void land_do_not_use_GPS();

    bool landing_with_GPS();
    bool loiter_init(bool ignore_checks);
    void loiter_run();
#if PRECISION_LANDING == ENABLED
    bool do_precision_loiter();
    void precision_loiter_xy();
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
    bool _precision_loiter_enabled;
#endif
    bool poshold_init(bool ignore_checks);
    void poshold_run();
    void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
    void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
    void poshold_update_wind_comp_estimate();
    void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
    void poshold_roll_controller_to_pilot_override();
    void poshold_pitch_controller_to_pilot_override();

    // Throw to launch functionality
    bool throw_init(bool ignore_checks);
    void throw_run();
    bool throw_detected();
    bool throw_attitude_good();
    bool throw_height_good();
    bool throw_position_good();

    bool rtl_init(bool ignore_checks);
    void rtl_restart_without_terrain();
    void rtl_run();
    void rtl_climb_start();
    void rtl_return_start();
    void rtl_climb_return_run();
    void rtl_loiterathome_start();
    void rtl_loiterathome_run();
    void rtl_descent_start();
    void rtl_descent_run();
    void rtl_land_start();
    void rtl_land_run();
    void rtl_build_path(bool terrain_following_allowed);
    void rtl_compute_return_target(bool terrain_following_allowed);
    bool sport_init(bool ignore_checks);
    void sport_run();
    bool stabilize_init(bool ignore_checks);
    void stabilize_run();
    void crash_check();
    void parachute_check();
    void parachute_release();
    void parachute_manual_release();

    // support for AP_Avoidance custom flight mode, AVOID_ADSB
    bool avoid_adsb_init(bool ignore_checks);
    void avoid_adsb_run();
    bool avoid_adsb_set_velocity(const Vector3f& velocity_neu);

    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    void esc_calibration_notify();
    bool should_disarm_on_failsafe();
    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void failsafe_battery_event(void);
    void failsafe_gcs_check();
    void failsafe_gcs_off_event(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void update_events();
    void failsafe_enable();
    void failsafe_disable();
    void fence_check();

    void heli_init();
    void check_dynamic_flight(void);
    void update_heli_control_dynamics(void);
    void heli_update_landing_swash();
    void heli_update_rotor_speed_targets();
    bool heli_acro_init(bool ignore_checks);
    void heli_acro_run();
    bool heli_stabilize_init(bool ignore_checks);
    void heli_stabilize_run();
    void read_inertia();
    bool land_complete_maybe();
    void update_land_and_crash_detectors();
    void update_land_detector();
    void update_throttle_thr_mix();
    void update_ground_effect_detector(void);
    void landinggear_update();
    void update_notify();
    void motor_test_output();

    void motor_test_stop();
    void arm_motors_check();
    void auto_disarm_check();
    bool init_arm_motors(bool arming_from_gcs);
    void init_disarm_motors();
    //void motors_output();
    void lost_vehicle_check();
    void run_nav_updates(void);
    void calc_distance_and_bearing();
    void calc_wp_distance();
    void calc_wp_bearing();
    void calc_home_distance_and_bearing();
    void run_autopilot();
    void perf_info_reset();
    void perf_ignore_this_loop();
    void perf_info_check_loop_time(uint32_t time_in_micros);
    uint16_t perf_info_get_num_loops();
    uint32_t perf_info_get_max_time();
    uint32_t perf_info_get_min_time();
    uint16_t perf_info_get_num_long_running();
    uint32_t perf_info_get_num_dropped();
    float pv_alt_above_origin(float alt_above_home_cm);
    float pv_alt_above_home(float alt_above_origin_cm);
    float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
    float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
    float pv_distance_to_home_cm(const Vector3f &destination);
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors();
    void init_barometer(bool full_calibration);
    //void read_barometer(void);
    int32_t read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok();
    void init_compass();
    void init_optflow();
    void update_optical_flow(void);
    void init_precland();
    void update_precland();
    void read_battery(void);
    void read_receiver_rssi(void);
    void epm_update();
    void gripper_update();
    void terrain_update();
    void terrain_logging();
    bool terrain_use();
    void report_batt_monitor();
    void report_frame();
    void report_radio();
    void report_ins();
    void report_flight_modes();
    void report_optflow();
    void print_radio_values();
    void print_switch(uint8_t p, uint8_t m, bool b);
    void print_accel_offsets_and_scaling(void);
    void print_gyro_offsets(void);
    void report_compass();
    void print_blanks(int16_t num);
    void print_divider(void);
    void print_enabled(bool b);
    void report_version();
    void read_control_switch();
    bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
    bool check_duplicate_auxsw(void);
    void reset_control_switch();
    uint8_t read_3pos_switch(uint8_t chan);
    void read_aux_switches();
    void init_aux_switches();
    void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
    void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
    void save_trim();
    void auto_trim();
    void init_ardupilot();
    void startup_INS_ground();
    bool calibrate_gyros();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    void update_auto_armed();
    void check_usb_mux(void);
    bool should_log(uint32_t mask);
    void set_default_frame_class();
    void allocate_motors(void);
    uint8_t get_frame_mav_type();
    const char* get_frame_string();
    bool current_mode_has_user_takeoff(bool must_navigate);
    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    void takeoff_timer_start(float alt_cm);
    void takeoff_stop();
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate);
    void print_hit_enter();
    void tuning();

    void init_capabilities(void);
    void dataflash_periodic(void);
    void accel_cal_update(void);

    /*
     * 上面这些函数都是私有的呀，别的类无法访问，只有自己能访问
     */

    void loop_fast();

    void set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input);

    void one_second_loop();

    void update_current_flight_mode(void);

    void update_navigation();

    void update_alt();

    void stabilize();

    // uses the yaw from the DCM to give more accurate turns
    void calc_bearing_error();



    // write out the servo PWM values
    // ------------------------------
    void set_servos_4();

    void init_led();
    void init_motor();
    void init_mpu6050();



    //get_yaw_rate_stabilized_ef(g.rc_4.control_in);
    void get_stabilize_roll(int32_t target_angle);

   void get_stabilize_pitch(int32_t target_angle);
   void get_stabilize_yaw(int32_t target_angle);
   void get_stabilize_rate_yaw(int32_t target_rate);
   void get_acro_roll(int32_t target_rate);
   void get_acro_pitch(int32_t target_rate);
   void get_acro_yaw(int32_t target_rate);
   // Roll with rate input and stabilized in the earth frame
    void get_roll_rate_stabilized_ef(int32_t stick_angle);

    // Pitch with rate input and stabilized in the earth frame
    void get_pitch_rate_stabilized_ef(int32_t stick_angle);

    // Yaw with rate input and stabilized in the earth frame
    void get_yaw_rate_stabilized_ef(int32_t stick_angle);

    // set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
    void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

    // set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
    void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

    // set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
    void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame );

    // update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
    void update_rate_contoller_targets();

    // run roll, pitch and yaw rate controllers and send output to motors
    // targets for these controllers comes from stabilize controllers
    void run_rate_controllers();

    int16_t get_rate_roll(int32_t target_rate);

    int16_t get_rate_pitch(int32_t target_rate);

    int16_t get_rate_yaw(int32_t target_rate);

    int16_t get_throttle_rate(int16_t z_target_speed);

    /*
     *  reset all I integrators
     */
    void reset_I_all(void);

    void reset_rate_I();

    void reset_throttle_I(void);

    void reset_stability_I(void);

    void update_yaw_mode(void);

    void update_roll_pitch_mode(void);


    void motors_output();
    void update_throttle_mode();

    int16_t get_angle_boost(int16_t value);

    void trim_radio();



    /*
     * 20170819为了自动驾驶增加的函数
     */

    void update_auto_yaw();
    void update_nav_wp();
    uint8_t check_missed_wp();
    void calc_XY_velocity();
    void calc_location_error(struct Location *next_loc);

    int16_t get_desired_speed(int16_t max_speed, bool _slow);
    int16_t get_desired_climb_rate();
    void calc_nav_rate(int16_t max_speed);
    void update_crosstrack(void);




    /*
     * commands.cpp
     */
    void init_commands();
    struct Location get_cmd_with_index(int i);
    void set_cmd_with_index(struct Location temp, int i);
    int32_t read_alt_to_hold();
    void set_next_WP(struct Location *wp);
    void init_home();

    /*
     * commands_logic.cpp
     */
	void  process_nav_command();
	void  process_cond_command();
	void  process_now_command();
	bool verify_must();
	bool verify_may();
	void  do_RTL(void);
	void  do_takeoff();
	void  do_nav_wp();
	void  do_land();
	void  do_loiter_unlimited();
	void  do_loiter_turns();
	void  do_loiter_time();
	bool verify_takeoff();
	bool verify_land_sonar();
	bool verify_land_baro();
	bool verify_nav_wp();
	bool verify_loiter_time();
	bool verify_loiter_turns();
	bool verify_RTL();
	void  do_wait_delay();

	/********************************************************************************/
	//	Condition (May) commands
	/********************************************************************************/
	//void  do_wait_delay();
	void  do_change_alt();
	void  do_within_distance();
	void  do_yaw();

	/********************************************************************************/
	// Verify Condition (May) commands
	/********************************************************************************/

	bool  verify_wait_delay();
	bool  verify_change_alt();
	bool  verify_within_distance();
	bool  verify_yaw();

	/********************************************************************************/
	//	Do (Now) commands
	/********************************************************************************/

	void  do_change_speed();
	void  do_target_yaw();
	void  do_loiter_at_location();
	void  do_jump();
	void  do_set_home();
	void  do_set_servo();
	void  do_set_relay();

	void  do_repeat_servo();
	void  do_repeat_relay();




    /*
     * commands_process.cpp
     */

	// For changing active command mid-mission
	//----------------------------------------
	 void change_command(uint8_t cmd_index);
	// called by 10 Hz loop
	// --------------------
	 void update_commands();
	 void execute_nav_command(void);

	// called with GPS navigation update - not constantly
	 void verify_commands(void);
	 void calc_loiter_pitch_roll();
	 void update_trig(void);

	 void reset_nav_params(void);

	 void adjust_altitude();

	 void update_throttle_cruise();
	 void clear_new_altitude();
	 void set_new_altitude(int32_t _new_alt);

	 int32_t get_altitude_error();
	 int16_t get_nav_throttle(int32_t z_error);
	 int get_z_damping();
	 int32_t get_new_altitude();

};


extern const AP_HAL::HAL& hal;
extern Copter copter;




#endif /* COPTER_H_ */
