/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

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

// Libraries
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "utility.h"

#include "gps.h"        // ArduPilot GPS library
#include "compass.h"     // ArduPilot Mega Magnetometer Library
#include "compass_hmc5843.h"
#include "BIT_MATH.h"        // ArduPilot Mega Vector/Matrix math Library
#include "imu.h"         // ArduPilot Mega IMU Library
#include "imu_oilpan.h"
#include "ahrs_DCM.h"         // ArduPilot Mega DCM Library
#include "pid.h"            // PID library
#include "rc.h"         // ArduPilot Mega RC Library
#include "rc_channel.h"     // RC Channel Library

// Local modules
#include "defines.h"
#include "global.h"
//#include "GCS.h"




////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
Global_Pilot      g;


// All GPS access should be through this pointer.
GPS         *g_gps;

/*
 * 下面这些都是真实传感器的实例化
 */
// real sensors
//AP_ADC_ADS7844          adc;
//APM_BMP085_Class        barometer;
//AP_Compass_HMC5843      compass();

//AP_DCM              dcm;
AP_IMU_Oilpan *imu;

AP_DCM dcm(imu,g_gps);

AP_Compass_HMC5843 compass;

/*
 * 这里得读取gps数据
 */
// real GPS selection


//AP_RangeFinder_MaxsonarXL sonar;

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

int8_t    control_mode        = MANUAL;
int8_t    oldSwitchPosition;              // for remembering the control mode switch

const char *comma = ",";

const char* flight_mode_strings[] = {
	"Manual",
	"Circle",
	"Stabilize",
	"",
	"",
	"FBW_A",
	"FBW_B",
	"",
	"",
	"",
	"Auto",
	"RTL",
	"Loiter",
	"Takeoff",
	"Land"};


/* Radio values
		Channel assignments
			1   Ailerons (rudder if no ailerons)
			2   Elevator
			3   Throttle
			4   Rudder (if we have ailerons)
			5   Mode
			6   TBD
			7   TBD
			8   TBD
*/



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
long	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
long	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
int		climb_rate;							// m/s * 100  - For future implementation of controlled ascent/descent by rate
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
long 	loiter_time;						// millis : when we started LOITER mode
int 	loiter_time_max;					// millis : how long to stay in LOITER mode

// these are the values for navigation control functions
// ----------------------------------------------------
long	nav_roll;							// deg * 100 : target roll angle
long	nav_pitch;							// deg * 100 : target pitch angle
int     throttle_nudge = 0;                 // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel

// Waypoints
// ---------
long	wp_distance;						// meters - distance between plane and next waypoint
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
long 	condition_value;						// used in condition commands (eg delay, change alt, etc.)
long 	condition_start;
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
float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)


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
unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
unsigned long 	fast_loopTimeStamp;			// Time Stamp when fast loop was complete
uint8_t 		delta_ms_fast_loop; 		// Delta Time in miliseconds
int 			mainLoop_count;

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





// Read radio
// ----------
void read_radio();


void read_airspeed();




// uses the yaw from the DCM to give more accurate turns
void calc_bearing_error();

void navigate();
void update_alt();



// custom code/exceptions for flight modes
// ---------------------------------------
void update_current_flight_mode();

void stabilize();

// write out the servo PWM values
// ------------------------------
void set_servos_4();

/******************************************************/
/*****************/
/*
 * 初始化驾驶仪
 */
int init_pilot();
int loop_fast();
int loop_medium();
int loop_slow();

/******************************************************/
/*****************/
#define MAINTASK_TICK_TIME_MS 20
struct tm *global_time_val;//全局时间变量，其它的时间都从这里取
time_t timep;

int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

int maintask_cnt;

#define ONE_HZ_CNT  50//1hz
#define TEN_HZ_CNT   5//10hz
#define FIFTY_HZ_CNT 1//50hz



void init_led();
void init_motor();
void init_mpu6050();
int init_bitpilot()
{
	init_led();
	init_motor();
	init_mpu6050();


	return 0;
}
int main()
{
	cout<<"Welcome to BitPilot"<<endl;

	/*
	 * 初始化工作
	 */
	init_bitpilot();

	/*
	 * 这个while循环是20ms
	 * 也就是50hz
	 */
	while (1)
	{
		maintask_tick.tv_sec = seconds;
		maintask_tick.tv_usec = mseconds;
		select(0, NULL, NULL, NULL, &maintask_tick);
		maintask_cnt++;

		loop_fast();

		if(0 == maintask_cnt % ONE_HZ_CNT)
		{
			/*
			 * 1hz的循环
			 */

			printf("hello \n");
		}

		if(0 == maintask_cnt % TEN_HZ_CNT)
		{
			/*
			 * 10hz的循环
			 */

			compass.read();     // Read magnetometer
			compass.calculate(dcm.get_dcm_matrix());  // Calculate heading
			compass.null_offsets(dcm.get_dcm_matrix());


			// calculate the plane's desired bearing
			// -------------------------------------
			navigate();

			// Read altitude from sensors
			// ------------------
			update_alt();

		}

	}

	return 0;
}

int init_pilot()
{
	return 0;
}

int loop_fast()
{

	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (delta_ms_fast_loop > G_Dt_max)
		G_Dt_max = delta_ms_fast_loop;

	/*
	 * wangbo20170801
	 * 其实如果只是增稳控制的话
	 * 只需要下面5步骤就可以了，其他都是用来与地面站通信然后实现自动驾驶的，比如气压计，空速计，gps，导航，航点等
	 * 1--read_radio
	 * 2--update_DCM
	 * 3--update_current_flight_mode
	 * 4--stabilize
	 * 5--set_servos
	 */
	// Read radio
	// ----------
	read_radio();

	dcm.update_DCM(G_Dt);

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_current_flight_mode();

	// apply desired roll, pitch and yaw to the plane
	// ----------------------------------------------
	if (control_mode > MANUAL)
		stabilize();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();

	// uses the yaw from the DCM to give more accurate turns
	calc_bearing_error();

	read_airspeed();

	return 0;
}

int loop_medium()
{
	return 0;
}

int loop_slow()
{
	return 0;
}

void one_second_loop()
{
	/*
	 * 一秒钟给地面站发送一组数据--实时数据
	 */
}

void update_GPS(void)
{


}

void update_current_flight_mode(void)
{

}

void update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

}


void update_alt()
{

}

void stabilize()
{

}

// uses the yaw from the DCM to give more accurate turns
void calc_bearing_error()
{

}


// write out the servo PWM values
// ------------------------------
void set_servos_4()
{

}
void navigate()
{

}

void init_led()
{

}
void init_motor()
{

}
void init_mpu6050()
{

}

#if 0

BIT_PID pid;
	pid.set_kP(2);
	float pid_result = pid.get_pid(2,1000,1);
	cout<<"pid_result="<<pid_result<<endl;


	Vector2i vector2_1(1,2);
	Vector2i vector2_2(3,4);
	Vector2i vector2_result;

	vector2_result=vector2_1+vector2_2;

	cout<<"vector2_result.x="<<vector2_result.x<<endl;
	cout<<"vector2_result.y="<<vector2_result.y<<endl;

	Vector3i vector3_1(1,2,3);
	Vector3i vector3_2(3,4,5);
	Vector3i vector3_result;

	vector3_result=vector3_1+vector3_2;

	cout<<"vector3_result.x="<<vector3_result.x<<endl;
	cout<<"vector3_result.y="<<vector3_result.y<<endl;
	cout<<"vector3_result.z="<<vector3_result.z<<endl;

	Matrix3i m3(1,2,3,4,5,6,7,8,9);

	cout<<"m3.12="<<m3.a.y<<endl;

	Global_Pilot g;

	init_pilot();

	loop_fast();

	loop_medium();

	loop_slow();
#endif



/******************************************/
#if 0

void stabilize()
{
	static int8_t temp = 0;
	float ch1_inf = 1.0;
	float ch2_inf = 1.0;
	float ch4_inf = 1.0;
	float speed_scaler;

	if (airspeed_enabled == true){
		if(airspeed > 0)
			speed_scaler = (STANDARD_SPEED * 100) / airspeed;
		else
			speed_scaler = 2.0;
			speed_scaler = constrain(speed_scaler, 0.5, 2.0);
	} else {
		if (g.channel_throttle.servo_out > 0){
			speed_scaler = 0.5 + (THROTTLE_CRUISE / g.channel_throttle.servo_out / 2.0);	// First order taylor expansion of square root
																				// Should maybe be to the 2/7 power, but we aren't goint to implement that...
		}else{
			speed_scaler = 1.67;
		}
		speed_scaler = constrain(speed_scaler, 0.6, 1.67);		// This case is constrained tighter as we don't have real speed info
	}

	if(crash_timer > 0){
		nav_roll = 0;
	}

	// For Testing Only
	// roll_sensor = (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 10;
	// Serial.printf_P(PSTR(" roll_sensor "));
	// Serial.print(roll_sensor,DEC);

	// Calculate dersired servo output for the roll
	// ---------------------------------------------
	g.channel_roll.servo_out = g.pidServoRoll.get_pid((nav_roll - dcm.roll_sensor), delta_ms_fast_loop, speed_scaler);
	long tempcalc = nav_pitch +
	        fabs(dcm.roll_sensor * g.kff_pitch_compensation) +
	        (g.channel_throttle.servo_out * g.kff_throttle_to_pitch) -
	        (dcm.pitch_sensor - g.pitch_trim);
	g.channel_pitch.servo_out = g.pidServoPitch.get_pid(tempcalc, delta_ms_fast_loop, speed_scaler);

	// Mix Stick input to allow users to override control surfaces
	// -----------------------------------------------------------
	if ((control_mode < FLY_BY_WIRE_A) || (ENABLE_STICK_MIXING == 1 && control_mode > FLY_BY_WIRE_B)) {


		// TODO: use RC_Channel control_mix function?
		ch1_inf = (float)g.channel_roll.radio_in - (float)g.channel_roll.radio_trim;
		ch1_inf = fabs(ch1_inf);
		ch1_inf = min(ch1_inf, 400.0);
		ch1_inf = ((400.0 - ch1_inf) /400.0);

		ch2_inf = (float)g.channel_pitch.radio_in - g.channel_pitch.radio_trim;
		ch2_inf = fabs(ch2_inf);
		ch2_inf = min(ch2_inf, 400.0);
		ch2_inf = ((400.0 - ch2_inf) /400.0);

		// scale the sensor input based on the stick input
		// -----------------------------------------------
		g.channel_roll.servo_out *= ch1_inf;
		g.channel_pitch.servo_out *= ch2_inf;

		// Mix in stick inputs
		// -------------------
		g.channel_roll.servo_out +=	g.channel_roll.pwm_to_angle();
		g.channel_pitch.servo_out +=	g.channel_pitch.pwm_to_angle();

		//Serial.printf_P(PSTR(" servo_out[CH_ROLL] "));
		//Serial.println(servo_out[CH_ROLL],DEC);
	}

	// stick mixing performed for rudder for all cases including FBW unless disabled for higher modes
	// important for steering on the ground during landing
	// -----------------------------------------------
	if (control_mode <= FLY_BY_WIRE_B || ENABLE_STICK_MIXING == 1) {
		ch4_inf = (float)g.channel_rudder.radio_in - (float)g.channel_rudder.radio_trim;
		ch4_inf = fabs(ch4_inf);
		ch4_inf = min(ch4_inf, 400.0);
		ch4_inf = ((400.0 - ch4_inf) /400.0);
	}

	// Apply output to Rudder
	// ----------------------
	calc_nav_yaw(speed_scaler);
	g.channel_rudder.servo_out *= ch4_inf;
	g.channel_rudder.servo_out += g.channel_rudder.pwm_to_angle();

	// Call slew rate limiter if used
	// ------------------------------
	//#if(ROLL_SLEW_LIMIT != 0)
	//	g.channel_roll.servo_out = roll_slew_limit(g.channel_roll.servo_out);
	//#endif
}

void crash_checker()
{
	if(dcm.pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}


void calc_throttle()
{
  if (airspeed_enabled == false) {
	int throttle_target = g.throttle_cruise + throttle_nudge;

		// no airspeed sensor, we use nav pitch to determine the proper throttle output
		// AUTO, RTL, etc
		// ---------------------------------------------------------------------------
		if (nav_pitch >= 0) {
			g.channel_throttle.servo_out = throttle_target + (g.throttle_max - throttle_target) * nav_pitch / g.pitch_limit_max;
		} else {
			g.channel_throttle.servo_out = throttle_target - (throttle_target - g.throttle_min) * nav_pitch / g.pitch_limit_min;
		}

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
	} else {
		// throttle control with airspeed compensation
		// -------------------------------------------
		energy_error = airspeed_energy_error + (float)altitude_error * 0.098f;

		// positive energy errors make the throttle go higher
		g.channel_throttle.servo_out = g.throttle_cruise + g.pidTeThrottle.get_pid(energy_error, dTnav);
		g.channel_throttle.servo_out += (g.channel_pitch.servo_out * g.kff_pitch_to_throttle);

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out,
			g.throttle_min.get(), g.throttle_max.get());			// TODO - resolve why "saved" is used here versus "current"
	}

}

/*****************************************
 * Calculate desired roll/pitch/yaw angles (in medium freq loop)
 *****************************************/

//  Yaw is separated into a function for future implementation of heading hold on rolling take-off
// ----------------------------------------------------------------------------------------
void calc_nav_yaw(float speed_scaler)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
	Vector3f temp = imu.get_accel();
	long error = -temp.y;

	// Control is a feedforward from the aileron control + a PID to coordinate the turn (drive y axis accel to zero)
	g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out + g.pidServoRudder.get_pid(error, delta_ms_fast_loop, speed_scaler);
#else
	g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out;
	// XXX probably need something here based on heading
#endif
}


void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	if (airspeed_enabled == true) {
		nav_pitch = -g.pidNavPitchAirspeed.get_pid(airspeed_error, dTnav);
	} else {
		nav_pitch = g.pidNavPitchAltitude.get_pid(altitude_error, dTnav);
    }
	nav_pitch = constrain(nav_pitch, g.pitch_limit_min.get(), g.pitch_limit_max.get());
}


#define YAW_DAMPENER 0

void calc_nav_roll()
{

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed
	nav_gain_scaler = (float)g_gps->ground_speed / (STANDARD_SPEED * 100.0);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// negative error = left turn
	// positive error = right turn
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = g.pidNavRoll.get_pid(bearing_error, dTnav, nav_gain_scaler);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll, -g.roll_limit.get(), g.roll_limit.get());

	Vector3f omega;
	omega = dcm.get_gyro();

	// rate limiter
	long rate		= degrees(omega.z) * 100; 										// 3rad = 17188 , 6rad = 34377
	rate			= constrain(rate, -6000, 6000);									// limit input
	int dampener 	= rate * YAW_DAMPENER;											// 34377 * .175 = 6000

	// add in yaw dampener
	nav_roll		-= dampener;
	nav_roll		= constrain(nav_roll, -g.roll_limit.get(), g.roll_limit.get());
}


/*****************************************
 * Roll servo slew limit
 *****************************************/
/*
float roll_slew_limit(float servo)
{
	static float last;
	float temp = constrain(servo, last-ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f, last + ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f);
	last = servo;
	return temp;
}*/

/*****************************************
 * Throttle slew limit
 *****************************************/
/*float throttle_slew_limit(float throttle)
{
	static float last;
	float temp = constrain(throttle, last-THROTTLE_SLEW_LIMIT * delta_ms_fast_loop/1000.f, last + THROTTLE_SLEW_LIMIT * delta_ms_fast_loop/1000.f);
	last = throttle;
	return temp;
}
*/

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
	g.pidNavRoll.reset_I();
	g.pidNavPitchAirspeed.reset_I();
	g.pidNavPitchAltitude.reset_I();
	g.pidTeThrottle.reset_I();
//	g.pidAltitudeThrottle.reset_I();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
void set_servos_4(void)
{
	if(control_mode == MANUAL){
		// do a direct pass through of radio values
		if (mix_mode == 0){
			g.channel_roll.radio_out 		= g.channel_roll.radio_in;
			g.channel_pitch.radio_out 		= g.channel_pitch.radio_in;
		} else {
			g.channel_roll.radio_out 		= APM_RC.InputCh(CH_ROLL);
			g.channel_pitch.radio_out 		= APM_RC.InputCh(CH_PITCH);
		}
		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_out 		= g.channel_rudder.radio_in;

	} else {
		if (mix_mode == 0){
			g.channel_roll.calc_pwm();
			g.channel_pitch.calc_pwm();
			g.channel_rudder.calc_pwm();

		}else{
			/*Elevon mode*/
			float ch1;
			float ch2;
			ch1 = reverse_elevons * (g.channel_pitch.servo_out - g.channel_roll.servo_out);
			ch2 = g.channel_pitch.servo_out + g.channel_roll.servo_out;
			g.channel_roll.radio_out =	elevon1_trim + (reverse_ch1_elevon * (ch1 * 500.0/ ROLL_SERVO_MAX));
			g.channel_pitch.radio_out =	elevon2_trim + (reverse_ch2_elevon * (ch2 * 500.0/ PITCH_SERVO_MAX));
		}

		#if THROTTLE_OUT == 0
			g.channel_throttle.servo_out = 0;
		#else
			// convert 0 to 100% into PWM
			g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
		#endif

		g.channel_throttle.calc_pwm();

		//Radio_in: 1763	PWM output: 2000	Throttle: 78.7695999145	PWM Min: 1110	PWM Max: 1938

		/*  TO DO - fix this for RC_Channel library
		#if THROTTLE_REVERSE == 1
			radio_out[CH_THROTTLE] = radio_max(CH_THROTTLE) + radio_min(CH_THROTTLE) - radio_out[CH_THROTTLE];
		#endif
		*/
	}


	// send values to the PWM timers for output
	// ----------------------------------------
	APM_RC.OutputCh(CH_1, g.channel_roll.radio_out); // send to Servos
	APM_RC.OutputCh(CH_2, g.channel_pitch.radio_out); // send to Servos
	APM_RC.OutputCh(CH_3, g.channel_throttle.radio_out); // send to Servos
	APM_RC.OutputCh(CH_4, g.channel_rudder.radio_out); // send to Servos
 }

int readOutputCh(unsigned char ch)
{
 int pwm;
 switch(ch)
	{

 	 	 /*
 	 	  * 这里的pwm应该赋值，赋的值是从接收机读取的数
 	 	  */
		case 0:	pwm = 0; break;	// ch0
		case 1:	pwm = 0; break;	// ch1
		case 2:	pwm = 0; break;	// ch2
		case 3:	pwm = 0; break;	// ch3
		case 4:	pwm = 0; break;	// ch4
		case 5:	pwm = 0; break;	// ch5
		case 6:	pwm = 0; break;	// ch6
		case 7:	pwm = 0; break;	// ch7
		case 8:	pwm = 0; break;	// ch8,	PL3
		case 9:	pwm = 0; break;	// ch9,	PB5
		case 10: pwm = 0; break;	// ch10, PE3
	}
	pwm >>= 1;	 // pwm / 2;
	return pwm;
}
#endif
