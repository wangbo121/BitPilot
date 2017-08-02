/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

Copter copter;

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
			copter.navigate();

			// Read altitude from sensors
			// ------------------
			update_alt();

		}

	}

	return 0;
}

int init_pilot()
{
	copter.setup();
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


void init_led()
{

}
void init_motor()
{

}
void init_mpu6050()
{

}

