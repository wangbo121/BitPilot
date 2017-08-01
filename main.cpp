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

#include "pid.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"


#include "global.h"

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

int main()
{

	cout<<"Welcome to BitPilot"<<endl;

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
