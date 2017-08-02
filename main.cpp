/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

Copter copter;

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
	//init_bitpilot();
	copter.setup();

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

		copter.loop();

		//copter.loop_fast();

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

			//copter.compass.read();     // Read magnetometer
			//copter.compass.calculate(dcm.get_dcm_matrix());  // Calculate heading
			//copter.compass.null_offsets(dcm.get_dcm_matrix());


			// calculate the plane's desired bearing
			// -------------------------------------
			//copter.navigate();

			// Read altitude from sensors
			// ------------------
			//update_alt();

		}

	}

	return 0;
}

int init_pilot()
{
	copter.setup();
	return 0;
}

void Copter::loop_fast()
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


	/*
	 * 这个dcm的初始化构造函数有问题，得再改改
	 */
	//dcm.update_DCM(G_Dt);

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

void Copter::read_radio()
{

}

#if 0
//不要删除这个，还有参考价值
int main()
{
	cout<<"Welcome to BitPilot"<<endl;

	/*
	 * 初始化工作
	 */
	//init_bitpilot();
	copter.setup();

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

		copter.loop();

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

			//copter.compass.read();     // Read magnetometer
			//copter.compass.calculate(dcm.get_dcm_matrix());  // Calculate heading
			//copter.compass.null_offsets(dcm.get_dcm_matrix());


			// calculate the plane's desired bearing
			// -------------------------------------
			//copter.navigate();

			// Read altitude from sensors
			// ------------------
			//update_alt();

		}

	}

	return 0;
}

#endif
