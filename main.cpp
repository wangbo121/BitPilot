/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

Copter copter;

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

	/*
	 * 初始化工作
	 */
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
	}

	return 0;
}

/*
 * wangbo20170802
 * 其实下面这些Copter类的函数的定义是可以放在copter.cpp中的
 * 但是对于看代码的人来说，可以直接从这里看到函数定义，就不用跳到copter.cpp中去了
 */
void Copter::setup()
{
	init_led();
	init_motor();
	init_mpu6050();
}

void Copter::loop()
{
	loop_fast();
}

void Copter::loop_fast()
{
	/*
	 * wangbo20170801
	 * 其实如果只是增稳控制的话
	 * 只需要下面5步骤就可以了
	 * 其他的都是用来与地面站通信然后实现自动驾驶的，比如气压计，空速计，gps，导航，航点等
	 * 1--read_radio
	 * 2--update_DCM
	 * 3--update_current_flight_mode
	 * 4--stabilize
	 * 5--set_servos
	 */

	/* 1--读取接收机的信号，获取遥控器各个通道 */
	read_radio();

	/* 2--更新姿态，获取飞机现在的姿态角 */
	//dcm.update_DCM(G_Dt);

    // custom code/exceptions for flight modes
    // ---------------------------------------
	/*
	 * 这两个函数是从rc读取进来的pwm值，将其转换为目标角度roll_target等
	 * 角度的pid，第一级pid
	 */
    //update_yaw_mode();
    //update_roll_pitch_mode();

    // update targets to rate controllers
    /*
     * 这个是第二级pid，把上面的结果导入到这里
     */
    update_rate_contoller_targets();

	/* 3--update_current_flight_mode 更新控制状态，从而选择控制方式 */
	update_current_flight_mode();

	/* 4--把期望的roll pitch yaw作用于飞机 */
	switch(control_mode)
	{
	case MANUAL:
		break;
	case STABILIZE:
		stabilize();
		break;
	default:
		stabilize();
		break;
	}
	// run low level rate controllers that only require IMU data
	//attitude_control->rate_controller_run();

	/* 5--把计算所得控制量输出给电机 */
	set_servos_4();
	// send outputs to the motors library immediately
	//motors_output();
}

void Copter::read_radio()
{
	g.channel_roll.set_pwm(ap_rc.input_ch(CH_1));
	g.channel_pitch.set_pwm(ap_rc.input_ch(CH_2));
	g.channel_throttle.set_pwm(ap_rc.input_ch(CH_3));
	g.channel_rudder.set_pwm(ap_rc.input_ch(CH_4));
	g.rc_5.set_pwm(ap_rc.input_ch(CH_5));
	g.rc_6.set_pwm(ap_rc.input_ch(CH_6));
	g.rc_7.set_pwm(ap_rc.input_ch(CH_7));
	g.rc_8.set_pwm(ap_rc.input_ch(CH_8));

}

void Copter::update_current_flight_mode(void)
{

}

void Copter::stabilize()
{

}

// write out the servo PWM values
// ------------------------------
void Copter::set_servos_4()
{

}

void Copter::init_led()
{

}
void Copter::init_motor()
{

}
void Copter::init_mpu6050()
{

}









void Copter::one_second_loop()
{
	/*
	 * 一秒钟给地面站发送一组数据--实时数据
	 */
}


void Copter::update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

}

void Copter::update_alt()
{

}

void Copter::update_GPS(void)
{

}

// uses the yaw from the DCM to give more accurate turns
void Copter::calc_bearing_error()
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
