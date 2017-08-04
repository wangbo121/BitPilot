/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

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

int main(int argc,char * const argv[])
{
	cout<<"Welcome to BitPilot"<<endl;

	hal.run(argc,argv,&copter);

	return 0;

	/*
	 * hal的run函数就是调用copter，
	 * 然后执行copter的setup和loop函数，
	 * 跟直接写在下面是一样的
	 */
#if 0
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
#endif
}

/*
 * wangbo20170802
 * 其实下面这些Copter类的函数的定义是可以放在copter.cpp中的
 * 但是对于看代码的人来说，可以直接从这里看到函数定义，就不用跳到copter.cpp中去了
 */
void Copter::setup()
{
	//init_led();
	//init_motor();
	//init_mpu6050();

	/*
	 * 下面这些初始化，其实应该放在跟地面站连接时
	 * 地面站的setup按钮里，设置遥控器的最大最小值
	 * 但是我这里先直接赋值
	 */

	/*
	 * 1RC_Channel对象的内部变量的初始化
	 * 这个在angle_to_pwm,pwm_to_angle等中都有用
	 */
	g.channel_roll.radio_min = 1000;
	g.channel_pitch.radio_min = 1000;
	g.channel_throttle.radio_min = 1000;
	g.channel_rudder.radio_min = 1000;
	g.rc_5.radio_min = 1000;
	g.rc_6.radio_min = 1000;
	g.rc_7.radio_min = 1000;
	g.rc_8.radio_min = 1000;

	g.channel_roll.radio_max = 2000;
	g.channel_pitch.radio_max = 2000;
	g.channel_throttle.radio_max = 2000;
	g.channel_rudder.radio_max = 2000;
	g.rc_5.radio_max = 2000;
	g.rc_6.radio_max = 2000;
	g.rc_7.radio_max = 2000;
	g.rc_8.radio_max = 2000;

	g.channel_roll.radio_trim = 1500;
	g.channel_pitch.radio_trim = 1500;
	g.channel_throttle.radio_trim = 1500;
	g.channel_rudder.radio_trim = 1500;
	// 3 is not trimed  这里arducopter中注释说，throttle是没有trim的，但是没有的话，我的运行就有错误，到底是有没有呢
	g.rc_5.radio_trim = 1500;
	g.rc_6.radio_trim = 1500;
	g.rc_7.radio_trim = 1500;
	g.rc_8.radio_trim = 1500;

	g.channel_roll.set_angle(4500);//这个是设置舵机能单侧摆动的最大值，45度
	g.channel_pitch.set_angle(4500);
	g.channel_throttle.set_angle(4500);
	g.channel_rudder.set_angle(4500);
	g.rc_5.set_angle(4500);

	g.channel_roll.dead_zone=0;
	g.channel_pitch.dead_zone=0;
	g.channel_rudder.dead_zone=0;
	g.channel_throttle.dead_zone=0;
	g.rc_5.dead_zone=0;

	g.channel_roll.set_reverse(0);//不取反
	g.channel_pitch.set_reverse(0);
	g.channel_rudder.set_reverse(0);
	g.channel_throttle.set_reverse(0);
	g.rc_5.set_reverse(0);
}

void Copter::loop()
{
#ifdef TEST
	std::cout<<"hello wangbo loop"<<std::endl;
	sleep(1);
#else
	loop_fast();
#endif
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
	 * 4--control根据飞行模式 control_mode的选项，选择不同的控制方式
	 * 5--set_servos
	 */

	std::cout<<"Hello loopfast"<<std::endl;
	sleep(1);

	/* 1--读取接收机的信号，获取遥控器各个通道 */
	read_radio();

	/* 2--更新姿态，获取飞机现在的姿态角 */
	//ahrs->update_DCM(G_Dt);

	/* 3--update_current_flight_mode 更新控制状态，从而选择控制方式 */
	update_current_flight_mode();

	/* 4--把期望的roll pitch yaw作用于飞机 */
	switch(control_mode)
	{
	case STABILIZE:
		std::cout<<"Hello STABILIZE MODE"<<std::endl;
		/*
		* 先是roll pitch yaw的2级pid控制
		* 再是油门throttle的2级pid控制
		* 都是只是计算得出g.channel.servo_out的值
		* 在motors_output时再把这些计算的值真正输出
		*/
		update_roll_pitch_mode();
		update_yaw_mode();
		run_rate_controllers();

		update_throttle_mode();//计算油门量的输出值
		break;
	case ACRO:
		std::cout<<"Hello ACRO MODE"<<std::endl;
		// call rate controllers
		g.channel_roll.servo_out = g.channel_roll.control_in;
		g.channel_pitch.servo_out = g.channel_pitch.control_in;
		g.channel_rudder.servo_out = g.channel_rudder.control_in;

		g.channel_throttle.servo_out=g.channel_throttle.control_in;
		break;

	default:
		break;
	}

	/* 5--把计算所得控制量输出给电机 */
	motors_output();

}

void Copter::read_radio()
{
	/*
	 * //set_pwm做两件事，
	 * 1是给radion_in赋值，作为从rc_channel读取回来的数
	 * 2是把control_in = pwm_to_angle(radio)，也就是把读取回来的pwm转为-4500～+4500角度控制值
	 */
	g.channel_roll.set_pwm(ap_rc.input_ch(CH_1));
	g.channel_pitch.set_pwm(ap_rc.input_ch(CH_2));
	g.channel_throttle.set_pwm(ap_rc.input_ch(CH_3));
	g.channel_rudder.set_pwm(ap_rc.input_ch(CH_4));
	g.rc_5.set_pwm(ap_rc.input_ch(CH_5));
}

void Copter::update_current_flight_mode(void)
{
	if(g.rc_5.radio_in>1000&&g.rc_5.radio_in<1500)
	{
		control_mode=ACRO;
	}
	else if(g.rc_5.radio_in>1500)
	{
		control_mode=STABILIZE;
	}
}

void Copter::stabilize()
{

}

// write out the servo PWM values
// ------------------------------
void Copter::set_servos_4()
{

}
void Copter::motors_output()
{
	int16_t             motor_out[BIT_MOTORS_MAX_NUM_MOTORS];

    int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    float               _roll_factor[BIT_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[BIT_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[BIT_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)

    /*
     * 这里给factor赋值-1 0 或者1
     */
    _roll_factor[0]  =  -1;  _pitch_factor[0]  = +1;  _yaw_factor[0]  = +1;
    _roll_factor[1]  =  -1;  _pitch_factor[1]  =  -1;  _yaw_factor[1]  =  -1;
    _roll_factor[2]  = +1;  _pitch_factor[2]  =  -1;  _yaw_factor[2]  = +1;
    _roll_factor[3]  = +1;  _pitch_factor[3]  = +1;  _yaw_factor[3]  =  -1;

	g.channel_roll.calc_pwm();
	g.channel_pitch.calc_pwm();
	g.channel_rudder.calc_pwm();
	g.channel_throttle.calc_pwm();

	std::cout<<"g.channel_roll.servo_out="<<g.channel_roll.servo_out<<std::endl;
	std::cout<<"g.channel_pitch.servo_out="<<g.channel_pitch.servo_out<<std::endl;
	std::cout<<"g.channel_rudder.servo_out="<<g.channel_rudder.servo_out<<std::endl;
	std::cout<<"g.channel_throttle.servo_out="<<g.channel_throttle.servo_out<<std::endl;

	std::cout<<"********准备输出pwm脉宽给电机***********"<<std::endl;

	std::cout<<"g.channel_roll.pwm_out="<<g.channel_roll.pwm_out<<std::endl;
	std::cout<<"g.channel_pitch.pwm_out="<<g.channel_pitch.pwm_out<<std::endl;
	std::cout<<"g.channel_rudder.pwm_out="<<g.channel_rudder.pwm_out<<std::endl;

	std::cout<<"g.channel_throttle.radio_out="<<g.channel_throttle.radio_out<<std::endl;


	for(int i=0;i<4;i++)
	{

		/*
		 * 一定要注意这里的throttle是用的radio_out，radio_out=pwm_out+radio_trim，
		 * calc是把servo_out的-4500～+4500转为pwm的-500～+500
		 * radio_out=pwm_out+radio_trim=pwm_out+1500 radio_out的范围是1000-2000
		 */
		motor_out[i]=g.channel_throttle.radio_out+ \
				                 g.channel_roll.pwm_out*_roll_factor[i]+\
				                 g.channel_pitch.pwm_out* _pitch_factor[i]+\
				                 g.channel_rudder.pwm_out * _yaw_factor[i];
	}
	for(int i=0;i<4;i++)
	{
		g._rc.output_ch_pwm(i,motor_out[i]);
		//或者用下面的motors也是可以的
		motors->rc_write(i,motor_out[i]);
		std::cout<<"motor_out["<<i<<"]="<<motor_out[i]<<std::endl;
		sleep(1);
	}
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
