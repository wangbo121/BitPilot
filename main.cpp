/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

/*
 * 添加飞行动力模型
 *
 */
//#include "BIT_MATH.h"
//#include "utility.h"
//
//#include "aircraft.h"
//#include "quadcopter.h"
#include "fdm.h"

#include "SIM_Multicopter.h"
#include "SITL.h"
#include "udp.h"


//MultiCopter multi_copter("120,48,100,10","x");
//MultiCopter multi_copter("-122.357,37.6136,100,10","x");
//MultiCopter multi_copter("-122.357,37.6136,100,0","x");
//MultiCopter multi_copter("-122.357,37.6136,100,10","+");
MultiCopter multi_copter("-122.357,37.6136,100,0","+");
//MultiCopter multi_copter("-122.357192862,37.6135553166,100,0","+");

//MultiCopter multi_copter;
T_FDM fdm;
T_FDM fdm_send;
T_FDM fdm_feed_back;//这个动力模型解算出来后，就把数据返回给imu，gps，compass等




#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <string.h>
/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>

#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <string.h>
#include "udp.h"



#include "copter.h"

/******************************************************/
/*****************/
#define MAINTASK_TICK_TIME_MS 20
//#define MAINTASK_TICK_TIME_MS 2
struct tm *global_time_val;//全局时间变量，其它的时间都从这里取
time_t timep;

int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

int maintask_cnt;

#define ONE_HZ_CNT  50//1hz
#define TEN_HZ_CNT   5//10hz
#define FIFTY_HZ_CNT 1//50hz

int _input_flag=1;


/*
 * 添加动力模型
 */
//#define TEST
//#ifdef TEST
//char udp_string[]="0123456";
//#endif

T_GLOBAL  gblState;
T_AP2FG  ap2fg;
T_FG2AP fg2ap;
T_AP2FG  ap2fg_send;
T_AP2FG  ap2fg_send_test;
T_AP2FG  ap2fg_send_test_send;


T_AP2FG  ap2fg_recv;

/*
 * //这个作为遥控器的输入信号，或者说遥控器进来后经过出去又要out给servos
 * 虽然四旋翼最终控制的是motor，不是固定翼的servos，但是这个motor也是用servo的pwm波信号
 * 包括其他的舵机都是servo的1000-2000pwm波信号，所以我们统一用servo作为最终电机的
 * 有servo_in 和servo_out 其中servo_in就是等于channel_out的，然后进来后可能还要限幅什么的所以最终输出给电机的叫做
 * servo_out
*/
/*
 * channel_out[0]:aileron
 * channel_out[1]:elevator
 * channel_out[2]:throttle
 * channel_out[3]:rudder
*/
float channel_out[16];

float servos_set[16];

uint16_t servos_set_out[4];//这是驾驶仪计算的到的motor_out中的四个电机的转速，给电调的信号，1000～2000

Aircraft::sitl_input input;

int fd_socket_generic;
int16_t             motor_out_flightgear[AP_MOTORS_MAX_NUM_MOTORS];

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

	init_ardupilot();

	//第一级pid参数设置
	g.pi_stabilize_roll.set_kP(2.0);
	g.pi_stabilize_roll.set_kI(0.0);
	g.pi_stabilize_roll.set_kD(0.0);

	g.pi_stabilize_pitch.set_kP(2.0);
	g.pi_stabilize_pitch.set_kI(0.0);
	g.pi_stabilize_pitch.set_kD(0.0);

	g.pi_stabilize_yaw.set_kP(2.0);
	g.pi_stabilize_yaw.set_kI(0.0);
	g.pi_stabilize_yaw.set_kD(0.0);


	//第2级pid参数设置


	g.pid_rate_roll.set_kP(1.2);
	g.pid_rate_roll.set_kI(0.0);
	g.pid_rate_roll.set_kD(0.0);

	g.pid_rate_pitch.set_kP(1.2);
	g.pid_rate_pitch.set_kI(0.0);
	g.pid_rate_pitch.set_kD(0.0);

	g.pid_rate_yaw.set_kP(1.2);
	g.pid_rate_yaw.set_kI(0.0);
	g.pid_rate_yaw.set_kD(0.0);


	/*
	 * 下面这些初始化，其实应该放在跟地面站连接时
	 * 地面站的setup按钮里，设置遥控器的最大最小值
	 * 但是我这里先直接赋值
	 */

	init_rc_in();

	fast_loopTimer=clock_gettime_ms();//必须有这个初始化，否则第一次G_Dt的值会非常大

	roll_pitch_mode=ROLL_PITCH_STABLE;
	yaw_mode=YAW_STABILE;



	/*
	 * 添加动力模型
	 */


	servos_set[0]=1500;
	servos_set[1]=1500;
	servos_set[2]=1500;
	servos_set[3]=1500;
	memcpy(input.servos,servos_set,sizeof(servos_set));

	open_udp_dev(IP_SEND_TO, PORT_SENT_TO, PORT_RECEIVE);//发送fdm飞行动力模型给flightgear，从而能够呈现姿态等
	open_socket_udp_dev(&fd_socket_generic,"127.0.0.1",5056);//发送generic的协议给flightgear，从而能够螺旋桨能够旋转

	ap2fg.throttle0 = 0.2;
	ap2fg.throttle1 = 0.3;
	ap2fg.throttle2 = 0.4;
	ap2fg.throttle3 = 0.5;

	ap2fg_send_test.throttle0=0;
	ap2fg_send_test.throttle1=0;
	ap2fg_send_test.throttle2=0;
	ap2fg_send_test.throttle3=0;
}

void Copter::loop()
{
	maintask_tick.tv_sec = seconds;
	//maintask_tick.tv_usec = mseconds*50;
	maintask_tick.tv_usec = mseconds;

	maintask_tick.tv_sec = 1;
	maintask_tick.tv_usec = 0;
	select(0, NULL, NULL, NULL, &maintask_tick);
	maintask_cnt++;

	if(maintask_cnt>50)
	{
		std::cout<<"*********maintask_cnt>50*********************************************************"<<std::endl;
		float system_time_s=0;
		system_time_s=clock_gettime_ms();
		std::cout<<"system_time_s="<<system_time_s/1000<<std::endl;
		maintask_cnt=0;
	}

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
	 * 4--control根据飞行模式 control_mode的选项，选择不同的控制方式
	 * 5--set_servos
	 */
	float timer;
	timer=clock_gettime_ms();
	//std::cout<<"timer="<<timer/1000<<std::endl;

	G_Dt=(timer-fast_loopTimer)/1000.0;//单位是秒[s]
	//std::cout<<"G_Dt="<<G_Dt<<std::endl;

	fast_loopTimer=timer;

	/* 1--读取接收机的信号，获取遥控器各个通道 */
	read_radio();

//	g.channel_rudder.set_pwm(1600);//这个set_pwm参数的范围是1000～2000
	g.channel_pitch.set_pwm(1600);//这个set_pwm参数的范围是1000～2000，把pitch一直设置为1600，看能不能稳定在9度左右
	g.rc_5.set_pwm(1400);//rc_5大于1500时，是增稳控制状态
	//g.rc_5.set_pwm(1600);//rc_5大于1500时，是增稳控制状态

	/* 2--更新姿态，获取飞机现在的姿态角 */
	compass.read();
	gps.read();
	imu.update();
	/*
	 * 因为下面的ahrs中需要imu gps compass的数据，
	 * 所以需要先读取那些传感器的数据
	 */
	ahrs.update_DCM(G_Dt);

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
		* update_roll_pitch_mode和update_yaw_mode都是只有p控制器，计算得到目标姿态角度
		*/
		update_roll_pitch_mode();
		update_yaw_mode();//上面这两个函数有问题呀，上面两个函数赋值给的是EARTH_FRAME，但是下面的run_rate_controllers是用的BODY_FRAME，所以还需要仔细再看一下apm

	    //这个是更新内环的速率控制器的目标，update targets to rate controllers
	    update_rate_contoller_targets();//这个步骤很重要，是把上面的earth坐标系下的转为机体坐标系

	    //这个是执行了角速度的控制器，需要从ahrs或者imu获取角速度的大小，扩大了100倍，这个函数还得看一下
		run_rate_controllers();

		//这个是油门的控制，跟姿态的控制分开
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

	std::cout<<"g.channel_roll.radio_out="<<g.channel_roll.radio_out<<std::endl;
	std::cout<<"g.channel_pitch.radio_out="<<g.channel_pitch.radio_out<<std::endl;
	std::cout<<"g.channel_throttle.radio_out="<<g.channel_throttle.radio_out<<std::endl;
	std::cout<<"g.channel_rudder.radio_out="<<g.channel_rudder.radio_out<<std::endl;

	/*
	 * motors_output()函数中把motor_out[]，也就是每个电机的1000～2000的值赋值给了
	 */
	servos_set_out[0]=motor_out_flightgear[0];
	servos_set_out[1]=motor_out_flightgear[1];
	servos_set_out[2]=motor_out_flightgear[2];
	servos_set_out[3]=motor_out_flightgear[3];

	for(int i=0;i<4;i++)
	{
		std::cout<<"servos_set_out"<<"["<<i<<"]="<<servos_set_out[i]<<std::endl;
	}

	/*
	 * 注意这里的input.servos是uint_16类型的，无符号short型，
	 * 所以servos_set_out一定不要有负数，所以需要提前处理servos_set_out
	*/
	memcpy(input.servos,servos_set_out,sizeof(servos_set_out));

	multi_copter.update(input);//利用input更新，copter四旋翼的位置，速度，线加速度，角度，角速度，角加速度是没有的，所以一共3*5=15个数据
	multi_copter.fill_fdm_flightgear(fdm);

	memcpy(&fdm_feed_back,&fdm,sizeof(fdm));
	memcpy(&fdm_send,&fdm,sizeof(fdm));

	fdm_send.version = htonl(FG_NET_FDM_VERSION);
	fdm_send.latitude = htond(fdm_send.latitude);
	fdm_send.longitude = htond(fdm_send.longitude);
	fdm_send.altitude = htond(fdm_send.altitude);
	fdm_send.phi = htonf(fdm_send.phi );
	//fdm_send.phi = htonf(1.0 );
	fdm_send.theta = htonf(fdm_send.theta);
	fdm_send.psi = htonf(fdm_send.psi);
	//fdm_send.psi = htonf(radians(57));
	fdm_send.num_engines = htonl(1);
	fdm_send.num_tanks = htonl(1);
	fdm_send.fuel_quantity[0] = htonf(100.0);
	fdm_send.num_wheels = htonl(3);
	fdm_send.cur_time = htonl(time(0));
	fdm_send.warp = htonl(1);
	fdm_send.visibility = htonf(5000.0);

	sendto(fd_sock_send, &fdm_send, sizeof(fdm_send), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));









	/*
	 * 下面是发送数据给flightgear的，20170818不再需要更改
	 */

	/*
	 * motors_output()函数中把motor_out[]，也就是每个电机的1000～2000的值赋值给了
	 * motor_out_flightgear，我们用来把这些数给到flightgear用来显示和控制螺旋桨转动
	 */
	servos_set_out[0]=motor_out_flightgear[0];
	servos_set_out[1]=motor_out_flightgear[1];
	servos_set_out[2]=motor_out_flightgear[2];
	servos_set_out[3]=motor_out_flightgear[3];

//	ap2fg.throttle0 = ((float)(servos_set_out[0])-1000.0)/1000.0;
//	ap2fg.throttle1 = ((float)(servos_set_out[1])-1000.0)/1000.0;
//	ap2fg.throttle2 = ((float)(servos_set_out[2])-1000.0)/1000.0;
//	ap2fg.throttle3 = ((float)(servos_set_out[3])-1000.0)/1000.0;
//
//	std::cout<<"ap2fg.throttle0="<<ap2fg.throttle0<<std::endl;
//	std::cout<<"ap2fg.throttle1="<<ap2fg.throttle1<<std::endl;
//	std::cout<<"ap2fg.throttle2="<<ap2fg.throttle2<<std::endl;
//	std::cout<<"ap2fg.throttle3="<<ap2fg.throttle3<<std::endl;

	/*
	 * 固定发送给flightgear的油门速度，能够看出旋转的方向来
	 * 因为不知道为什么arducopter这个模型，在1500转左侧是逆时针转动
	 * 在1500转右侧hi顺时针转动，所以这里就是让看个螺旋桨转动的趋势
	 * 具体的数值换别的控制量来显示
	 * 不要删除20170818
	 */
	ap2fg.throttle0=0.1420;
	ap2fg.throttle1=0.1420;
	ap2fg.throttle2=0.1580;
	ap2fg.throttle3=0.1580;

	ap2fg.rpm0 = ((float)(servos_set_out[0])-1000.0);
	ap2fg.rpm1 = ((float)(servos_set_out[1])-1000.0);
	ap2fg.rpm2 = ((float)(servos_set_out[2])-1000.0);
	ap2fg.rpm3 = ((float)(servos_set_out[3])-1000.0);

	ap2fg_send.throttle0=hton_double(ap2fg.throttle0);
	ap2fg_send.throttle1=hton_double(ap2fg.throttle1);
	ap2fg_send.throttle2=hton_double(ap2fg.throttle2);
	ap2fg_send.throttle3=hton_double(ap2fg.throttle3);
	ap2fg_send.rpm0=hton_double(ap2fg.rpm0);
	ap2fg_send.rpm1=hton_double(ap2fg.rpm1);
	ap2fg_send.rpm2=hton_double(ap2fg.rpm2);
	ap2fg_send.rpm3=hton_double(ap2fg.rpm3);

	unsigned char socket_udp_send[2000];
	memcpy(socket_udp_send,&ap2fg_send,sizeof(ap2fg_send));
	send_socket_udp_data(fd_socket_generic, socket_udp_send, sizeof(ap2fg_send),"127.0.0.1",5506 );

#if 0
	//被注释掉的是用来进行测试的，上面的是实际发送的

	/*
	 * 固定发送给flightgear的油门速度，能够看出旋转的方向来
	 * 因为不知道为什么arducopter这个模型，在1500转左侧是逆时针转动
	 * 在1500转右侧hi顺时针转动，所以这里就是让看个螺旋桨转动的趋势
	 * 具体的数值换别的控制量来显示
	 * 不要删除20170818
	 */
	ap2fg_send_test.throttle0=0.1420;
	ap2fg_send_test.throttle1=0.1420;
	ap2fg_send_test.throttle2=0.1580;
	ap2fg_send_test.throttle3=0.1580;

	ap2fg_send_test_send.throttle0=hton_double(ap2fg_send_test.throttle0);
	ap2fg_send_test_send.throttle1=hton_double(ap2fg_send_test.throttle1);
	ap2fg_send_test_send.throttle2=hton_double(ap2fg_send_test.throttle2);
	ap2fg_send_test_send.throttle3=hton_double(ap2fg_send_test.throttle3);

	ap2fg_send_test.rpm0=1000;
	ap2fg_send_test.rpm1=800;
	ap2fg_send_test.rpm2=500;
	ap2fg_send_test.rpm3=300;

	ap2fg_send_test_send.rpm0=hton_double(ap2fg_send_test.rpm0);
	ap2fg_send_test_send.rpm1=hton_double(ap2fg_send_test.rpm1);
	ap2fg_send_test_send.rpm2=hton_double(ap2fg_send_test.rpm2);
	ap2fg_send_test_send.rpm3=hton_double(ap2fg_send_test.rpm3);

	unsigned char socket_udp_send[2000];
	memcpy(socket_udp_send,&ap2fg_send_test_send,sizeof(ap2fg_send_test_send));
	send_socket_udp_data(fd_socket_generic, socket_udp_send, sizeof(ap2fg_send_test_send),"127.0.0.1",5506 );
#endif
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
		std::cout<<"飞控模式是增稳模式:"<<std::endl;
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
	int16_t             motor_out[AP_MOTORS_MAX_NUM_MOTORS];

    int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float              _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)

#if 0
    //这个是x型的
    /*
     * 这里给factor赋值-1 0 或者1
     */
    _roll_factor[0]  =  -1;  _pitch_factor[0]  = +1;  _yaw_factor[0]  = +1;
    _roll_factor[1]  =  -1;  _pitch_factor[1]  =  -1;  _yaw_factor[1]  =  -1;
    _roll_factor[2]  = +1;  _pitch_factor[2]  =  -1;  _yaw_factor[2]  = +1;
    _roll_factor[3]  = +1;  _pitch_factor[3]  = +1;  _yaw_factor[3]  =  -1;
#else

    //这个是针对+型机架的系数
	_roll_factor[0]  =  -1;  _pitch_factor[0]  =  0; _throttle_factor[0]=+1; _yaw_factor[0]  = +1;
	_roll_factor[1]  =   +1;  _pitch_factor[1]  =  0; _throttle_factor[1]=+1; _yaw_factor[1]  =  +1;
	_roll_factor[2]  = 0;  _pitch_factor[2]  =  +1;  _throttle_factor[2]=+1;_yaw_factor[2]  = -1;
	_roll_factor[3]  =  0;  _pitch_factor[3]  = -1;  _throttle_factor[3]=+1;_yaw_factor[3]  =  -1;
#endif


	g.channel_roll.calc_pwm();
	g.channel_pitch.calc_pwm();
	g.channel_rudder.calc_pwm();
	g.channel_throttle.calc_pwm();

	std::cout<<"g.channel_roll.servo_out="<<g.channel_roll.servo_out<<std::endl;
	std::cout<<"g.channel_pitch.servo_out="<<g.channel_pitch.servo_out<<std::endl;
	std::cout<<"g.channel_rudder.servo_out="<<g.channel_rudder.servo_out<<std::endl;
	std::cout<<"g.channel_throttle.servo_out="<<g.channel_throttle.servo_out<<std::endl;

	std::cout<<"********准备输出pwm脉宽给电调***********"<<std::endl;

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
		motor_out[i]=g.channel_throttle.radio_out*_throttle_factor[i]+ \
								g.channel_roll.pwm_out*_roll_factor[i]+\
								g.channel_pitch.pwm_out* _pitch_factor[i]+\
								g.channel_rudder.pwm_out * _yaw_factor[i];
	}

	for(int i=0;i<4;i++)
	{
		//g._rc.output_ch_pwm(i,motor_out[i]);
		//或者用下面的motors也是可以的
		//motors.rc_write(i,motor_out[i]);
		std::cout<<"motor_out["<<i<<"]="<<motor_out[i]<<std::endl;
		//sleep(1);
	}

	memcpy(motor_out_flightgear,motor_out,sizeof(motor_out));//模型fdm需要的四个输入量就是电机的1000-2000的信号量

	for(int i=0;i<4;i++)
	{
		//g._rc.output_ch_pwm(i,motor_out[i]);
		//或者用下面的motors也是可以的
		//motors.rc_write(i,motor_out[i]);
		std::cout<<"motor_out_flightgear["<<i<<"]="<<motor_out_flightgear[i]<<std::endl;
		//sleep(1);
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


void Copter:: init_ardupilot()
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
