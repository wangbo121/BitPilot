/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

#ifdef  LINUX_OS
//#define MAINTASK_TICK_TIME_MS 20
#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms主要是为了跟sim_aircraft的仿真频率一致，其实20ms（50hz就够）
int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

void verify_send_all_waypoint_to_gcs( void);
#endif

unsigned int maintask_cnt;

/*
 * 这两个函数即使不在linux系统上也是需要的
 * 按照mavlink协议发送数据给地面站
 */
void send_heartbeat_to_gcs( void );
void send_attitude_to_gcs( void );

int main(int argc,char * const argv[])
{
	DEBUG_PRINTF("Welcome to BitPilot\n");

#if 0
	/*
	 * 先不用apm的硬件抽象层了，传感器数据的获取由王正阳来写，写成任务
	 * 把数据的单位统一后，传到某一个全局变量all_sensors_in和all_sensors_out中，这样飞控程序直接从这个全局变量
	 * 获取数据，不用修改每个源文件了
	 */
	hal.run(argc,argv,&copter);

	return 0;

#else

	/*
	 * hal的run函数就是载入copter实例，然后执行copter的setup和loop函数，
	 * 跟直接写在下面是一样的
	 */

	//初始化工作
	copter.setup();

	while (1)
	{
#ifdef LINUX_OS
		//20170919为了跟sim_multicopter的动力模型计算频率一致，我们现在的循环频率是100hz
		maintask_tick.tv_sec = seconds;
		maintask_tick.tv_usec = mseconds;
		select(0, NULL, NULL, NULL, &maintask_tick);
		maintask_cnt++;
#endif

		/*
		 * 如果这个while(1)的循环周期是10ms那么
		 * 这个loop循环中所有的函数都执行一边（或者说运行最多函数时），所需要的时间应该是小于10ms的
		 */
		copter.loop();
	}

	return 0;
#endif
}

void Copter::loop()
{
	/*
	 * 快循环，用来保证增稳，即使没有gps也应该可以手动遥控增稳飞行
	 */
	loop_fast();

	/*
	 * // reads all of the necessary trig functions for cameras, throttle, etc.
	 * 这个是更新所有需要的三角函数的数值	trigonometric function （三角函数）
	 */
	update_trig();

	// check for new GPS messages
	// --------------------------
//	if(GPS_enabled){
//		update_GPS();
//	}

	medium_loop();

	// Stuff to run at full 50hz, but after the med loops
	// --------------------------------------------------
	fifty_hz_loop();

	counter_one_herz++;

	// trigger our 1 hz loop
	//if(counter_one_herz >= 50){
	//因为我这里改成了100hz所以需要改成100
	if(counter_one_herz >= 100)
	{
		super_slow_loop();
		counter_one_herz = 0;

		/*
		 * 发送数据包给地面站,但是里面的串口发送还用的是linux的,仍然需要更改
		 */
		//send_heartbeat_to_gcs();
		//send_attitude_to_gcs();
	}

#ifdef LINUX_OS
	maintask_cnt++;
	if(maintask_cnt>100)
	{
		//这个是1秒钟打印一次
		DEBUG_PRINTF("*********maintask_cnt>100*********************************************************");
		float system_time_s=0;
		system_time_s=clock_gettime_ms();
		DEBUG_PRINTF("system_time_s==%f\n",system_time_s/1000);


		//发送实时数据给地面站，只是作为在linux平台的测试，在linux平台上暂时测试是1秒钟发送一个实时数据包
		send_realdata_to_gcs();

		maintask_cnt=0;
	}

	//这个是10ms就判断一次是否收到地面站请求回传航点的命令
	verify_send_all_waypoint_to_gcs();
#endif
}

void send_heartbeat_to_gcs( void )
{
#ifdef LINUX_OS
	// 发送心跳包
	send_heartbeat_to_gcs();
	mavlink_system.sysid=1;
	mavlink_system.compid=1;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t mav_send_buf[MAVLINK_MAX_PACKET_LEN];

	uint8_t system_type=	MAV_TYPE_QUADROTOR;
	uint8_t autopilot_type=MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode=MAV_MODE_PREFLIGHT;
	uint8_t custom_mode=0;
	uint8_t system_state=MAV_STATE_STANDBY;

	int len;
	//send heart beat
	mavlink_system.compid = MAV_COMP_ID_ALL;
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type,autopilot_type, system_mode, custom_mode, system_state);
	len = mavlink_msg_to_send_buffer(mav_send_buf, &msg);
#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)mav_send_buf,len);
#endif
#endif
}

void send_attitude_to_gcs( void )
{
#ifdef LINUX_OS
	mavlink_system.sysid=1;
	mavlink_system.compid=1;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t mav_send_buf[MAVLINK_MAX_PACKET_LEN];

	int len;
	mavlink_system.compid = MAV_COMP_ID_ALL;

	//send IMU
	mavlink_msg_attitude_pack(  mavlink_system.sysid,mavlink_system.compid,  &msg,  maintask_cnt,
														fdm_feed_back.phi, fdm_feed_back.theta, fdm_feed_back.psi,
														fdm_feed_back.phidot,fdm_feed_back.thetadot,fdm_feed_back.psidot);

	len = mavlink_msg_to_send_buffer(mav_send_buf, &msg);
#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)mav_send_buf,len);
#endif
#endif
}

#ifdef  LINUX_OS
void verify_send_all_waypoint_to_gcs( void)
{
	if(global_bool_boatpilot.send_ap2gcs_wp_req)
	{
		//global_bool_boatpilot.wp_total_num=4;
		//global_bool_boatpilot.send_ap2gcs_wp_end_num=3;
		printf("电台--请求发送航点数据给地面站\n");
		global_bool_boatpilot.ap2gcs_wp_cnt++;

		if(global_bool_boatpilot.ap2gcs_wp_cnt_previous!=global_bool_boatpilot.ap2gcs_wp_cnt)
		{
			ap2gcs_wp.pack_func_info3=global_bool_boatpilot.ap2gcs_wp_cnt;

			if(global_bool_boatpilot.send_ap2gcs_wp_end_num>=global_bool_boatpilot.wp_total_num-1)
			{
				global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.wp_total_num-1;
			}
			send_ap2gcs_waypoint_num(global_bool_boatpilot.send_ap2gcs_wp_start_num,global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1);
			global_bool_boatpilot.ap2gcs_wp_cnt_previous=global_bool_boatpilot.ap2gcs_wp_cnt;
			global_bool_boatpilot.send_ap2gcs_wp_req=FALSE;
		}
	}
}
#endif
