/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

//#include "mavlink.h"
mavlink_system_t mavlink_system;



#ifdef  LINUX_OS
//#define MAINTASK_TICK_TIME_MS 20
#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms主要是为了跟sim_aircraft的仿真频率一致，其实20ms（50hz就够）
int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;
#endif

unsigned int maintask_cnt;

int main(int argc,char * const argv[])
{
	cout<<"Welcome to BitPilot"<<endl;

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
	 * hal的run函数就是载入copter实例，
	 * 然后执行copter的setup和loop函数，
	 * 跟直接写在下面是一样的
	 */

	/*
	 * 初始化工作
	 */
	copter.setup();

	/*
	 * 这个while循环是20ms
	 * 也就是50hz，如果写在ucOS系统中，则该任务是20ms的
	 */
	while (1)
	{

//#if 1
#ifdef LINUX_OS
		maintask_tick.tv_sec = seconds;
		maintask_tick.tv_usec = mseconds;
		select(0, NULL, NULL, NULL, &maintask_tick);
		maintask_cnt++;
#endif

		copter.loop();//20ms一个周期的运行，那么这个loop循环中所有的函数都执行一边（或者说运行最多函数时），所需要的时间应该是小于20ms的





	}

	return 0;
#endif

}

void Copter::loop()
{
	maintask_cnt++;

	if(maintask_cnt>100)
	{
#ifdef LINUX_OS
		std::cout<<"*********maintask_cnt>100*********************************************************"<<std::endl;
		float system_time_s=0;
		system_time_s=clock_gettime_ms();
		//std::cout<<"system_time_s="<<system_time_s/1000<<std::endl;
		printf_debug("system_time_s==%f\n",system_time_s/1000);
		maintask_cnt=0;
#endif
	}

	loop_fast();

	/*
	 * // reads all of the necessary trig functions for cameras, throttle, etc.
	 * 这个是更新所有需要的三角函数的数值
	 */
	update_trig();

	// check for new GPS messages
	// --------------------------
//	if(GPS_enabled){
//		update_GPS();
//	}

#if 1
	medium_loop();

#else
	if(maintask_cnt%10)
	{
		//10hz
		/*
		 * 其实fast_loop应该就是仅仅包括radio ahrs update_rate_control servo_out就可以了，不需要知道高度，gps等
		 * 但是ahrs为了更加准确，加入了gps作为融合，所以把gps的读取放到了fastloop里，但其实gps更新速率应该是
		 * 没有那么快的，10hz的gps是比较常规的，fastloop是50hz的
		 */

		//更新高度，气压计或者gps高度
		update_alt();

		// calculate distance, angles to target
		navigate();

		// this calculates the velocity for Loiter
		// only called when there is new data
		// ----------------------------------
		calc_XY_velocity();

		// update flight control system
		update_navigation();
		//printf_debug("update_navigation    original_target_bearing=%d\n",original_target_bearing);

		if(control_mode == AUTO)
		{
			if(home_is_set == true && g.command_total > 1)
			{
				//command_nav_queue.id = NO_COMMAND;
				printf_debug("home is set     command_nav_queue.id=%d\n",command_nav_queue.id);
				update_commands();
			}
		}
	}
#endif

	// Stuff to run at full 50hz, but after the med loops
	// --------------------------------------------------
	fifty_hz_loop();

	counter_one_herz++;

	// trgger our 1 hz loop
	//if(counter_one_herz >= 50){
	//因为我这里改成了100hz所以需要改成100
	if(counter_one_herz >= 100){
		super_slow_loop();
		counter_one_herz = 0;
	}

#if 0
	这个更新油门模式的已经在50hz的循环里有了，不需要再写了
	if(maintask_cnt%2)
	{//50hz
		invalid_throttle=true;
		update_throttle_mode();
	}
#endif

	if(maintask_cnt%100)
	{
#ifdef LINUX_OS
		//发送实时数据给地面站，只是作为在linux平台的测试，在linux平台上暂时测试是1秒钟发送一个实时数据包
		send_realdata_to_gcs();


		/*
		 * 发送心跳包
		 */
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

		 send_uart_data(uart_device_ap2gcs.uart_name, (char *)mav_send_buf,len);

	    //send IMU

#endif
	}


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



