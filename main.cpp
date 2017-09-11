/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "copter.h"

#define MAINTASK_TICK_TIME_MS 20
int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;
unsigned int maintask_cnt;

int main(int argc,char * const argv[])
{
	cout<<"Welcome to BitPilot"<<endl;

#if 0
	/*
	 * 先不用apm的硬件抽象层了，传感器数据的获取由王正阳来写，写成任务
	 * 把数据的单位统一后，传到某一个全局变量all_sensors中，这样飞控程序直接从这个全局变量
	 * 获取数据，不用修改每个源文件了
	 */
	hal.run(argc,argv,&copter);

	return 0;

#else

	/*
	 * hal的run函数就是调用copter，
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
#if 1
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

	if(maintask_cnt>50)
	{
#if LINUX_OS
		std::cout<<"*********maintask_cnt>50*********************************************************"<<std::endl;
		float system_time_s=0;
		system_time_s=clock_gettime_ms();
		std::cout<<"system_time_s="<<system_time_s/1000<<std::endl;
		maintask_cnt=0;
#endif
	}

	loop_fast();

	update_trig();

	if(maintask_cnt%10)
	{
		//100ms  10hz 的循环
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
		std::cout<<"update_navigation    original_target_bearing="<<original_target_bearing<<std::endl;

		if(control_mode == AUTO)
		{
			if(home_is_set == true && g.command_total > 1)
			{
				//std::cout<<"update_commands()"<<std::endl;
				//command_nav_queue.id = NO_COMMAND;
				printf("home is set     command_nav_queue.id=%d\n",command_nav_queue.id);
				update_commands();
			}
		}
	}

	if(maintask_cnt%2)
	{//50hz
		invalid_throttle=true;
		update_throttle_mode();
	}

	if(maintask_cnt%100)
	{

	    unsigned char buf_data[256];
	    unsigned char buf_packet[256];
	    int ret;
	    static int real_cnt;
	    real_cnt++;


	    std::cout<<"send ap2gcs current_loc="<<current_loc.lng<<std::endl;
		//ap2gcs.lng=current_loc.lng*1e-2;
	    ap2gcs.lng=-current_loc.lng*1e-2;
		ap2gcs.lat=current_loc.lat*1e-2;
		ap2gcs.alt=current_loc.alt*1e-2;

	    //20170728把帧头帧尾加入到数据结构中
	    static unsigned char frame_len=76;
	    static unsigned char frame_head_len=8;
	    static unsigned char frame_checksum_len=2;
	    static unsigned char frame_data_len;
	    frame_data_len=frame_len-frame_head_len-frame_checksum_len;

	    memcpy(buf_data, &ap2gcs.pack_func_flag, frame_data_len);
		ret=generate_packet(buf_packet, buf_data, frame_data_len,\
												real_cnt, 0x10,\
												0,1);
#ifdef LINUX_OS
		send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf_packet,ret);
#endif
	}
}

