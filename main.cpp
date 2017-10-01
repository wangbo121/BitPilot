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
#endif

unsigned int maintask_cnt;

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

    // try to send any deferred messages if the serial port now has
    // some space available
	//20170930apm这里是发送延迟的信息，但是我不需要，屏蔽掉了
    //gcs_send_message(MSG_RETRY_DEFERRED);

	/*
	 * 快循环，用来保证增稳，即使没有gps也应该可以手动遥控增稳飞行，只需要imu和dcm算法就可以了
	 * 其实fast_loop应该跟其他函数隔离开来，这个是保证飞机稳定的，其他的跟loop_fast无关，在apm2.0.50中fast_loop频率是200hz
	 * 其他函数都是在这个函数执行结束后才会运行，所以我在后面需要把这个改成最新的apm的任务模式
	 * 目前最新的apm是保证快循环在2.5ms内必须运行一次，如果快循环只用了0.5ms，那就剩下的2ms用来执行其他的任务，
	 * 那么如何保证每个任务都能够按照期望的时间内，至少运行一次呢，还没有看明白
	 * 总之，Copter::loop这个函数的执行频率必须是20ms或者某一个周期执行，而loop_fast必须在这个周期内执行一次，在loop_fast执行结束后计时
	 * 看还剩下多少，比如Copter::loop是20ms loop_fast执行了10ms，那就还剩下10ms，剩下的medium_loop(); fifty_hz_loop();等函数都必须在这个10ms内
	 * 执行结束，如果没有执行结束，则应该放弃某些任务
	 */
	loop_fast();

	/*
	 * // reads all of the necessary trig functions for cameras, throttle, etc.
	 * 这个是更新所有需要的三角函数的数值	trigonometric function （三角函数）
	 * 是用来计算auto_roll的
	 */
	update_trig();

	medium_loop();

	// Stuff to run at full 50hz, but after the med loops
	// --------------------------------------------------
	fifty_hz_loop();

	maintask_cnt++;
	// trigger our 1 hz loop
	//因为我这里改成了100hz所以需要改成100
	if(maintask_cnt>100)
	{
		DEBUG_PRINTF("**********************************  1秒钟，发送心跳包******************************************************************************\n");
		super_slow_loop();//心跳包在super_slow_loop里面

		maintask_cnt = 0;
	}
}
