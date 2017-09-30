/*
 * system.cpp
 *
 *  Created on: 2017-9-19
 *      Author: wangbo
 */

#include "copter.h"


void Copter::startup_ground( void )
{

}

void Copter::init_ardupilot()
{
	/*
	 * init_ardupilot这个函数主要是飞控外部硬件设备的初始化
	 * 比如设置gps用哪一个串口，gcs地面站用哪一个串口
	 * 比如串口设置波特率，iic总线设置，spi总线设置
	 * 比如加载flash中的参数或者上一次飞行航点
	 * 比如设置mavlink的系统id和设备id
	 */

	//设置地面站串口波特率等
	// init the GCS，地面站以Serial这个硬件驱动串口初始化，其实我并没有用，我把串口已经设置为1了
	char uart_device[]={'u','a','r','t','\0'};
	gcs0.init(uart_device);

	//设置mavlink协议中地面站的idmavlink_system.sysid =

	// Console serial port设置交互中断串口波特率等

	// GPS serial port设置gps串口波特率等

	//如果只考虑增稳控制，我们传感器只需要imu的加速度计acc和陀螺仪gyro就可以了
	//如果只在增稳基础上能够绕航点航行我们需要加上gps（测位置 测高 测地速） compass（测航向） barometer（测高度 动压测空速）
	//这样就需要用到iic或者spi等，其实串口也是可以的，如果这些设备都有串口的话，岂不是很好写程序了，
	//但是有时候为了提升传感器的速度或者串口数量限制，必须多用几个iic设备

	//iic设备初始化

	//spi设备初始化

	//气压计初始化init_barometer();

	//加载参数

	//打开日志文件，日志初始化

	/*
	 * 剩下的基本跟硬件也就没关系了，主要是一些飞控程序上的初始化
	 */

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

	// initialize commands初始化航点命令，最开始都是没有任何命令的
	// -------------------
	init_commands();

	//这个函数还是比较重要，需要做磁力计的校准
	startup_ground();
}


