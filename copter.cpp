/*
 * copter.cpp
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

#include "copter.h"

#ifdef LINUX_OS
/*
 * 四旋翼的飞行动力模型输入是4个电机的1000-2000的pwm值，输出是飞行的15个状态
 */
//MultiCopter multi_copter("37.6136,-122.357,10,0","x");//x型机架，起始高度为10，yaw是0
MultiCopter multi_copter("37.6136,-122.357,10,0","+");//+型机架，起始高度为10，yaw是0
#endif

//fdm是从四旋翼的simulate数学模型中获取数据，
//然后fdm_send发送给flightgear，
//fdm_feed_back作为状态观测或者说是传感器把数据反馈回来
T_FDM fdm;
T_FDM fdm_send;
T_FDM fdm_feed_back;//这个动力模型解算出来后，就把数据返回给imu，gps，compass等

#ifdef LINUX_OS
int16_t motor_out_flightgear[AP_MOTORS_MAX_NUM_MOTORS];//这个不能删除，本来是输出给flightgear的但是这个1000-2000的量又得赋值给fdm模拟用的servos_set_out
uint16_t servos_set_out[4];//这是驾驶仪计算的到的motor_out中的四个电机的转速，给电调的信号，1000～2000

Aircraft::sitl_input input;//这个是4个电机的输入，然后用于multi_copter.update(input)更新出飞机的飞行状态
#endif

/*
 * wangbo20170802setup包括2部分，
 * 1是init_ardupilot是用来初始化硬件驱动的，
 * 2是剩下部分是用来初始化参数或者状态的
 */
void Copter::setup()
{
	init_ardupilot();

	//一共10组pid参数
	//第一级pid参数设置
	float stabilize_roll_p=3.69;
	float stabilize_roll_i=0.0;
	float stabilize_roll_d=0.0;
	g.pi_stabilize_roll.set_kP(stabilize_roll_p);
	g.pi_stabilize_roll.set_kI(stabilize_roll_i);
	g.pi_stabilize_roll.set_kD(stabilize_roll_d);

	float stabilize_pitch_p=3.69;
	//float stabilize_pitch_p=0.69;
	float stabilize_pitch_i=0.0;
	float stabilize_pitch_d=0.0;
	g.pi_stabilize_pitch.set_kP(stabilize_pitch_p);
	g.pi_stabilize_pitch.set_kI(stabilize_pitch_i);
	g.pi_stabilize_pitch.set_kD(stabilize_pitch_d);

	float stabilize_yaw_p=4.0;
	float stabilize_yaw_i=0.0;
	float stabilize_yaw_d=0.0;
	g.pi_stabilize_yaw.set_kP(stabilize_yaw_p);
	g.pi_stabilize_yaw.set_kI(stabilize_yaw_i);
	g.pi_stabilize_yaw.set_kD(stabilize_yaw_d);

	//第2级pid参数设置
	float stabilize_roll_rate_p=0.15;//0.255;
	float stabilize_roll_rate_i=0.1;//0.122;
	float stabilize_roll_rate_d=0.004;//0.017;
	g.pid_rate_roll.set_kP(stabilize_roll_rate_p);
	g.pid_rate_roll.set_kI(stabilize_roll_rate_i);
	g.pid_rate_roll.set_kD(stabilize_roll_rate_d);

	float stabilize_pitch_rate_p=0.15;//0.255;
	float stabilize_pitch_rate_i=0.1;//0.122;
	float stabilize_pitch_rate_d=0.004;//0.017;
	g.pid_rate_pitch.set_kP(stabilize_pitch_rate_p);
	g.pid_rate_pitch.set_kI(stabilize_pitch_rate_i);
	g.pid_rate_pitch.set_kD(stabilize_pitch_rate_d);

	float stabilize_yaw_rate_p=0.2;//0.17;
	float stabilize_yaw_rate_i=0.02;//0.2;
	float stabilize_yaw_rate_d=0.0;//0.003;
	g.pid_rate_yaw.set_kP(stabilize_yaw_rate_p);
	g.pid_rate_yaw.set_kI(stabilize_yaw_rate_i);
	g.pid_rate_yaw.set_kD(stabilize_yaw_rate_d);


	/*
	* 导航用的pid
	*/
	//经度差pid
	float pid_p_3=2.0;
	g.pid_nav_lon.set_kP(pid_p_3);
	g.pid_nav_lon.set_kI(0.122);
	//g.pid_nav_lon.set_kD(0.017);
	g.pid_nav_lon.set_kD(0.0);

	//纬度差pid
	//g.pid_nav_lat.set_kP(pid_p_3);
	g.pid_nav_lat.set_kP(2.0);
	g.pid_nav_lat.set_kI(0.122);//又犯了复制没有改名字的错误，set_kI写成了set_kP，导致p又改为了0
	//g.pid_nav_lat.set_kD(0.017);
	g.pid_nav_lat.set_kD(0.0);

	/*
	* 油门控高
	*/
	float pid_p_alt=1;
	g.pi_alt_hold.set_kP(pid_p_alt);
	g.pi_alt_hold.set_kI(0.0);
	g.pi_alt_hold.set_kD(0.0);

	g.pid_throttle.set_kP(0.5);
	g.pid_throttle.set_kI(1.0);
	g.pid_throttle.set_kD(0.0);
	//上面一共10组pid控制器

	/*
	* 下面这些初始化，其实应该放在跟地面站连接时
	* 地面站的setup按钮里，设置遥控器的最大最小值
	* 但是我这里先直接赋值
	*/

	init_rc_in();

	roll_pitch_mode=ROLL_PITCH_STABLE;
	yaw_mode=YAW_STABILE;
	g.auto_slew_rate=AUTO_SLEW_RATE;

	cos_roll_x 	= 1;
	cos_pitch_x 	= 1;
	cos_yaw_x 		= 1;
	sin_pitch_y=0;
	sin_yaw_y=0;
	sin_roll_y=0;

	yaw_tracking = MAV_ROI_WPNEXT;
	wp_control =WP_MODE;//设置为自动巡航模式

	wp_total_array[0].id=0;

	g.waypoint_speed_max=WAYPOINT_SPEED_MAX;

	/*
	* 为了自行测试绕航点飞行，我先自行设置了初始的经度纬度如下
	*/
	long start_longtitude=-1223571928;//原始gps经度乘以10的7次方，最小一位对应的是厘米
	long start_latitude=376135531;

	int wp_num=10;
	long delta_lon=1e5;//1000米，初始是1000米，但是在后面分别增加了一直向东和绕正方形航行，所以会重新给delta_lon赋值
	long delta_lat=1e5;
	g.waypoint_radius=10;//waypoint_radius的单位是米,程序中比较的是厘米,在程序里面已经乘以100了,这里的单位就是米

	wp_total_array[0].id 	= MAV_CMD_NAV_WAYPOINT;
	wp_total_array[0].lng 	= start_longtitude;				// Lon * 10**7
	wp_total_array[0].lat 	= start_latitude;				// Lat * 10**7
	wp_total_array[0].alt 	= 0;							// Home is always 0

	current_loc.lng=start_longtitude;
	current_loc.lat=start_latitude;

	int alt_temp=5000;

	//这个是经度纬度都增加，也就是一直往北或者一直往东，减小经度
	//		for(int i=1;i<wp_num;i++)
	//		{
	//			wp_total_array[i].id 	= MAV_CMD_NAV_WAYPOINT;
	//			wp_total_array[i].lng 	= start_longtitude+delta_lon*i;				// Lon * 10**7
	//			//wp_total_array[i].lng 	= start_longtitude-delta_lon*i;				// Lon * 10**7
	//			//wp_total_array[i].lng 	= start_longtitude;				// Lon * 10**7
	//			wp_total_array[i].lat 	= start_latitude+delta_lat*i;				// Lat * 10**7
	//			wp_total_array[i].alt 	= alt_temp;							// Home is always 0
	//		}

	/*
	* 这个是矩形，绕航线飞行，每个航点距离是100米
	*/
	for(int i=1;i<wp_num;i++)
	{
	wp_total_array[i].id 	= MAV_CMD_NAV_WAYPOINT;

	wp_total_array[i].alt 	= alt_temp;							// Home is always 0

	delta_lon=10000;
	delta_lat=10000;
	}
	wp_total_array[1].lng 	= start_longtitude+delta_lon*1;				// Lon * 10**7
	wp_total_array[1].lat 	= start_latitude;				// Lat * 10**7

	wp_total_array[2].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
	wp_total_array[2].lat 	= start_latitude;				// Lat * 10**7

	wp_total_array[3].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
	wp_total_array[3].lat 	= start_latitude+delta_lat*1;				// Lat * 10**7

	wp_total_array[4].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
	wp_total_array[4].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7


	wp_total_array[5].lng 	= start_longtitude+delta_lon*1;				// Lon * 10**7
	wp_total_array[5].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7

	wp_total_array[6].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
	wp_total_array[6].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7

	wp_total_array[7].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
	wp_total_array[7].lat 	= start_latitude+delta_lat*1;				// Lat * 10**7

	wp_total_array[8].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
	wp_total_array[8].lat 	= start_latitude+delta_lat*0;				// Lat * 10**7

	wp_total_array[9].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
	wp_total_array[9].lat 	= start_latitude+delta_lat*0;				// Lat * 10**7

	for(int i=0;i<wp_num;i++)
	{
		DEBUG_PRINTF("setup    :    wp_total_array[%d].lng = %d\n",i,wp_total_array[i].lng);
		DEBUG_PRINTF("setup    :    wp_total_array[%d].lng = %d\n",i,wp_total_array[i].lng);
		DEBUG_PRINTF("setup    :    wp_total_array[%d].lng = %d\n",i,wp_total_array[i].lng);
	}

	init_home();

	DEBUG_PRINTF("setup    :    home.lng = %d\n",home.lng);
	DEBUG_PRINTF("setup    :    home.lat = %d\n",home.lat);

	g.waypoint_total=wp_num;
	next_WP.id=MAV_CMD_NAV_WAYPOINT;

	init_commands();
	g.command_total=wp_num;

	command_nav_queue.id = NO_COMMAND;//只有nav_queue的id是没有命令时，update_commands才会生效，更新命令
	command_nav_index=0;

	g.crosstrack_gain=1;

	g.channel_throttle.control_in=500;
	g.channel_throttle.servo_out=500;
	g.throttle_cruise=500;

	g.sonar_enabled=0;

	dTnav=100;//单位应该是毫秒  // Delta Time in milliseconds for navigation computations  因为pid的函数中计算积分所需要的时间就是以毫秒为单位的
	//dTnav=0.1;//单位是秒,20170919应该按照gps更新的速度设置的,那么就应该是10hz或者5hz呀

#ifdef LINUX_OS
	/*
	 * 下面是用来连接flightgear和地面站进行模拟测试的
	 */
	input.servos[0]=1500;
	input.servos[1]=1500;
	input.servos[2]=1500;
	input.servos[3]=1500;
#endif

#ifdef LINUX_OS
	fast_loopTimer=clock_gettime_ms();//必须有这个初始化，否则第一次G_Dt的值会非常大，不过现在有没有都无所谓了，我已经把G_Dt固定为0.01
#endif


#ifdef LINUX_OS
	open_udp_dev(IP_SEND_TO, PORT_SENT_TO, PORT_RECEIVE);//发送fdm飞行动力模型给flightgear，从而能够呈现姿态等
	open_socket_udp_dev(&fd_socket_generic,"127.0.0.1",5056);//发送generic的协议给flightgear，从而能够螺旋桨能够旋转
#endif

#ifdef LINUX_OS
	ap2fg.throttle0 = 0.2;
	ap2fg.throttle1 = 0.3;
	ap2fg.throttle2 = 0.4;
	ap2fg.throttle3 = 0.5;
#endif

#ifdef LINUX_OS
	char *char_uart="/dev/ttyUSB0";

	uart_device_ap2gcs.uart_name=char_uart;

	uart_device_ap2gcs.baudrate=UART_AP2GCS_BAUD;
	uart_device_ap2gcs.baudrate=115200;
	uart_device_ap2gcs.databits=UART_AP2GCS_DATABITS;
	uart_device_ap2gcs.parity=UART_AP2GCS_PARITY;
	uart_device_ap2gcs.stopbits=UART_AP2GCS_STOPBITS;

	uart_device_ap2gcs.uart_num=open_uart_dev(uart_device_ap2gcs.uart_name);

	uart_device_ap2gcs.ptr_fun=read_radio_data;

	set_uart_opt( uart_device_ap2gcs.uart_name, uart_device_ap2gcs.baudrate,
							uart_device_ap2gcs.databits, uart_device_ap2gcs.parity,
							uart_device_ap2gcs.stopbits);

	DEBUG_PRINTF("uart recvbuf 串口名字=%s\n",uart_device_ap2gcs.uart_name);

	create_uart_pthread(&uart_device_ap2gcs);//2017023直接从串口读取，主动读取，不是单独线程了
#endif
}

void Copter::loop_fast()
{
	/*
	 * 将来跟硬件驱动获取数据整合时,这个函数是不需要的,现在是模拟,所以才需要
	 */
	//20170918添加了all_external_device_input和output一直循环从驱动中获取数据，至于硬件驱动到底多大频率获取的我不管，我只是每次从这里获取数据
	update_all_external_device_input();
	update_mavlink_reatime();

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
#ifdef LINUX_OS
	float timer;
	timer=clock_gettime_ms();
	G_Dt=(timer-fast_loopTimer)/1000.0;
	fast_loopTimer=timer;
#endif

	G_Dt=0.01;//G_Dt是dcm积分要用的，这个设置为0.01秒也就是100hz主要是为了跟sim_aircraft的速率一致，但是其实20ms(50hz)就够

	/* 1--读取接收机的信号，获取遥控器各个通道 */
	read_radio();
	//下面的设置遥控器其实是不需要的，应该按照从地面站或者从遥控器的第五通道来决定飞行模式
	//g.channel_rudder.set_pwm(1600);//这个set_pwm参数的范围是1000～2000
	//g.channel_pitch.set_pwm(1600);//这个set_pwm参数的范围是1000～2000，把pitch一直设置为1600，看能不能稳定在9度左右
	//g.rc_5.set_pwm(1400);//rc_5大于1500时，是增稳控制状态
	//g.rc_5.set_pwm(1600);//rc_5大于1500时，是增稳控制状态
	g.rc_5.set_pwm(1990);//rc_5大于1900时，是绕航点飞行状态

	/* 2--更新姿态，获取飞机现在的姿态角 */
	imu.update();
	compass.read();
	//这里本来应该有一个获取gps数据的，但是100hz更新gps数据没有太大的意义，并且在程序中我们需要用gps来计算实际的朝东和朝北的速度，
	//如果gps更新太快，导致每次更新gps时，老的经纬度和新的经纬度几乎是一样的
	//导致计算的actual_speed是0
	//update_GPS();//gps read只是读取数据 update_GPS里面还需要给current_loc赋值//20170919放在这里太快了,还是放在10hz的里面好点
	/*
	 * 因为下面的ahrs中需要imu gps compass的数据，
	 * 所以需要先读取那些传感器的数据
	 */
	ahrs.update_DCM(G_Dt);//20170920目前ahrs更新时还没有用drift_correction，也就是没有用gps的数据，但是后面可能是要加上的

	/* 3--update_current_flight_mode 更新控制状态，从而选择控制方式
	 * 设置yaw_mode roll_pitch_mode throttle_mode的模式
	 * 然后update_roll_pitch_mode，update_yaw_mode，update_throttle_mode要用
	 */
	update_current_flight_mode();

	/* 4--把期望的roll pitch yaw作用于飞机 */
	switch(control_mode)
	{
	case STABILIZE:
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
	    update_rate_contoller_targets();//这个步骤很重要，是把上面的earth坐标系下的计算数值量转为机体坐标系下的

	    //这个是执行了角速度的控制器，需要从ahrs或者imu获取角速度的大小，扩大了100倍，这个函数还得看一下
		run_rate_controllers();

		//这个是油门的控制，跟姿态的控制分开，油门的更新速率不需要那么快，油门的更新放在了medium_loop中了，5分之1的loop的频率，如果是50hz的话，那么就是10hz，100ms更新一次
		//update_throttle_mode();//计算油门量的输出值
		break;
	case ACRO:
		// call rate controllers
		g.channel_roll.servo_out = g.channel_roll.control_in;
		g.channel_pitch.servo_out = g.channel_pitch.control_in;
		g.channel_rudder.servo_out = g.channel_rudder.control_in;

		g.channel_throttle.servo_out=g.channel_throttle.control_in;
		break;
	case AUTO:
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

		//这个是油门的控制，跟姿态的控制分开，油门的更新放在了medium_loop中了，5分之1的loop的频率，如果是50hz的话，那么就是10hz，100ms更新一次
		//update_throttle_mode();//计算油门量的输出值
		break;
	default:
		break;
	}

	/* 5--把计算所得控制量输出给电机 */
	motors_output();

	if(takeoff_complete == false)
	{
		//没有起飞之前，把所有的积分项都清零
		// reset these I terms to prevent awkward tipping on takeoff
		reset_rate_I();
		reset_stability_I();
	}

#ifdef LINUX_OS
	/*
	 * 其实到这里主程序也就结束了，下面的程序主要是为了模拟仿真，把数据发送给flightgear
	 */

	/*
	 * motors_output()函数中把最终输出给radio_out的数值，
	 * 也就是每个电机的1000～2000的值赋值给了motor_out_flightgear
	 * 所以motor_out_flightgear就是飞行动力模型fdm的4个输入量
	 */
	servos_set_out[0]=motor_out_flightgear[0];
	servos_set_out[1]=motor_out_flightgear[1];
	servos_set_out[2]=motor_out_flightgear[2];
	servos_set_out[3]=motor_out_flightgear[3];

	/*
	 * 注意这里的input.servos是uint_16类型的，无符号short型，
	 * 所以servos_set_out一定不要有负数，所以需要提前处理servos_set_out
	*/
	memcpy(input.servos,servos_set_out,sizeof(servos_set_out));

	multi_copter.update(input);//利用input更新，copter四旋翼的位置，速度，线加速度，角度，角速度，角加速度是没有的，所以一共3*5=15个数据
	multi_copter.fill_fdm_flightgear(fdm);//现在的fdm中的数值就是四旋翼飞行动力模型的各个飞行状态15个数据

	memcpy(&fdm_feed_back,&fdm,sizeof(fdm));//fdm_feed_back被作为了估计出来的飞行状态，用来模拟用的，实际使用中这些数据都需要从all_external_device_output获取

//	std::cout<<"fdm_feed_back.phidot="<<fdm_feed_back.phidot<<std::endl;
//	std::cout<<"fdm_feed_back.thetadot="<<fdm_feed_back.thetadot<<std::endl;
//	std::cout<<"fdm_feed_back.psidot="<<fdm_feed_back.psidot<<std::endl;
#endif

#ifdef LINUX_OS
	memcpy(&fdm_send,&fdm,sizeof(fdm));//fdm_send是为了把这个飞行状态发送给flightgear使得flightgear能够三维动态显示飞行状态

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
#endif

#ifdef LINUX_OS
	sendto(fd_sock_send, &fdm_send, sizeof(fdm_send), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));
#endif

#ifdef LINUX_OS
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
#endif
}

void Copter::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
#if 0
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
#endif
}

void Copter::update_current_flight_mode(void)
{
	if(g.rc_5.radio_in>1000&&g.rc_5.radio_in<1500)
	{
		control_mode=ACRO;
	}
	else if(g.rc_5.radio_in>1500 && g.rc_5.radio_in<1900)
	{
		control_mode=STABILIZE;
	//	std::cout<<"飞控模式是增稳模式:"<<std::endl;
	}
	else if(g.rc_5.radio_in>1900)
	{
		control_mode=AUTO;
		//std::cout<<"飞控模式是绕航点飞行:"<<std::endl;
	}

	/*
	 * 根据飞行模式决定控制yaw roll pitch throttle的模式
	 */
	switch(control_mode)
	{
	case ACRO:
		yaw_mode                = YAW_ACRO;
		roll_pitch_mode = ROLL_PITCH_ACRO;
		throttle_mode   = THROTTLE_MANUAL;
		break;
	case STABILIZE:
		yaw_mode                = YAW_HOLD;
		roll_pitch_mode = ROLL_PITCH_STABLE;
		throttle_mode   = THROTTLE_MANUAL;
		break;
	case AUTO:
		yaw_mode                = YAW_AUTO;
		roll_pitch_mode = ROLL_PITCH_AUTO;
		throttle_mode   = THROTTLE_AUTO;

		// loads the commands from where we left off
		//init_commands();
		break;
	case LOITER:
//		yaw_mode                = LOITER_YAW;
//		roll_pitch_mode = LOITER_RP;
//		throttle_mode   = LOITER_THR;
//		set_next_WP(&current_loc);
		break;
	default:
		break;
	}
}

void Copter::stabilize()
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

// called after a GPS read
void Copter::update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

    // wp_distance is in CM
    // --------------------
    switch(control_mode) {
    case AUTO:
        // note: wp_control is handled by commands_logic
    	DEBUG_PRINTF("update_navigation    :    verify_commands()\n");
       verify_commands();

        // calculates desired Yaw
        update_auto_yaw();//直接得到auto_yaw,顺便有经纬度经过左边转换计算，得到nav_roll nav_pitch，这个就是经纬度转为期望roll 和 pitch的函数，要记得很重要

        // calculates the desired Roll and Pitch
        update_nav_wp();//由update_auto_yaw计算得到的nav_roll nav_pitch，在经过该函数的计算，得到auto_roll auto_pitch

        DEBUG_PRINTF("update_navigation    :    auto_yaw = %d\n",auto_yaw);
        DEBUG_PRINTF("update_navigation    :    auto_roll = %d\n",auto_roll);
        DEBUG_PRINTF("update_navigation    :    auto_pitch = %d\n",auto_pitch);
        break;
    case STABILIZE:
        update_nav_wp();
        break;
    }
 }



//#ifndef constrain
//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#endif
// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{

}

void Copter::update_alt()
{
	static int16_t 	old_sonar_alt 	= 0;
	static int32_t 	old_baro_alt 	= 0;
	static int16_t 	old_climb_rate 	= 0;

	// read in Actual Baro Altitude
	baro_alt 			= (baro_alt + read_barometer()) >> 1;

	// calc the vertical accel rate
	int temp			= (baro_alt - old_baro_alt) * 10;//为什么乘以10，其实应该是除以0.1，因为周期是10hz，0.1秒，所以爬升律就等于这个
	baro_rate 			= (temp + baro_rate) >> 1;
	old_baro_alt		= baro_alt;

	// NO Sonar case
	current_loc.alt = baro_alt + home.alt;
	climb_rate 		= baro_rate;

	// simple smoothing
	climb_rate = (climb_rate + old_climb_rate)>>1;

	// manage bad data
	climb_rate = constrain(climb_rate, -300, 300);

	// save for filtering
	old_climb_rate = climb_rate;

	// update the target altitude
	next_WP.alt = get_new_altitude();
}

void Copter::update_GPS(void)
{
	gps.read();

	current_loc.lng = gps.longitude;	// Lon * 10 * *7
	current_loc.lat = gps.latitude;	// Lat * 10 * *7

	//20170917
	//其实这里当gps信号稳定且状态可行时，就需要告诉导航级别，可以导航
	//否则就不导航
	nav_ok=true;

}

// uses the yaw from the DCM to give more accurate turns
void Copter::calc_bearing_error()
{

}


void Copter::update_auto_yaw()
{
    if(wp_control == CIRCLE_MODE) {
       // auto_yaw = get_bearing_cd(&current_loc, &circle_WP);

    }else if(wp_control == LOITER_MODE) {
        // just hold nav_yaw

    }else if(yaw_tracking == MAV_ROI_LOCATION) {
        auto_yaw = get_bearing_cd(&current_loc, &target_WP);

    }else if(yaw_tracking == MAV_ROI_WPNEXT) {
        // Point towards next WP
    	/*
    	 * 朝着航点飞行就是这个了
    	 */
    	//std::cout<<"original_target_bearing="<<original_target_bearing<<std::endl;
        //auto_yaw = original_target_bearing;//original_target_bearing是在set_wp_next的时候的目标航点的方位bearing
        auto_yaw = target_bearing;//这个是2.3版本的
    	DEBUG_PRINTF("update_auto_yaw    :    auto_yaw = %d\n",auto_yaw);
    }
}

// Outputs Nav_Pitch and Nav_Roll
void Copter::update_nav_wp()
{
    if(wp_control == LOITER_MODE) {

        // calc error to target
        //calc_location_error(&next_WP);

        // use error as the desired rate towards the target
   //     calc_loiter(long_error, lat_error);

    }else if(wp_control == CIRCLE_MODE) {

   }else if(wp_control == WP_MODE) {
	   /*
	    * 这个是我们需要的航点飞行时，把误差转换为
	    * 20170821
	    * 一开始由于next_WP都是0,所以肯定计算结果是错误的,但是update_commands很快就会执行,那么next_WP就会计算出来,从而很快就正确了
	    */
	   DEBUG_PRINTF("update_nav_wp    :    wp_control == WP_MODE\n");

        // calc error to target
        calc_location_error(&next_WP);//这个对于航点飞行来说没什么用，但是可以打印出来看看经度纬度差多少

        //speed的单位是cm每秒,最小是100cm/s最大是600cm/s
        int16_t speed = get_desired_speed(g.waypoint_speed_max, slow_wp);//这个是位置环计算机头方向的最大速度，根据wp_distance来计算
        // use error as the desired rate towards the target
        calc_nav_rate(speed);//这个是位置环的2级pid控制，但是有两个方向，1是机头方向速度，2是向左右侧滑的速度靠近航线去，max_speed方向上的，也就是机头方向，如果要压航线，还需要cross_speed
        calc_loiter_pitch_roll();//这个函数可能还有点问题20170820,这个函数是由nav_roll nav_pitch得到auto_roll auto_pitch

    }else if(wp_control == NO_NAV_MODE) {
        // clear out our nav so we can do things like land straight down
        // or change Loiter position

        // We bring copy over our Iterms for wind control, but we don't navigate
        nav_lon = g.pid_loiter_rate_lon.get_integrator();
        nav_lat = g.pid_loiter_rate_lon.get_integrator();

   //     nav_lon                 = constrain(nav_lon, -2000, 2000);                              // 20°
      //  nav_lat                 = constrain(nav_lat, -2000, 2000);                              // 20°
    }
}

void Copter::update_trig(void){
	Vector2f yawvector;
	Matrix3f temp 	= ahrs.get_dcm_matrix();

	yawvector.x 	= temp.a.x; // sin
	yawvector.y 	= temp.b.x;	// cos
	yawvector.normalize();


	sin_pitch_y 	= -temp.c.x;						// level = 0
	cos_pitch_x 	= sqrt(1 - (temp.c.x * temp.c.x));	// level = 1

	sin_roll_y 		= temp.c.y / cos_pitch_x;			// level = 0
	cos_roll_x 		= temp.c.z / cos_pitch_x;			// level = 1

	sin_yaw_y 		= yawvector.x;						// 1y = north
	cos_yaw_x 		= yawvector.y;						// 0x = north

	//flat:
	// 0 ° = cos_yaw:  0.00, sin_yaw:  1.00,
	// 90° = cos_yaw:  1.00, sin_yaw:  0.00,
	// 180 = cos_yaw:  0.00, sin_yaw: -1.00,
	// 270 = cos_yaw: -1.00, sin_yaw:  0.00,
}

void Copter::medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS and Compass
		//-----------------------------------------
		case 0:
			medium_loopCounter++;

			//20170930这里更新gps还是太快了，所以我用了下面的gps_cnt计数
			//if(GPS_enabled){
			//	update_GPS();
			//}
			static int gps_cnt;
			gps_cnt++;
			if(gps_cnt>10)
			{
				update_GPS();
				gps_cnt=0;
			}


			if(g.compass_enabled)
			{
				//if (compass.read())
				{
					//compass.calculate(dcm.get_dcm_matrix());  	// Calculate heading
					//compass.null_offsets(dcm.get_dcm_matrix());
				}
			}

			// record throttle output
			// ------------------------------
			throttle_integrator += g.channel_throttle.servo_out;
			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;

			// Auto control modes:
			if(nav_ok){
				// clear nav flag
				nav_ok = false;

				// calculate the copter's desired bearing and WP distance
				// ------------------------------------------------------
				if(navigate()){
					/*
					 * 程序开始运行时,navigate计算的wp_distance     = get_distance(&filtered_loc, &next_WP);
					 * 肯定是非常大的,因为next_WP是(0,0)也就是经度为0,纬度也为0,所以下面的update_navigation
					 * 中的verify_commands();肯定返回的也是fasle也就是没有到达,
					 * 直到update_commands更新,得到下一个航点的位置,所以实际上update_commands是先有效执行的
					 */

					// this calculates the velocity for Loiter
					// only called when there is new data
					// ----------------------------------
					calc_XY_velocity();

					//20170930通过光流应该能够更好的定位，但是基本的飞控是不需要的，删除掉
					// If we have optFlow enabled we can grab a more accurate speed
					// here and override the speed from the GPS
					// ----------------------------------------
					//#ifdef OPTFLOW_ENABLED
					//if(g.optflow_enabled && current_loc.alt < 500){
					//	// optflow wont be enabled on 1280's
					//	x_GPS_speed 	= optflow.x_cm;
					//	y_GPS_speed 	= optflow.y_cm;
					//}
					//#endif

					// control mode specific updates
					// -----------------------------
					update_navigation();

					/*
					if (g.log_bitmask & MASK_LOG_NTUN)
						Log_Write_Nav_Tuning();
						*/
				}
			}
			break;

		// command processing
		//-------------------
		case 2:
			medium_loopCounter++;

			// Read altitude from sensors
			// ------------------
			update_alt();

			// invalidate the throttle hold value
			// ----------------------------------
			invalid_throttle = true;//为什么把油门无效了呢

			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;

			// perform next command
			// --------------------
			if(control_mode == AUTO){
				if(home_is_set == true && g.command_total > 1){
					update_commands();
				}
			}

			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter = 0;

			gcs_update();//发送参数包和航点包

			if (g.battery_monitoring != 0){
				//read_battery();
			}

			// Do an extra baro read读取气压计数据 不要删除20170930
			// ---------------------
			//barometer.read();

			// agmatthews - USERHOOKS
			#ifdef USERHOOK_MEDIUMLOOP
			   USERHOOK_MEDIUMLOOP
			#endif

			slow_loop();
			break;

		default:
			// this is just a catch all
			// ------------------------
			medium_loopCounter = 0;
			break;
	}
}

void Copter::slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
		//----------------------------------------
		switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			superslow_loopCounter++;

			/*
			 * 下面这个是更新记录磁力计的偏移量,后期还是得用，不要删除
			 */
			if(superslow_loopCounter > 1200){
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled){
						compass.save_offsets();
						superslow_loopCounter = 0;
					}
				#endif
			}
			break;

		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			//read_control_switch();

			// agmatthews - USERHOOKS
			#ifdef USERHOOK_SLOWLOOP
			   USERHOOK_SLOWLOOP
			#endif

			break;

		case 2:
			slow_loopCounter = 0;

			//更新发生故障保护的事件驱动，不要删除
			//update_events();

			//灯光闪烁，不要删除
			// blink if we are armed
			//update_lights();

			// send all requested output streams with rates requested
			// between 3 and 5 Hz
			//gcs_data_stream_send(3,5);

			break;

		default:
			slow_loopCounter = 0;
			break;
	}

}


// stuff that happens at 50 hz
// ---------------------------
void Copter::fifty_hz_loop()
{
	// moved to slower loop
	// --------------------
	invalid_throttle=true;
	update_throttle_mode();

	//声纳
	// Read Sonar
	// ----------

	//光流
	// syncronise optical flow reads with altitude reads
	#ifdef OPTFLOW_ENABLED
	if(g.optflow_enabled){
		update_optical_flow();
	}
	#endif

	//用户钩子函数，也就是在程序运行之前的回调函数
	// agmatthews - USERHOOKS
	#ifdef USERHOOK_50HZLOOP
	  USERHOOK_50HZLOOP
	#endif

	  //相机云台的稳定控制
	//camera_stabilization();

	  /*
	   * 20170922//接收来自地面站的数据，更新命令
	   */
	// kick the GCS to process uplink data
	//gcs_update();//20170923发现使用这个函数后因为读取一个字节函数是有时间延时的，阻塞了，那么程序就不运行了，读取那里还需要更改
	  //上面的gcs_update()中的更新函数，其实是从串口读取数据然后做解析，所以我习惯性的放在了串口的接收线程中，不再在这里运行，而gcs_update函数中只保留了发送参数包和航点包的，把接收删掉了
	  // //gcs_data_stream_send(45,1000);
	gcs_data_stream_send();//gcs_data_stream_send()这个函数是发送实时数据的，也就是发送除了参数包，航点包等的，优先级要低于gcs_update

}

void Copter::update_all_external_device_input( void )
{
	//如果跟王正阳的硬件驱动一起调试，则这个函数是不需要的，硬件驱动把数据赋值给all_external_device_input即可

#ifdef LINUX_OS
	/*
	 * gps数据 gps数据更新最多也就是10hz，所以这里只是赋值，到底gps的值在哪里用呢，是在medium_loop中调用的
	 */
	all_external_device_input.latitude    =    (fdm_feed_back.latitude *RAD_TO_DEG)*1e7;
	all_external_device_input.longitude =    (fdm_feed_back.longitude *RAD_TO_DEG)*1e7;
	all_external_device_input.altitude    =   (fdm_feed_back.altitude )*1e2;
	all_external_device_input.v_north    =    fdm_feed_back.v_north;
	all_external_device_input.v_east    =    fdm_feed_back.v_east;
	all_external_device_input.v_down    =    fdm_feed_back.v_down;

	/*
	 * imu的数据
	 */
	all_external_device_input._accel_x    =    fdm_feed_back.A_X_pilot;
	all_external_device_input._accel_y    =    fdm_feed_back.A_Y_pilot;
	all_external_device_input._accel_z    =    fdm_feed_back.A_Z_pilot;
	all_external_device_input._gyro_x    =    fdm_feed_back.phidot;
	all_external_device_input._gyro_y    =    fdm_feed_back.thetadot;
	all_external_device_input._gyro_z    =    fdm_feed_back.psidot;
#endif
	/*
	 * 这里应该是获取遥控器的信号
	 */
	all_external_device_input.rc_raw_in_0    =    1500;
	all_external_device_input.rc_raw_in_1    =    1500;
	all_external_device_input.rc_raw_in_2    =    1500;
	all_external_device_input.rc_raw_in_3    =    1500;
	all_external_device_input.rc_raw_in_4    =    1990;//绕航点飞行模式
	all_external_device_input.rc_raw_in_5    =    1500;
	all_external_device_input.rc_raw_in_6    =    1500;
	all_external_device_input.rc_raw_in_7    =    1500;
	all_external_device_input.rc_raw_in_8    =    1500;


	/*
	 * 上面的all_external_device_input其实应该是由外部设备有数据更新后把数据
	 * 赋值给all_external_device_input，而我的飞控只是从这里获取数据，不用管数据是否更新
	 * 而且我只是从这里读取数据，应该不会出现同时写某一个变量的情况
	 * 上面的这些赋值应该是由王正阳从设备驱动那里获取数据值
	 * 实际使用时，上面的需要删除掉我这里并不需要
	 * 我需要的是下面的从all_external_device_input获取数据
	 */
}
void Copter::update_mavlink_reatime()
{
	ap2gcs_mavlink.attitude_roll_rad=ahrs.roll;
	ap2gcs_mavlink.attitude_pitch_rad=ahrs.pitch;
	ap2gcs_mavlink.attitude_yaw_rad=ahrs.yaw;
	ap2gcs_mavlink.attitude_roll_speed=imu._gyro.x;
	ap2gcs_mavlink.attitude_pitch_speed=imu._gyro.y;
	ap2gcs_mavlink.attitude_yaw_speed=imu._gyro.z;

	ap2gcs_mavlink.global_position_lat=(int32_t)gps.latitude;
	ap2gcs_mavlink.global_position_lon=-(int32_t)gps.longitude;//这里正常情况下是没有负号的，我因为用flightgear模拟，定的机场是-122度，所以我加了负号，把他变为122度，山东的附近
	ap2gcs_mavlink.global_position_alt=(int32_t)gps.altitude * 10;//gps.altitude的高度是厘米，但是发送给地面站的是毫米
	ap2gcs_mavlink.global_position_relative_alt=(int32_t)gps.altitude * 10;
	ap2gcs_mavlink.global_position_vx=fdm_feed_back.v_north * 100;
	ap2gcs_mavlink.global_position_vy=fdm_feed_back.v_east * 100;
	ap2gcs_mavlink.global_position_vz=fdm_feed_back.v_down * 100;
	ap2gcs_mavlink.global_position_hdg=0;

}


#define AUTO_ARMING_DELAY 60
// 1Hz loop
void Copter::super_slow_loop()
{
	//王博20170917这个函数里面最重要的就是1超过30秒上锁，使得电机无法转动，2发送心跳包给地面站，当油门一直处在0位置时，超过30秒就上锁，需要解锁
#if 0
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

	// this function disarms the copter if it has been sitting on the ground for any moment of time greater than 30s
	// but only of the control mode is manual
	if((control_mode <= ACRO) && (g.rc_3.control_in == 0)){
		auto_disarming_counter++;
		if(auto_disarming_counter == AUTO_ARMING_DELAY){
			init_disarm_motors();
		}else if (auto_disarming_counter > AUTO_ARMING_DELAY){
			auto_disarming_counter = AUTO_ARMING_DELAY + 1;
		}
	}else{
		auto_disarming_counter = 0;
	}
#endif

    //gcs_send_message(MSG_HEARTBEAT);
    //gcs_data_stream_send(1,3);

    gcs_send_message(MSG_HEARTBEAT);
    //gcs_update();

	// agmatthews - USERHOOKS
	#ifdef USERHOOK_SUPERSLOWLOOP
	   USERHOOK_SUPERSLOWLOOP
	#endif

	   //20170930看下面打印还有一个throttle_cruise巡航油门，我也得设置一个这个参数
	/*
	Serial.printf("alt %d, next.alt %d, alt_err: %d, cruise: %d, Alt_I:%1.2f, wp_dist %d, tar_bear %d, home_d %d, homebear %d\n",
					current_loc.alt,
					next_WP.alt,
					altitude_error,
					g.throttle_cruise.get(),
					g.pi_alt_hold.get_integrator(),
					wp_distance,
					target_bearing,
					home_distance,
					home_to_copter_bearing);
	*/
}

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
Copter copter;
#ifdef LINUX_OS
mavlink_system_t mavlink_system;
#endif
struct Location wp_total_array_temp[255]={0};
struct T_MAVLINK_REALTIME_DATA ap2gcs_mavlink;












/*
 * 下面的按道理跟飞控就没什么关系了，主要是为了接收串口来的数据，然后进行解析
 */
#ifdef LINUX_OS
/*
 * 通过串口通信，发送实时数据给无人船地面站做测试用，主要是经纬度用来地面站上显示
 */
struct T_UART_DEVICE uart_device_ap2gcs;

#endif

#ifdef LINUX_OS
/*
 * 通过udp网络通信，发送给flightgear
 */
T_AP2FG  ap2fg;
T_AP2FG  ap2fg_send;

T_FG2AP fg2ap;

int fd_socket_generic;
//int16_t             motor_out_flightgear[AP_MOTORS_MAX_NUM_MOTORS];
#endif


/*
 * 20170923为了接收missionplanner的数据，这个函数很重要，虽然以后是需要把这个接收线程放在别的地方
 *
 */
#ifdef LINUX_OS
#include "mavlink.h"
#include "GCS.h"
#include "GCS_Mavlink.h"

int read_radio_data(unsigned char *recv_buf,unsigned int recv_len)
{
	/* 直接把读取到的数据拷贝到_buffer数组中 */
	static unsigned char _buffer[512];

	/* 这个帧头的8个字节，都保存下来，放在_frame_received里面 */
	//head1=0xaa;
	//head2=0x55;
	static int _pack_recv_len = 0;
	static int _pack_recv_cnt = 0;
	static unsigned char _sysid;
	static unsigned char _pack_recv_type;
	static unsigned char _pack_recv_com_link;
	static unsigned char _pack_recv_ack_req;

	/* 这个是帧尾的校验和 其实是2个字节 但是我们现在只用1个字节 */
	static unsigned char _checksum = 0;

	unsigned int i = 0;
	static unsigned char c;
	static unsigned char valid_len=5;

	static unsigned char _frame_received_cnt;
	static unsigned char _frame_received[512];//存放一帧的数据，从帧头到帧尾校验和都包括，完整的一帧
	static unsigned char _frame_checksum_len=2;



	memcpy(_buffer, recv_buf, recv_len);
#if 1
	DEBUG_PRINTF("radio data buf=#################################################################################################\n");
	for(i=0;i<recv_len;i++)
	{
		DEBUG_PRINTF("%x ",recv_buf[i]);
	}
	DEBUG_PRINTF("\n");
#endif

	// receive new packets
	static mavlink_message_t msg;
	static mavlink_status_t status;
	status.packet_rx_drop_count = 0;

	// process received bytes
	//因为comm_get_available(chan)返回值一直是1，所以这个循环是永久的
//	while(comm_get_available(chan))
//	{
//		//关键是这个函数需要从串口读取一个字节的数据，这个要改写以下底层
//		uint8_t c = (uint8_t)comm_receive_ch(chan);
//
//
//		// Try to get a new message
//		if (mavlink_parse_char(chan, c, &msg, &status)) {
//			printf("mavlink update :受到地面站数据，开始处理受到的msg，handleMessage\n");
//			copter.gcs0.mavlink_active = true;
//			copter.gcs0.handleMessage(&msg);
//		}
//	}

	// Update packet drops counter
//	copter.gcs0.packet_drops += status.packet_rx_drop_count;


	for (i = 0; i<recv_len; i++)
	{
		c = _buffer[i];
		// Try to get a new message
		if (mavlink_parse_char(0, c, &msg, &status)) {
			DEBUG_PRINTF("mavlink update :收到地面站数据，开始处理受到的msg，handleMessage\n");
			copter.gcs0.mavlink_active = true;
			copter.gcs0.handleMessage(&msg);
		}

	}

	copter.gcs0.packet_drops += status.packet_rx_drop_count;

	return 0;
}
#endif
