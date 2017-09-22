/*
 * copter.cpp
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

#include "copter.h"

#ifdef LINUX_OS
//我们保留了fdm模拟部分，但是删除了把发送给无人船地面站和flightgear这2部分放在了最下面用#define LINUX_OS包含着
/*
 * 四旋翼的飞行动力模型输入是4个电机的1000-2000的pwm值，输出是飞行的15个状态
 * 20180821发现初始化的纬度在先
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
int16_t             motor_out_flightgear[AP_MOTORS_MAX_NUM_MOTORS];//这个不能删除，本来是输出给flightgear的但是这个1000-2000的量又得赋值给fdm模拟用的servos_set_out
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

	memcpy(&wp_total_array_temp,&wp_total_array,sizeof(wp_total_array));

	for(int i=0;i<wp_num;i++)
	{
	std::cout<<"wp_total_array["<<i<<"].lng="<<wp_total_array[i].lng<<std::endl;
	std::cout<<"wp_total_array["<<i<<"].lat="<<wp_total_array[i].lat<<std::endl;
	std::cout<<"wp_total_array["<<i<<"].alt="<<wp_total_array[i].alt<<std::endl;
	}

	init_home();

	std::cout<<"home.lng="<<home.lng<<std::endl;
	std::cout<<"home.lat="<<home.lat<<std::endl;

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
	global_bool_boatpilot.wp_total_num=wp_num;
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
	uart_device_ap2gcs.databits=UART_AP2GCS_DATABITS;
	uart_device_ap2gcs.parity=UART_AP2GCS_PARITY;
	uart_device_ap2gcs.stopbits=UART_AP2GCS_STOPBITS;

	uart_device_ap2gcs.uart_num=open_uart_dev(uart_device_ap2gcs.uart_name);

	uart_device_ap2gcs.ptr_fun=read_radio_data;

	set_uart_opt( uart_device_ap2gcs.uart_name, uart_device_ap2gcs.baudrate,
							uart_device_ap2gcs.databits, uart_device_ap2gcs.parity,
							uart_device_ap2gcs.stopbits);

	DEBUG_PRINTF("uart recvbuf 串口名字=%s\n",uart_device_ap2gcs.uart_name);

	create_uart_pthread(&uart_device_ap2gcs);
#endif
}

void Copter::loop_fast()
{
	/*
	 * 将来跟硬件驱动获取数据整合时,这个函数是不需要的,现在是模拟,所以才需要
	 */
	//20170918添加了all_external_device_input和output一直循环从驱动中获取数据，至于硬件驱动到底多大频率获取的我不管，我只是每次从这里获取数据
	update_all_external_device_input();


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

	G_Dt=0.01;//这个设置为0.01秒也就是100hz主要是为了跟sim_aircraft的速率一致，但是其实20ms(50hz)就够

	/* 1--读取接收机的信号，获取遥控器各个通道 */
	read_radio();

	//g.channel_rudder.set_pwm(1600);//这个set_pwm参数的范围是1000～2000
	//g.channel_pitch.set_pwm(1600);//这个set_pwm参数的范围是1000～2000，把pitch一直设置为1600，看能不能稳定在9度左右
	//g.rc_5.set_pwm(1400);//rc_5大于1500时，是增稳控制状态
	//g.rc_5.set_pwm(1600);//rc_5大于1500时，是增稳控制状态
	g.rc_5.set_pwm(1990);//rc_5大于1900时，是绕航点飞行状态

	/* 2--更新姿态，获取飞机现在的姿态角 */
	compass.read();
	//gps.read();
	//100hz更新gps数据没有太大的意义，并且在程序中我们需要用gps来计算实际的朝东和朝北的速度，如果gps更新太快，导致每次更新gps时，老的经纬度和新的经纬度是一样的
	//导致计算的actual_speed是0
	//update_GPS();//gps read只是读取数据 update_GPS里面还需要给current_loc赋值//20170919放在这里太快了,还是放在10hz的里面好点
	imu.update();
	/*
	 * 因为下面的ahrs中需要imu gps compass的数据，
	 * 所以需要先读取那些传感器的数据
	 */
	ahrs.update_DCM(G_Dt);//20170920目前ahrs更新时还没有用drift_correction，也就是没有用gps的数据，但是后面肯定是要加上的

	if(takeoff_complete == false)
	{
		// reset these I terms to prevent awkward tipping on takeoff
		reset_rate_I();
		reset_stability_I();
	}

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
	    update_rate_contoller_targets();//这个步骤很重要，是把上面的earth坐标系下的转为机体坐标系

	    //这个是执行了角速度的控制器，需要从ahrs或者imu获取角速度的大小，扩大了100倍，这个函数还得看一下
		run_rate_controllers();

		//这个是油门的控制，跟姿态的控制分开，油门的更新放在了medium_loop中了，5分之1的loop的频率，如果是50hz的话，那么就是10hz，100ms更新一次
		//update_throttle_mode();//计算油门量的输出值
		break;
	case ACRO:
		//std::cout<<"Hello ACRO MODE"<<std::endl;
		// call rate controllers
		g.channel_roll.servo_out = g.channel_roll.control_in;
		g.channel_pitch.servo_out = g.channel_pitch.control_in;
		g.channel_rudder.servo_out = g.channel_rudder.control_in;

		g.channel_throttle.servo_out=g.channel_throttle.control_in;
		break;

	case AUTO:
		//std::cout<<"Hello AUTO MODE 绕航点航行"<<std::endl;
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

#ifdef LINUX_OS
	/*
	 * 其实到这里主程序也就结束了，我本来想把下面的都放在#define LINUX_OS中的
	 * 但是ucos上将来是不是也需要模拟仿真呢，就先留着了20170920
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
	 * 什么时候是航点飞行模式呢
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

// write out the servo PWM values
// ------------------------------
void Copter::set_servos_4()
{

}
void Copter::motors_output()
{
	int16_t             motor_out[AP_MOTORS_MAX_NUM_MOTORS];

    //int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
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

	std::cout<<"g.channel_roll.radio_out="<<g.channel_roll.radio_out<<std::endl;
	std::cout<<"g.channel_pitch.radio_out="<<g.channel_pitch.radio_out<<std::endl;
	std::cout<<"g.channel_throttle.radio_out="<<g.channel_throttle.radio_out<<std::endl;
	std::cout<<"g.channel_rudder.radio_out="<<g.channel_rudder.radio_out<<std::endl;

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

#ifdef LINUX_OS
	memcpy(motor_out_flightgear,motor_out,sizeof(motor_out));//模型fdm需要的四个输入量就是电机的1000-2000的信号量
#endif

	for(int i=0;i<4;i++)
	{
		g._rc.output_ch_pwm(i,motor_out[i]);//把控制量输出到all_external_device_output
		//或者用下面的motors也是可以的
		//motors.rc_write(i,motor_out[i]);
		//std::cout<<"motor_out_flightgear["<<i<<"]="<<motor_out_flightgear[i]<<std::endl;
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







void Copter::one_second_loop()
{
	/*
	 * 一秒钟给地面站发送一组数据--实时数据
	 */
}



//void Copter::navigate()
//{
//	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
//	// ------------------------------------------------------------------------
//
//}

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
    	std::cout<<"verify_commands();"<<std::endl;
       verify_commands();

        // calculates desired Yaw
        update_auto_yaw();//直接得到auto_yaw,顺便有经纬度经过左边转换计算，得到nav_roll nav_pitch，这个就是经纬度转为期望roll 和 pitch的函数，要记得很重要

        // calculates the desired Roll and Pitch
        update_nav_wp();//由update_auto_yaw计算得到的nav_roll nav_pitch，在经过该函数的计算，得到auto_roll auto_pitch

        std::cout<<"update_navigation  auto_yaw="<<auto_yaw<<std::endl;
        std::cout<<"update_navigation  auto_roll="<<auto_roll<<std::endl;
        std::cout<<"update_navigation  auto_pitch="<<auto_pitch<<std::endl;

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


		// This is real life

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
        std::cout<<"auto_yaw="<<auto_yaw<<std::endl;
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

	   std::cout<<"wp_control == WP_MODE"<<std::endl;

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
	//Matrix3f temp 	= dcm.get_dcm_matrix();
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

			// auto_trim, uses an auto_level algorithm
//			auto_trim();

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
			// --------------------------
			#if HIL_MODE != HIL_MODE_ATTITUDE					// don't execute in HIL mode
			update_altitude();
			#endif

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

			/*
			 //下面主要是记录姿态的日志，控制的日志，电机的日志以及发送数据给地面站
            if(motor_armed){
                if (g.log_bitmask & MASK_LOG_ATTITUDE_MED)
                    Log_Write_Attitude();

                if (g.log_bitmask & MASK_LOG_CTUN)
                    Log_Write_Control_Tuning();
            }

				// send all requested output streams with rates requested
				// between 5 and 45 Hz
				gcs_data_stream_send(5,45);

			if (g.log_bitmask & MASK_LOG_MOTORS)
				Log_Write_Motors();
				*/

			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter = 0;

			if (g.battery_monitoring != 0){
				//read_battery();
			}

			// Accel trims 		= hold > 2 seconds
			// Throttle cruise  = switch less than 1 second
			// --------------------------------------------
			//read_trim_switch();

			// Check for engine arming
			// -----------------------
			//arm_motors();

			// Do an extra baro read
			// ---------------------
			#if HIL_MODE != HIL_MODE_ATTITUDE
			barometer.read();
			#endif

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
			 * 下面这个是更新记录磁力计的偏移量,后期还是得用
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
			//update_events();
#if 0
			这些不需要
			// blink if we are armed
			update_lights();

			// send all requested output streams with rates requested
			// between 3 and 5 Hz
			gcs_data_stream_send(3,5);

			if(g.radio_tuning > 0)
				tuning();
#endif
			#if MOTOR_LEDS == 1
				update_motor_leds();
			#endif

			#if USB_MUX_PIN > 0
			check_usb_mux();
			#endif
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

	  //接收来自地面站的数据，更新命令
	// kick the GCS to process uplink data
	//gcs_update();
    //gcs_data_stream_send(45,1000);

	  /*
	   * 20170922
	   */
	// kick the GCS to process uplink data
	gcs_update();
	gcs_data_stream_send();

}

void Copter::update_all_external_device_input( void )
{
	//如果跟王正阳的硬件驱动一起调试，则这个函数是不需要的，硬件驱动把数据赋值给all_external_device_input即可

#ifdef LINUX_OS
	/*
	 * gps数据
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

void Copter::gcs_update(void){
	//先放在这里，应该是重新建立一个文件gcs_mavlink.cpp放在这个源文件里
}

#define AUTO_ARMING_DELAY 60
// 1Hz loop
void Copter::super_slow_loop()
{
	//王博20170917这个函数里面最重要的就是1超过30秒上锁，使得电机无法转动，2发送心跳包给地面站
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

	// agmatthews - USERHOOKS
	#ifdef USERHOOK_SUPERSLOWLOOP
	   USERHOOK_SUPERSLOWLOOP
	#endif

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


















#ifdef LINUX_OS
/*
 * 通过串口通信，发送实时数据给无人船地面站做测试用，主要是经纬度用来地面站上显示
 */
int fd_ap2gcs;
struct T_UART_DEVICE uart_device_ap2gcs;
struct AP2GCS_REAL ap2gcs;

struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;
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




int generate_packet(unsigned char*dst_buf, unsigned char *src_buf,unsigned char len,\
                    unsigned int packet_cnt, unsigned char message_type,\
                    unsigned char commu_method, unsigned char ack_req)
{
    static unsigned char frame_head_len=8;
    static unsigned char frame_end_len=2;
    unsigned char packet[128];
    unsigned char checksum = 0;

    int i, j;
    int packet_data_len;

    packet[0] = 0xaa;
    packet[1] = 0x55;
    packet[2] = 76;//20170728统一定义为76字节，包含帧头帧尾
    packet_data_len = len;

    packet[3] = packet_cnt;

    packet[4] = 0x01;
    packet[5] = message_type;

    packet[6]=commu_method;
    packet[7]=ack_req;

    for (i = frame_head_len, j = 0; i < packet_data_len + frame_head_len; i++, j++)
    {
        packet[i] = src_buf[j];
    }

    for (i = 0; i < len + frame_head_len; i++)
    {
        checksum += packet[i];
    }

    i = len + frame_head_len;

    //20170728把数据包长度统一减少2个字节，为76个字节
    packet[i]=0;
    checksum=checksum+packet[i];
    packet[i+1] = (checksum & 0xFF);

    memcpy(dst_buf, packet, packet_data_len + frame_head_len + frame_end_len);
    //printf("打包后返回的字节长度=%d\n",packet_data_len + frame_head_len + frame_end_len);//20170729已测试

    /*返回总的发送字节数*/
    return packet_data_len + frame_head_len + frame_end_len;
}


#ifdef LINUX_OS
void Copter::send_realdata_to_gcs( void )
{
	 unsigned char buf_data[256];
	unsigned char buf_packet[256];
	int ret;
	static int real_cnt;
	real_cnt++;

#if 0
	std::cout<<"send ap2gcs current_loc="<<current_loc.lng<<std::endl;
	//ap2gcs.lng=current_loc.lng*1e-2;
	ap2gcs.lng=-current_loc.lng*1e-2;
	ap2gcs.lat=current_loc.lat*1e-2;
	ap2gcs.alt=current_loc.alt*1e-2;
#else

	//ap2gcs.lng=current_loc.lng*1e-2;
	ap2gcs.lng=-current_loc.lng*1e-2;
	ap2gcs.lat=current_loc.lat*1e-2;
	ap2gcs.alt=current_loc.alt*1e-2;

#endif
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
#endif



#ifdef LINUX_OS
#define RADIO_RECV_HEAD1  0
#define RADIO_RECV_HEAD2  1
#define RADIO_RECV_LEN  2
#define RADIO_RECV_CNT  3
#define RADIO_RECV_SYSID 4
#define RADIO_RECV_TYPE 5
#define RADIO_RECV_COM_LINK 6
#define RADIO_RECV_ACK_REQ 7
#define RADIO_RECV_DATA 8

#define RADIO_RECV_CHECKSUM0 9
#define RADIO_RECV_CHECKSUM1 10

static int radio_recv_state = 0;

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
	printf("radio data buf=\n");
	for(i=0;i<recv_len;i++)
	{
		printf("%0x ",recv_buf[i]);
	}
	printf("\n");
#endif

	for (i = 0; i<recv_len; i++)
	{
		c = _buffer[i];
		switch (radio_recv_state)
		{
		case RADIO_RECV_HEAD1:
			if (c == 0xaa)
			{
				radio_recv_state = RADIO_RECV_HEAD2;
				_checksum = c;
			}
			break;
		case RADIO_RECV_HEAD2:
			if (c == 0x55)
			{
				radio_recv_state = RADIO_RECV_LEN;
				_checksum += c;
			}
			else
			{
				_checksum = 0;
				radio_recv_state = RADIO_RECV_HEAD1;
				_frame_received_cnt=0;
			}
			break;
		case RADIO_RECV_LEN:
			_checksum += c;
			_pack_recv_len=c;

			_frame_received[0]=0xaa;
			_frame_received[1]=0x55;
			_frame_received[2]=c;//20170728这个是统一为76个字节了，可以打印出来看
			_frame_received_cnt=3;

			radio_recv_state = RADIO_RECV_CNT;
			break;
		case RADIO_RECV_CNT:
			_pack_recv_cnt = c;
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			radio_recv_state = RADIO_RECV_SYSID;
			break;
		case RADIO_RECV_SYSID:
			_sysid = c;
			if(0!=_sysid)
			{
				_checksum += c;

				_frame_received[_frame_received_cnt]=c;
				_frame_received_cnt++;

				radio_recv_state = RADIO_RECV_TYPE;
			}
			break;
		case RADIO_RECV_TYPE:
			_pack_recv_type = c;
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			radio_recv_state = RADIO_RECV_COM_LINK;
			break;
		case RADIO_RECV_COM_LINK:
			_pack_recv_com_link = c;
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			radio_recv_state = RADIO_RECV_ACK_REQ;
			break;
		case RADIO_RECV_ACK_REQ:
			_pack_recv_ack_req = c;
			_checksum += c;

			//_frame_received[7]=c;
			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			radio_recv_state = RADIO_RECV_DATA;
			break;
		case RADIO_RECV_DATA:
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			if (_frame_received_cnt >= _pack_recv_len-_frame_checksum_len)
			{
				//printf("checksum=%0x \n",_checksum);
				radio_recv_state = RADIO_RECV_CHECKSUM0;
			}
			break;
		case RADIO_RECV_CHECKSUM0:
			_checksum += c;

			_frame_received[_frame_received_cnt]=c;
			_frame_received_cnt++;

			radio_recv_state = RADIO_RECV_CHECKSUM1;
			break;
		case RADIO_RECV_CHECKSUM1:
			if (_checksum == c)
			{
				_frame_received[_frame_received_cnt]=c;//如果正常的话，这里的_frame_received_cnt应该等于76，包含帧头和帧尾
				_frame_received_cnt++;
				//printf("_frame_received_cnt=%d\n",_frame_received_cnt);

				global_bool_boatpilot.radio_recv_packet_cnt = _pack_recv_cnt;
				if(global_bool_boatpilot.radio_recv_packet_cnt_previous!=global_bool_boatpilot.radio_recv_packet_cnt)
				{
					switch (_pack_recv_type)
					{
					case COMMAND_GCS2AP_WAYPOINT:
						if (_frame_received_cnt == sizeof(gcs_ap_wp))
						{
							printf("正确接收到GCS_AP_WP数据包，且数据包数据长度与航点结构长度相同\n");
							memcpy(&gcs_ap_wp, _frame_received, _frame_received_cnt);
							global_bool_boatpilot.bool_get_gcs2ap_waypoint = TRUE;
							 decode_gcs2ap_waypoint(wp_data,&gcs_ap_wp);
							//global_bool_boatpilot.bool_gcs2ap_beidou=_pack_recv_com_link;
							//global_bool_boatpilot.send_ap2gcs_wp_req=_pack_recv_ack_req;
							//global_bool_boatpilot.gcs2ap_wp_cnt=_pack_recv_cnt;
							//global_bool_boatpilot.ap2gcs_wp_cnt=_pack_recv_cnt;
						}
						_checksum = 0;
						radio_recv_state = 0;
						_frame_received_cnt=0;
						break;
					case COMMAND_GCS2AP_CMD:
						if (_frame_received_cnt == sizeof(gcs2ap_cmd))
						{
							//printf("正确接收到GCS2AP_CMD数据包，且数据包数据长度与命令结构长度相同\n");//20170410已测试
							memcpy(&gcs2ap_cmd, _frame_received, _frame_received_cnt);
							global_bool_boatpilot.bool_get_gcs2ap_cmd = TRUE;
							decode_gcs2ap_cmd(&gcs2ap_radio_all, &gcs2ap_cmd);
							//global_bool_boatpilot.bool_gcs2ap_beidou=_pack_recv_com_link;//这次先不判断北斗，北斗和电台同时发送实时数据，同时接收并解析命令包
							//global_bool_boatpilot.send_ap2gcs_cmd_req=_pack_recv_ack_req;
							//global_bool_boatpilot.gcs2ap_cmd_cnt=_pack_recv_cnt;
							global_bool_boatpilot.ap2gcs_cmd_cnt=_pack_recv_cnt;
						}
						_checksum = 0;
						radio_recv_state = 0;
						_frame_received_cnt=0;
						break;

					default:
						break;
					}

				}
			}
			else
			{
				printf("电台--数据校验和错误，校验和是加和\n");
				_checksum = 0;
				radio_recv_state = 0;
				_frame_received_cnt=0;
			}
		}
	}

	return 0;


}
#endif
















