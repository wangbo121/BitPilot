/*
 * copter.cpp
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

#include "copter.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
 * 构造函数是没有返回值的
 */
//Copter::Copter(void)
//{
//
//}

void Copter::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
#if 0
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
#endif
}

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


/******************************************************/
/*****************/

int fd_ap2gcs;
struct T_UART_DEVICE uart_device_ap2gcs;

/*
 * 20180821发现初始化的纬度在先
 */
//MultiCopter multi_copter("37.6136,-122.357,10,0","x");//起始高度为10，yaw是0
MultiCopter multi_copter("37.6136,-122.357,10,0","+");//起始高度为10，yaw是0

//MultiCopter multi_copter;
T_FDM fdm;
T_FDM fdm_send;
T_FDM fdm_feed_back;//这个动力模型解算出来后，就把数据返回给imu，gps，compass等


int _input_flag=1;

T_GLOBAL  gblState;
T_AP2FG  ap2fg;
T_FG2AP fg2ap;
T_AP2FG  ap2fg_send;
T_AP2FG  ap2fg_send_test;
T_AP2FG  ap2fg_send_test_send;

T_AP2FG  ap2fg_recv;

struct AP2GCS_REAL ap2gcs;

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

Aircraft::sitl_input input;//这个是4个电机的输入，然后用于multi_copter.update(input)更新出飞机的飞行状态

int fd_socket_generic;
int16_t             motor_out_flightgear[AP_MOTORS_MAX_NUM_MOTORS];




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

	float pid_p_1=1.0;
	//第一级pid参数设置
	g.pi_stabilize_roll.set_kP(pid_p_1);
	g.pi_stabilize_roll.set_kI(0.0);
	g.pi_stabilize_roll.set_kD(0.0);

	g.pi_stabilize_pitch.set_kP(pid_p_1);
	g.pi_stabilize_pitch.set_kI(0.0);
	g.pi_stabilize_pitch.set_kD(0.0);

	g.pi_stabilize_yaw.set_kP(pid_p_1);
	g.pi_stabilize_yaw.set_kI(0.0);
	g.pi_stabilize_yaw.set_kD(0.0);


	//第2级pid参数设置


	float pid_p_2=5.0;

	g.pid_rate_roll.set_kP(pid_p_2);
	g.pid_rate_roll.set_kI(0.0);
	g.pid_rate_roll.set_kD(0.0);

	float pid_p_pitch=5.0;
	g.pid_rate_pitch.set_kP(pid_p_pitch);
	g.pid_rate_pitch.set_kI(0.0);
	g.pid_rate_pitch.set_kD(0.0);

	g.pid_rate_yaw.set_kP(pid_p_2);
	g.pid_rate_yaw.set_kI(0.0);
	g.pid_rate_yaw.set_kD(0.0);


	/*
	 * 导航用的pid
	 */
	float pid_p_3=2.0;
	g.pid_nav_lon.set_kP(pid_p_3);
	g.pid_nav_lon.set_kI(0.0);
	g.pid_nav_lon.set_kD(0.0);


	//g.pid_nav_lat.set_kP(pid_p_3);
	g.pid_nav_lat.set_kP(2.0);
	g.pid_nav_lat.set_kI(0.0);//又犯了复制没有改名字的错误，set_kI写成了set_kP，导致p又改为了0
	g.pid_nav_lat.set_kD(0.0);

	/*
	 * 油门控高
	 */
	float pid_p_alt=1;
	g.pi_alt_hold.set_kP(pid_p_alt);
	g.pi_alt_hold.set_kI(0.0);
	g.pi_alt_hold.set_kD(0.0);

	g.pid_throttle.set_kP(pid_p_alt);
	g.pid_throttle.set_kI(0.0);
	g.pid_throttle.set_kD(0.0);


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

#if LINUX_OS
	open_udp_dev(IP_SEND_TO, PORT_SENT_TO, PORT_RECEIVE);//发送fdm飞行动力模型给flightgear，从而能够呈现姿态等
	open_socket_udp_dev(&fd_socket_generic,"127.0.0.1",5056);//发送generic的协议给flightgear，从而能够螺旋桨能够旋转
#endif

	ap2fg.throttle0 = 0.2;
	ap2fg.throttle1 = 0.3;
	ap2fg.throttle2 = 0.4;
	ap2fg.throttle3 = 0.5;

	ap2fg_send_test.throttle0=0;
	ap2fg_send_test.throttle1=0;
	ap2fg_send_test.throttle2=0;
	ap2fg_send_test.throttle3=0;

	g.auto_slew_rate=AUTO_SLEW_RATE;


	cos_roll_x 	= 1;
	cos_pitch_x 	= 1;
	cos_yaw_x 		= 1;
	sin_pitch_y=0;
	sin_yaw_y=0;
	sin_roll_y=0;

	yaw_tracking = MAV_ROI_WPNEXT;
	wp_control =WP_MODE;


		wp_total_array[0].id=0;

		g.waypoint_speed_max=WAYPOINT_SPEED_MAX;

		/*
		 * 为了自行测试绕航点飞行，我先自行设置了初始的经度纬度如下
		 */
		long start_longtitude=-1223571928;
		long start_latitude=376135531;
//		long delta_lon=10012;
//		long delta_lat=10001;//大概有100米？
//
//		long delta_lon=10000000;
//		long delta_lat=10000000;



		int wp_num=10;

		//long delta_lon=5000;//111米  5000，然后半径设置微100米能够比较好仿真
		//long delta_lon=100000;
		long delta_lon=0;


		//long delta_lat=0;
		//long delta_lat=5000;
		//long delta_lat=100000;
		long delta_lat=9000;

//		long delta_lon=100000;//111米
//		long delta_lat=100000;

		//g.waypoint_radius=100;//100米
		g.waypoint_radius=10;//100米

		wp_total_array[0].id 	= MAV_CMD_NAV_WAYPOINT;
		wp_total_array[0].lng 	= start_longtitude;				// Lon * 10**7
		wp_total_array[0].lat 	= start_latitude;				// Lat * 10**7
		wp_total_array[0].alt 	= 0;							// Home is always 0

		current_loc.lng=start_longtitude;
		current_loc.lat=start_latitude;

		int alt_temp=5000;

		/*
		 * 这个是经度纬度都增加，也就是一直往北或者一直往东，减小经度
		 */
		for(int i=1;i<wp_num;i++)
		{
			wp_total_array[i].id 	= MAV_CMD_NAV_WAYPOINT;
			wp_total_array[i].lng 	= start_longtitude+delta_lon*i;				// Lon * 10**7
			//wp_total_array[i].lng 	= start_longtitude-delta_lon*i;				// Lon * 10**7
			//wp_total_array[i].lng 	= start_longtitude;				// Lon * 10**7
			wp_total_array[i].lat 	= start_latitude+delta_lat*i;				// Lat * 10**7
			wp_total_array[i].alt 	= alt_temp;							// Home is always 0
		}





		/*
		 * 这个是矩形，绕航线飞行
		 */
//		for(int i=1;i<wp_num;i++)
//		{
//			wp_total_array[i].id 	= MAV_CMD_NAV_WAYPOINT;
//
//			wp_total_array[i].alt 	= alt_temp;							// Home is always 0
//
//
//		}
//		wp_total_array[1].lng 	= start_longtitude+delta_lon*1;				// Lon * 10**7
//		wp_total_array[1].lat 	= start_latitude;				// Lat * 10**7
//
//		wp_total_array[2].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
//		wp_total_array[2].lat 	= start_latitude;				// Lat * 10**7
//
//		wp_total_array[3].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
//		wp_total_array[3].lat 	= start_latitude+delta_lat*1;				// Lat * 10**7
//
//		wp_total_array[4].lng 	= start_longtitude+delta_lon*2;				// Lon * 10**7
//		wp_total_array[4].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7
//
//
//		wp_total_array[5].lng 	= start_longtitude+delta_lon*1;				// Lon * 10**7
//		wp_total_array[5].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7
//
//		wp_total_array[6].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
//		wp_total_array[6].lat 	= start_latitude+delta_lat*2;				// Lat * 10**7
//
//		wp_total_array[7].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
//		wp_total_array[7].lat 	= start_latitude+delta_lat*1;				// Lat * 10**7
//
//		wp_total_array[8].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
//		wp_total_array[8].lat 	= start_latitude+delta_lat*0;				// Lat * 10**7
//
//		wp_total_array[9].lng 	= start_longtitude+delta_lon*0;				// Lon * 10**7
//		wp_total_array[9].lat 	= start_latitude+delta_lat*0;				// Lat * 10**7




//
//		gps.longitude=-1223571928;				// Lon * 10**7
//		gps.latitude=376135531;				// Lat * 10**7
//
//		gps.longitude=-1223570059;			// Lon * 10**7
//		gps.latitude=376135956;


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

		command_nav_queue.id = NO_COMMAND;
		//command_nav_index=0;
		command_nav_index=0;



		g.crosstrack_gain=1;

		g.channel_throttle.control_in=500;
		g.channel_throttle.servo_out=500;
		g.throttle_cruise=500;

		g.sonar_enabled=0;

		dTnav=100;
		g.crosstrack_gain=1;

#ifdef LINUX_OS
		//fd_ap2gcs=open_uart_dev("/dev/ttyUSB0");

		//open_uart_dev(UART_DEVICE_APGCS);
		uart_device_ap2gcs.uart_name=UART_DEVICE_APGCS;

		uart_device_ap2gcs.baudrate=UART_AP2GCS_BAUD;
		uart_device_ap2gcs.databits=UART_AP2GCS_DATABITS;
		uart_device_ap2gcs.parity=UART_AP2GCS_PARITY;
		uart_device_ap2gcs.stopbits=UART_AP2GCS_STOPBITS;

		uart_device_ap2gcs.uart_num=open_uart_dev(uart_device_ap2gcs.uart_name);

		//uart_device_ap2gcs.ptr_fun=read_radio_data;

		set_uart_opt( uart_device_ap2gcs.uart_name, \
				uart_device_ap2gcs.baudrate,\
				uart_device_ap2gcs.databits,\
				uart_device_ap2gcs.parity,\
				uart_device_ap2gcs.stopbits);

		//create_uart_pthread(&uart_device_radio);



		ap2gcs.lng=12235719;
		ap2gcs.lat=3761355;
		ap2gcs.alt=50;


		send_uart_data(uart_device_ap2gcs.uart_name, (char *)&ap2gcs,sizeof(ap2gcs));
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
	float timer;
	timer=clock_gettime_ms();
	//std::cout<<"timer="<<timer/1000<<std::endl;

	G_Dt=(timer-fast_loopTimer)/1000.0;//单位是秒[s]
	G_Dt=0.01;
	//std::cout<<"G_Dt="<<G_Dt<<std::endl;

	fast_loopTimer=timer;

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
	update_GPS();//gps read只是读取数据 update_GPS里面还需要给current_loc赋值
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
//		std::cout<<"Hello STABILIZE MODE"<<std::endl;
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
		//update_throttle_mode();//计算油门量的输出值
		break;
	case ACRO:
//		std::cout<<"Hello ACRO MODE"<<std::endl;
		// call rate controllers
		g.channel_roll.servo_out = g.channel_roll.control_in;
		g.channel_pitch.servo_out = g.channel_pitch.control_in;
		g.channel_rudder.servo_out = g.channel_rudder.control_in;

		g.channel_throttle.servo_out=g.channel_throttle.control_in;
		break;

	case AUTO:
		std::cout<<"Hello AUTO MODE 绕航点航行"<<std::endl;
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
		//update_throttle_mode();//计算油门量的输出值


		break;

	default:
		break;
	}

	/* 5--把计算所得控制量输出给电机 */
	motors_output();

//	std::cout<<"g.channel_roll.radio_out="<<g.channel_roll.radio_out<<std::endl;
//	std::cout<<"g.channel_pitch.radio_out="<<g.channel_pitch.radio_out<<std::endl;
//	std::cout<<"g.channel_throttle.radio_out="<<g.channel_throttle.radio_out<<std::endl;
//	std::cout<<"g.channel_rudder.radio_out="<<g.channel_rudder.radio_out<<std::endl;

	/*
	 * motors_output()函数中把motor_out[]，也就是每个电机的1000～2000的值赋值给了
	 */
	servos_set_out[0]=motor_out_flightgear[0];
	servos_set_out[1]=motor_out_flightgear[1];
	servos_set_out[2]=motor_out_flightgear[2];
	servos_set_out[3]=motor_out_flightgear[3];

	for(int i=0;i<4;i++)
	{
		//std::cout<<"servos_set_out"<<"["<<i<<"]="<<servos_set_out[i]<<std::endl;
	}

	/*
	 * 注意这里的input.servos是uint_16类型的，无符号short型，
	 * 所以servos_set_out一定不要有负数，所以需要提前处理servos_set_out
	*/
	memcpy(input.servos,servos_set_out,sizeof(servos_set_out));

	multi_copter.update(input);//利用input更新，copter四旋翼的位置，速度，线加速度，角度，角速度，角加速度是没有的，所以一共3*5=15个数据
	multi_copter.fill_fdm_flightgear(fdm);

	memcpy(&fdm_feed_back,&fdm,sizeof(fdm));

//	std::cout<<"fdm_feed_back.phidot="<<fdm_feed_back.phidot<<std::endl;
//	std::cout<<"fdm_feed_back.thetadot="<<fdm_feed_back.thetadot<<std::endl;
//	std::cout<<"fdm_feed_back.psidot="<<fdm_feed_back.psidot<<std::endl;

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

#ifdef LINUX_OS
	sendto(fd_sock_send, &fdm_send, sizeof(fdm_send), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));
#endif








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
#ifdef LINUX_OS
	send_socket_udp_data(fd_socket_generic, socket_udp_send, sizeof(ap2fg_send),"127.0.0.1",5506 );
#endif
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
//
//	std::cout<<"********准备输出pwm脉宽给电调***********"<<std::endl;

//	std::cout<<"g.channel_roll.pwm_out="<<g.channel_roll.pwm_out<<std::endl;
//	std::cout<<"g.channel_pitch.pwm_out="<<g.channel_pitch.pwm_out<<std::endl;
//	std::cout<<"g.channel_rudder.pwm_out="<<g.channel_rudder.pwm_out<<std::endl;
//
//	std::cout<<"g.channel_throttle.radio_out="<<g.channel_throttle.radio_out<<std::endl;

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
//		std::cout<<"motor_out["<<i<<"]="<<motor_out[i]<<std::endl;
		//sleep(1);
	}

	memcpy(motor_out_flightgear,motor_out,sizeof(motor_out));//模型fdm需要的四个输入量就是电机的1000-2000的信号量

	for(int i=0;i<4;i++)
	{
		//g._rc.output_ch_pwm(i,motor_out[i]);
		//或者用下面的motors也是可以的
		//motors.rc_write(i,motor_out[i]);
//		std::cout<<"motor_out_flightgear["<<i<<"]="<<motor_out_flightgear[i]<<std::endl;
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



#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
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
	    */

	   std::cout<<"wp_control == WP_MODE"<<std::endl;

        // calc error to target
        calc_location_error(&next_WP);//这个对于航点飞行来说没什么用，但是可以打印出来看看经度纬度差多少

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


Copter copter;
