/*
 * GCS_Mavlink.cpp
 *
 *  Created on: 2017-9-18
 *      Author: wangbo
 */

/*
 * 20171001 GCS_Mavlink是与地面站通信的部分，非常重要，改变时必须小心对照，并且测试
 */

#include "copter.h"
#include "GCS_Mavlink.h"
#include <limits.h>

#ifdef LINUX_OS

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

//20170930我这里面的函数都是从apm3.3复制粘贴过来的，但是我又要参照apm2.6的一些函数来写

/*
 *  send a message on both GCS links，原来说的是2个地面站口，但是我这里先只用1个
 */
void Copter::gcs_send_message(enum ap_message id)
{
	DEBUG_PRINTF("Copter::gcs_send_message\n");
	gcs0.send_message(id);//gcs0 是 GCS_MAVLINK类定义的对象，所以从gcs0访问的函数应该都是在GCS_MAVLINK中声明定义过的
//
//    for (uint8_t i=0; i<num_gcs; i++) {
//        if (gcs[i].initialised) {
//            gcs[i].send_message(id);
//        }
//    }
}

void Copter::gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

void Copter::gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

/*
 * 20171001 下面增加copter send_xxx函数时，按照global.h文件中的
 * enum ap_message 这个结构的顺序增加，
 * 我已经增加了心跳，姿态，位置，当前目标航点号4个，剩下的慢慢增加
 */

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */
/*
 * 20171001wangbo内联函数的目的是为了解决程序中函数调用的效率问题，因为内联函数会把函数内的语句直接展开，而不是调用
 * 这样就省去了函数调用保存堆栈的空间和时间，但是我们下面这些函数都不允许内联，防止出错，非常重要，所以下面函数都是noinline
 */
NOINLINE void Copter::send_heartbeat(mavlink_channel_t chan)
{
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;//20171003 无人机的飞行状态 解锁还是上锁等
	//    uint8_t system_status = ap.land_complete ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
	//uint8_t system_status =MAV_STATE_ACTIVE;
	uint32_t custom_mode = control_mode;

	//    // set system as critical if any failsafe have triggered
	//    if (failsafe.radio || failsafe.battery || failsafe.gcs || failsafe.ekf)  {
	//        system_status = MAV_STATE_CRITICAL;
	//    }

	// work out the base_mode. This value is not very useful
	// for APM, but we calculate it as best we can so a generic
	// MAVLink enabled ground station can work out something about
	// what the MAV is up to. The actual bit values are highly
	// ambiguous for most of the APM flight modes. In practice, you
	// only get useful information from the custom_mode, which maps to
	// the APM flight mode and has a well defined meaning in the
	// ArduPlane documentation
	base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
	switch (control_mode) {
	case AUTO:
	case RTL:
	case LOITER:
	case GUIDED:
	case CIRCLE:
	//    case POSHOLD:
	//    case BRAKE:
	base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
	// note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
	// APM does in any mode, as that is defined as "system finds its own goal
	// positions", which APM does not currently do
	break;
	}

	// all modes except INITIALISING have some form of manual
	// override if stick mixing is enabled
	base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

	#if HIL_MODE != HIL_MODE_DISABLED
	base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	#endif
	// we are armed if we are not initialising
	if (motors.armed()) {//20171003 测试这个if进不来
	base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	//20171003 模拟仿真的时候 假设我们一直是解锁状态
	base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

	// indicate we have set a custom mode
	base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	//uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	//uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack( mavlink_system.sysid,mavlink_system.compid,&msg,\
															system_type,
															autopilot_type,
															base_mode,
															custom_mode,
															system_state);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef LINUX_OS
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	//uart0_send(buf, len);
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}

NOINLINE void Copter::send_attitude(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process



	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_attitude_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
													0,\
													ap2gcs_mavlink.attitude_roll_rad,\
													ap2gcs_mavlink.attitude_pitch_rad,\
													ap2gcs_mavlink.attitude_yaw_rad,\
													ap2gcs_mavlink.attitude_roll_speed,\
													ap2gcs_mavlink.attitude_pitch_speed,\
													ap2gcs_mavlink.attitude_yaw_speed);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	//20171001下面注释不要删除，查看一下uart0_send(buf, len);这个函数是apm哪个版本的函数
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	//uart0_send(buf, len);
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}


void NOINLINE Copter::send_location(mavlink_channel_t chan)
{
#if 0
	/*
	 * 20171001这里有个旋转矩阵，把gps的速度分解了，先别删除
	 */
    Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
    mavlink_msg_global_position_int_send(
	chan,
	current_loc.lat,
	current_loc.lng,
	current_loc.alt * 10,
	g_gps->ground_speed * rot.a.x,
	g_gps->ground_speed * rot.b.x,
	g_gps->ground_speed * rot.c.x);
#endif

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_global_position_int_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
																		ap2gcs_mavlink.time_boot_ms,\
																		ap2gcs_mavlink.global_position_lat,\
																		ap2gcs_mavlink.global_position_lon,\
																		ap2gcs_mavlink.global_position_alt,\
																		ap2gcs_mavlink.global_position_relative_alt,\
																		ap2gcs_mavlink.global_position_vx,\
																		ap2gcs_mavlink.global_position_vy,\
																		ap2gcs_mavlink.global_position_vz,\
																		ap2gcs_mavlink.global_position_hdg);


	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}

NOINLINE void Copter::send_extended_status1(mavlink_channel_t chan)
{
	uint32_t control_sensors_present = 0;
	uint32_t control_sensors_enabled;
	uint32_t control_sensors_health;

	// first what sensors/controllers we have
	control_sensors_present |= (1<<0); // 3D gyro present
	control_sensors_present |= (1<<1); // 3D accelerometer present
	if (g.compass_enabled) {
		control_sensors_present |= (1<<2); // compass present
	}
	control_sensors_present |= (1<<3); // absolute pressure sensor present
//	if (&gps != NULL && g_gps->status() == GPS::GPS_OK) {
//		control_sensors_present |= (1<<5); // GPS present
//	}
	control_sensors_present |= (1<<5); // GPS present//20171003先假设任何时候gps都是存在的
	control_sensors_present |= (1<<10); // 3D angular rate control
	control_sensors_present |= (1<<11); // attitude stabilisation
	control_sensors_present |= (1<<12); // yaw position
	control_sensors_present |= (1<<13); // altitude control
	control_sensors_present |= (1<<14); // X/Y position control
	control_sensors_present |= (1<<15); // motor control

	// now what sensors/controllers are enabled

	// first the sensors
	control_sensors_enabled = control_sensors_present & 0x1FF;

	// now the controllers
	control_sensors_enabled = control_sensors_present & 0x1FF;

	control_sensors_enabled |= (1<<10); // 3D angular rate control
	control_sensors_enabled |= (1<<11); // attitude stabilisation
	control_sensors_enabled |= (1<<13); // altitude control
	control_sensors_enabled |= (1<<15); // motor control

	switch (control_mode) {
	case AUTO:
	case RTL:
	case LOITER:
	case GUIDED:
	case CIRCLE:
	case POSITION:
		control_sensors_enabled |= (1<<12); // yaw position
		control_sensors_enabled |= (1<<14); // X/Y position control
		break;
	}

	// at the moment all sensors/controllers are assumed healthy
	control_sensors_health = control_sensors_present;

	uint16_t battery_current = -1;
	uint8_t battery_remaining = -1;

//	if (current_total1 != 0 && g.pack_capacity != 0) {
//		battery_remaining = (100.0 * (g.pack_capacity - current_total1) / g.pack_capacity);
//	}
//	if (current_total1 != 0) {
//		battery_current = current_amps1 * 100;
//	}
//
//	if (g.battery_monitoring == 3) {
//		/*setting a out-of-range value.
//		 *  It informs to external devices that
//		 *  it cannot be calculated properly just by voltage*/
//		battery_remaining = 150;
//	}

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	battery_remaining=100;//20171001电池剩余百分之百
	mavlink_msg_sys_status_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
														control_sensors_present,\
														control_sensors_enabled,\
														control_sensors_health,\
														0, // CPU Load not supported in AC yet
														battery_voltage1 * 1000, // mV
														battery_current,        // in 10mA units
														battery_remaining,      // in %
														0, // comm drops %,
														0, // comm drops in pkts,
														0, 0, 0, 0);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	//uart0_send(buf, len);
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif

}

void NOINLINE Copter::send_gps_raw(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
															ap2gcs_mavlink.gps_raw_time_usec,\
															ap2gcs_mavlink.gps_raw_fix_type,\
															ap2gcs_mavlink.gps_raw_lat,\
															ap2gcs_mavlink.gps_raw_lon,\
															ap2gcs_mavlink.gps_raw_alt,\
															ap2gcs_mavlink.gps_raw_eph,\
															ap2gcs_mavlink.gps_raw_epv,\
															ap2gcs_mavlink.gps_raw_vel,\
															ap2gcs_mavlink.gps_raw_cog,\
															ap2gcs_mavlink.gps_raw_satellites_visible);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}

void NOINLINE Copter::send_gps_status(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_gps_status_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
														ap2gcs_mavlink.gps_status_satellites_visible,
														NULL,
														NULL,
														NULL,
														NULL,
														NULL);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}
void NOINLINE Copter::send_raw_imu1(mavlink_channel_t chan)
{
//    Vector3f accel = imu.get_accel();//20171003我不在这里获取了，直接放在ap2gcs_mavlink更新的函数中获取
//    Vector3f gyro = imu.get_gyro();

    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_raw_imu_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
														ap2gcs_mavlink.raw_imu_time_usec,
														ap2gcs_mavlink.raw_imu_xacc,
														ap2gcs_mavlink.raw_imu_yacc,
														ap2gcs_mavlink.raw_imu_zacc,
														ap2gcs_mavlink.raw_imu_xgyro,
														ap2gcs_mavlink.raw_imu_ygyro,
														ap2gcs_mavlink.raw_imu_zgyro,
														ap2gcs_mavlink.raw_imu_xmag,
														ap2gcs_mavlink.raw_imu_ymag,
														ap2gcs_mavlink.raw_imu_zmag);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}
/*
 * 下面两个先不写了，只写上面一个imu的数据
 */
void NOINLINE Copter::send_raw_imu2(mavlink_channel_t chan)
{

}
void NOINLINE Copter::send_raw_imu3(mavlink_channel_t chan)
{

}





void NOINLINE Copter::send_nav_controller_output(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_nav_controller_output_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
																			ap2gcs_mavlink.nav_controller_output_nav_roll,
																			ap2gcs_mavlink.nav_controller_output_nav_pitch,
																			ap2gcs_mavlink.nav_controller_output_nav_bearing,
																			ap2gcs_mavlink.nav_controller_output_target_bearing,
																			ap2gcs_mavlink.nav_controller_output_wp_dist,
																			ap2gcs_mavlink.nav_controller_output_alt_error,
																			ap2gcs_mavlink.nav_controller_output_aspd_error,
																			ap2gcs_mavlink.nav_controller_output_xtrack_error);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}

// report simulator state
void NOINLINE Copter::send_simstate(mavlink_channel_t chan)
{

}

void NOINLINE Copter::send_hwstatus(mavlink_channel_t chan)
{

}

void NOINLINE Copter::send_servo_out(mavlink_channel_t chan)
{
#if HIL_MODE != HIL_MODE_DISABLED
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
#endif
#endif // HIL_MODE
}

void NOINLINE Copter::send_radio_in(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
			ap2gcs_mavlink.rc_channels_raw_time_boot_ms,
			ap2gcs_mavlink.rc_channels_raw_port,
			ap2gcs_mavlink.rc_channels_raw_chan1_raw,
			ap2gcs_mavlink.rc_channels_raw_chan2_raw,
			ap2gcs_mavlink.rc_channels_raw_chan3_raw,
			ap2gcs_mavlink.rc_channels_raw_chan4_raw,
			ap2gcs_mavlink.rc_channels_raw_chan5_raw,
			ap2gcs_mavlink.rc_channels_raw_chan6_raw,
			ap2gcs_mavlink.rc_channels_raw_chan7_raw,
			ap2gcs_mavlink.rc_channels_raw_chan8_raw,
			ap2gcs_mavlink.rc_channels_raw_rssi);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif

}

void NOINLINE Copter::send_radio_out(mavlink_channel_t chan)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_servo_output_raw_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
			ap2gcs_mavlink.servo_output_raw_time_usec,
			ap2gcs_mavlink.servo_output_raw_port,
			ap2gcs_mavlink.servo_output_raw_servo1_raw,
			ap2gcs_mavlink.servo_output_raw_servo2_raw,
			ap2gcs_mavlink.servo_output_raw_servo3_raw,
			ap2gcs_mavlink.servo_output_raw_servo4_raw,
			ap2gcs_mavlink.servo_output_raw_servo5_raw,
			ap2gcs_mavlink.servo_output_raw_servo6_raw,
			ap2gcs_mavlink.servo_output_raw_servo7_raw,
			ap2gcs_mavlink.servo_output_raw_servo8_raw);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif
}

void NOINLINE Copter::send_vfr_hud(mavlink_channel_t chan)
{
	/*
	 * 20171003 vfr : visula flight rules 目视飞行规则，飞机降落时需要参考的数据
	 */
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_vfr_hud_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
			ap2gcs_mavlink.vfr_hud_airspeed,
			ap2gcs_mavlink.vfr_hud_groundspeed,
			ap2gcs_mavlink.vfr_hud_heading,
			ap2gcs_mavlink.vfr_hud_throttle,
			ap2gcs_mavlink.vfr_hud_alt,
			ap2gcs_mavlink.vfr_hud_climb);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif






}

void NOINLINE Copter::send_current_waypoint(mavlink_channel_t chan)
{

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];


	mavlink_msg_mission_current_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
																	g.command_index);


	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef LINUX_OS
	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	//uart0_send(buf, len);
	send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
#endif


}

/*
  send RPM packet
 */
void NOINLINE Copter::send_rpm(mavlink_channel_t chan)
{

}


/*
  send PID tuning message
 */
void Copter::send_pid_tuning(mavlink_channel_t chan)
{

}


void NOINLINE Copter::send_statustext(mavlink_channel_t chan)
{

}


// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{

	int16_t *stream_rates = &streamRateRawSensors;
	uint8_t rate = (uint8_t)stream_rates[stream_num];//20171002必须非常注意这里，他这里用了类似于数组的方式，但是其实stream_rates并不是数组，只是streamRateRawSensors的地址，而这个streamRateRawSensors变量是在GCS.h中定义的

	//DEBUG_PRINTF("GCS_MAVLINK::stream_trigger    :    rate = %d\n",rate);
	//printf("GCS_MAVLINK::stream_trigger    :    rate = %d\n",rate);
	/*
	 * 因为并没有程序给streamRateRawSensors 以及 stream_rates赋值，所以一开始这里的stream_rates[stream_num] 必然都等于0
	 * 可是为什么一连接上地面站这个rate就不等于0了呢，我测试过了，一开始没有连接地面站时，rate的确一直是0
	 * 但是地面站在点击右上角的连接时，就会给驾驶仪发送MAVLINK_MSG_ID_REQUEST_DATA_STREAM: //66  从而给每个参数包设置频率freq
	 */
	if (rate == 0) {
		return false;
	}

	DEBUG_PRINTF("GCS_MAVLINK::stream_trigger    :    rate = %d\n",rate);

	if (stream_ticks[stream_num] == 0) {
		// we're triggering now, setup the next trigger point
		if (rate > 50) {
			rate = 50;
		}
		stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
		return true;
	}

	// count down at 50Hz
	stream_ticks[stream_num]--;
	return false;
}

/*
 * 20170922这个发送虽然是50hz的，但是发送有需要触发，
 * 而触发频率是handleMessage中设置的
 */
void
GCS_MAVLINK::data_stream_send(void)
{
    if (waypoint_receiving || waypoint_sending || (_queued_parameter != NULL) ) {
        // don't interfere with mission transfer
        return;
    }

//    if (!copter.in_mavlink_delay && !copter.motors.armed()) {
//        handle_log_send(copter.DataFlash);
//    }

//    copter.gcs_out_of_time = false;
//
//    if (_queued_parameter != NULL) {
//        if (streamRates[STREAM_PARAMS] <= 0) {
//            streamRates[STREAM_PARAMS]=10;
//        }
//        if (stream_trigger(STREAM_PARAMS)) {
//            send_message(MSG_NEXT_PARAM);
//        }
//        // don't send anything else at the same time as parameters
//        return;
//    }
//
//    if (copter.gcs_out_of_time) return;
//
//    if (copter.in_mavlink_delay) {
//        // don't send any other stream types while in the delay callback
//        return;
//    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);//2hz
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

   // if (copter.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);//2hz
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        //send_message(MSG_GPS_RAW);// TODO - remove this message after location message is working 后面location已经有了，这里不需要了
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        //send_message(MSG_LIMITS_STATUS);
    }

   // if (copter.gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);//3hz
        //send_message(MSG_LOCAL_POSITION);
    }

   // if (copter.gcs_out_of_time) return;

    //20171003 这个STREAM_RAW_CONTROLLER在连接地面站时貌似并没有设置她的发送频率
    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);//0hz
    }

    //if (copter.gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);//2hz
        send_message(MSG_RADIO_IN);
    }

    //if (copter.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);//10hz
        //send_message(MSG_SIMSTATE);
        //send_message(MSG_PID_TUNING);
    }

   // if (copter.gcs_out_of_time) return;

    //20171003VFR: visual flight rules 仪表飞行规则 靠眼睛进行着陆所需要的信息
    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);//10hz
    }

  //  if (copter.gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
//        send_message(MSG_AHRS);
//        send_message(MSG_HWSTATUS);
//        send_message(MSG_SYSTEM_TIME);
//        send_message(MSG_RANGEFINDER);
//        send_message(MSG_TERRAIN);
//        send_message(MSG_BATTERY2);
//        send_message(MSG_MOUNT_STATUS);
//        send_message(MSG_OPTICAL_FLOW);
//        send_message(MSG_GIMBAL_REPORT);
//        send_message(MSG_EKF_STATUS_REPORT);
//        send_message(MSG_VIBRATION);
//        send_message(MSG_RPM);
    }
}


//void GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
//{
//    copter.do_guided(cmd);
//}
//
//void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
//{
//    // add home alt if needed
//    if (cmd.content.location.flags.relative_alt) {
//        cmd.content.location.alt += copter.ahrs.get_home().alt;
//    }
//
//    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
//}

void Copter::gcs_update(void)
{
	//先放在这里，应该是重新建立一个文件gcs_mavlink.cpp放在这个源文件里


	gcs0.update();



}

void GCS_MAVLINK::update(void)
{
	// receive new packets
	mavlink_message_t msg;
	mavlink_status_t status;
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
//			mavlink_active = true;
//			handleMessage(&msg);
//		}
//	}

	 //if (_queued_parameter != NULL) {
	//20171002 修改这个判断成wang
	 if (_queued_parameter_wang != NULL) {
	if (streamRates[STREAM_PARAMS] <= 0) {
		streamRates[STREAM_PARAMS]=10;
	}
	//send_message(MSG_NEXT_PARAM);
	DEBUG_PRINTF("update    :    streamRates[STREAM_PARAMS]=%d\n",streamRates[STREAM_PARAMS]);//20171002测试是10
	if (stream_trigger(STREAM_PARAMS)) {
		send_message(MSG_NEXT_PARAM);
		//send_message(MSG_NEXT_PARAM);//20171002按照apm是放在这里的，我先放在外面测试一下，因为我发现参数多的时候，地面站获取全部参数总是出现错误中断
	}
	// don't send anything else at the same time as parameters
	return;
}

	// Update packet drops counter
	packet_drops += status.packet_rx_drop_count;

	/*
	 * 2.3的内容 参数包和航点包实际上在这里发送，handleMessage中对于参数和航点的动作只是把要发送的参数和航点放在了即将发送的循环丢列里
	 */
	 // send out queued params/ waypoints
//	if (NULL != _queued_parameter) {
//		send_message(MSG_NEXT_PARAM);
//	}


	/*
	 * 20170922下面的其实先不需要了，是关于航点于这个数据的互斥发送
	 */


	/*
	 * 如果不是接收航点或者发送航点，下面的代码就不执行，所以只有在接收航点或者发送航点时，send_message(MSG_NEXT_WAYPOINT);才会执行
	 */
	if (!waypoint_receiving && !waypoint_sending) {
		return;
	}

	uint32_t tnow = clock_gettime_ms();

//	if (waypoint_receiving &&
//		waypoint_request_i <= (unsigned)copter.g.command_total &&
//		tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
	if (waypoint_receiving &&
			waypoint_request_i <= (unsigned)copter.g.command_total) {
#if 0
		waypoint_timelast_request = tnow;
		send_message(MSG_NEXT_WAYPOINT);
#else
		if (tnow > waypoint_timelast_request + 500)
		{
			waypoint_timelast_request = tnow;
			send_message(MSG_NEXT_WAYPOINT);
		}
#endif
	}

//	// stop waypoint sending if timeout
//	if (waypoint_sending && (tnow - waypoint_timelast_send) > waypoint_send_timeout){
//		waypoint_sending = false;
//	}
//
//	// stop waypoint receiving if timeout
//	if (waypoint_receiving && (tnow - waypoint_timelast_receive) > waypoint_receive_timeout){
//		waypoint_receiving = false;
//	}

}


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void Copter::mavlink_delay_cb()
{

}

///*
// *  send a message on both GCS links
// */
//void Copter::gcs_send_message(enum ap_message id)
//{
//    for (uint8_t i=0; i<num_gcs; i++) {
//        if (gcs[i].initialised) {
//            gcs[i].send_message(id);
//        }
//    }
//}

/*
 *  send a mission item reached message and load the index before the send attempt in case it may get delayed
 */
void Copter::gcs_send_mission_item_reached_message(uint16_t mission_index)
{

}

/*
 *  send data streams in the given rate range on both links
 */
void Copter::gcs_data_stream_send(void)
{
//    for (uint8_t i=0; i<num_gcs; i++) {
//        if (gcs[i].initialised) {
//            gcs[i].data_stream_send();
//        }
//    }

	gcs0.data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Copter::gcs_check_input(void)
{
//    for (uint8_t i=0; i<num_gcs; i++) {
//        if (gcs[i].initialised) {
//#if CLI_ENABLED == ENABLED
//            gcs[i].update(g.cli_enabled==1?FUNCTOR_BIND_MEMBER(&Copter::run_cli, void, AP_HAL::UARTDriver *):NULL);
//#else
//            gcs[i].update(NULL);
//#endif
//        }
//    }
}

//void Copter::gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
//{
////    for (uint8_t i=0; i<num_gcs; i++) {
////        if (gcs[i].initialised) {
////            gcs[i].send_text_P(severity, str);
////        }
////    }
//}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
//void Copter::gcs_send_text_fmt(const prog_char_t *fmt, ...)
//{
//    va_list arg_list;
//    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
//    va_start(arg_list, fmt);
//    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
//            sizeof(gcs[0].pending_status.text), fmt, arg_list);
//    va_end(arg_list);
//    gcs[0].send_message(MSG_STATUSTEXT);
//    for (uint8_t i=1; i<num_gcs; i++) {
//        if (gcs[i].initialised) {
//            gcs[i].pending_status = gcs[0].pending_status;
//            gcs[i].send_message(MSG_STATUSTEXT);
//        }
//    }
//}


void
GCS_MAVLINK::send_message(enum ap_message id)
{
	chan=MAVLINK_COMM_0;
	//chan我默认为0了 那这个packet_drops呢
    mavlink_send_message(chan,id, packet_drops);
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];//这个数组大小为2也就决定了最多有2个串口
// send a message using mavlink
void GCS_MAVLINK::mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    /*
     * 什么时候这个num_deferred_messages不等于0呢，只有不等于0才能发送呀，我看下面  if (q->num_deferred_messages != 0 ||
     *   !mavlink_try_send_message  mavlink_try_send_message这个函数只有在缓存空间不够的情况下才可能返回false 如果缓存一直够的话，
     *   num_deferred_messages会一直都是0
     *   那么这个mavlink_send_message其实 while循环 for循环都没啥用，最下面的if语句，也就只有mavlink_try_send_message需要执行，
     *   所以这个mavlink_send_message就执行了一条语句，即mavlink_try_send_message
     * 初始化时这些
     */
    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    /*
     * 上面的代码在初始状态时，num_deferred_messages=0的情况下都是没啥用的，下面这个语句在num_deferred_messages=0时也会发送一条信息
     */
    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

static uint8_t mav_var_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAVLINK_TYPE_INT8_T;
    }
    if (t == AP_PARAM_INT16) {
	    return MAVLINK_TYPE_INT16_T;
    }
    if (t == AP_PARAM_INT32) {
	    return MAVLINK_TYPE_INT32_T;
    }
    // treat any others as float
    return MAVLINK_TYPE_FLOAT;
}



/**
* @brief Send the next pending waypoint, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_waypoint_send()
{

	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    if (waypoint_receiving &&
        waypoint_request_i < (unsigned)copter.g.command_total) {

    	mavlink_msg_mission_request_pack(mavlink_system.sysid ,mavlink_system.compid,&msg,\
    																										system_type, autopilot_type,waypoint_request_i);

    	// Copy the message to the send buffer
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);

//        mavlink_msg_waypoint_request_send(
//            chan,
//            waypoint_dest_sysid,
//            waypoint_dest_compid,
//            waypoint_request_i);
    }
}


/**
* @brief Send the next pending parameter, called from deferred message
* handling code
*/
/*
 * 20171002先注释掉，试一试我的行不行
 */
//void
//GCS_MAVLINK::queued_param_send()
//{
//    // Check to see if we are sending parameters
//    if (NULL == _queued_parameter) return;
//
//    AP_Param      *vp;
//    float       value;
//
//    // copy the current parameter and prepare to move to the next
//    vp = _queued_parameter;
//
//    // if the parameter can be cast to float, report it here and break out of the loop
//    //value = vp->cast_to_float(_queued_parameter_type);
//    value = (float)(_queued_parameter_type);
//
//    char param_name[ONBOARD_PARAM_NAME_LENGTH];
//    vp->copy_name(param_name, sizeof(param_name), true);
//
//    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
//   mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
//
//
//
//   // Initialize the required buffers
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//
//    mavlink_msg_param_value_pack_chan(mavlink_system.sysid ,mavlink_system.compid ,chan,
//    		                                                             &msg,
//    		                                                             param_name,
//																		 value,
//																		 mav_var_type(_queued_parameter_type),
//																		 _queued_parameter_count,
//																		 _queued_parameter_index );
//
//
//    // Copy the message to the send buffer
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
//
//    _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
//    _queued_parameter_index++;
//    printf("_queued_parameter_index=%d\n",_queued_parameter_index);
//}
void
GCS_MAVLINK::queued_param_send()
{
    // Check to see if we are sending parameters
    if (NULL == _queued_parameter_wang) return;

    T_PARAM      *vp;
    float       value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter_wang;

    // if the parameter can be cast to float, report it here and break out of the loop
    //value = vp->cast_to_float(_queued_parameter_type);
    //value = (float)(_queued_parameter_type);
    value=_queued_parameter_wang->value;

    char param_name[ONBOARD_PARAM_NAME_LENGTH];
    //vp->copy_name(param_name, sizeof(param_name), true);
    memcpy(param_name,&_queued_parameter_wang->name, sizeof(param_name));//只复制了15个字节，但是name定义的是16个字节

    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
   mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process



   // Initialize the required buffers
   mavlink_message_t msg;
   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_value_pack_chan(mavlink_system.sysid ,mavlink_system.compid ,chan,
    		                                                             &msg,
    		                                                             param_name,
																		 value,
																		 MAVLINK_TYPE_FLOAT,
																		 //mav_var_type(_queued_parameter_type),
																		 _queued_parameter_count,
																		 _queued_parameter_index );


    // Copy the message to the send buffer
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
   send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);

    _queued_parameter_wang = _queued_parameter_wang->next_param;
    _queued_parameter_index++;
    printf("_queued_parameter_index=%d\n",_queued_parameter_index);
}





// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false
// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
//    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
//
//    if (chan == MAVLINK_COMM_1 && millis() < MAVLINK_TELEMETRY_PORT_DELAY) {
//        // defer any messages on the telemetry port for 1 second after
//        // bootup, to try to prevent bricking of Xbees
//        return false;
//    }

	int payload_space=INT_MAX;//comm_get_txspace这个函数的返回值本来的意思是串口还剩下多少空间，但是这里直接返回最大值，意思就是串口总是有空闲的空间的
	switch(id)
	{
	case MSG_HEARTBEAT:
		CHECK_PAYLOAD_SIZE(HEARTBEAT);
		//send_message(MSG_HEARTBEAT);
		copter.send_heartbeat(chan);
		//return true;
		break;
	case MSG_ATTITUDE:
		CHECK_PAYLOAD_SIZE(ATTITUDE);
		copter.send_attitude(chan);
		break;
	case MSG_LOCATION:
		CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
		copter.send_location(chan);
		break;
	case MSG_EXTENDED_STATUS1:
		CHECK_PAYLOAD_SIZE(SYS_STATUS);
		copter.send_extended_status1(chan);
		break;
	case MSG_EXTENDED_STATUS2:
//		CHECK_PAYLOAD_SIZE(MEMINFO);
//		send_meminfo(chan);
		break;
	case MSG_NAV_CONTROLLER_OUTPUT:
		CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
		copter.send_nav_controller_output(chan);
		break;
	case MSG_GPS_RAW://这个其实已经不需要了 已经有了发送location这个
		copter.send_gps_raw(chan);
		break;
    case MSG_SERVO_OUT:
//        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
//        copter.send_servo_out(chan);
        break;
    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        //copter.send_radio_in(chan);
        break;
    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        copter.send_radio_out(chan);
        break;
    case MSG_VFR_HUD:
//        CHECK_PAYLOAD_SIZE(VFR_HUD);
//        send_vfr_hud(chan);
        break;
    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        copter.send_raw_imu1(chan);
        break;
    case MSG_RAW_IMU2:
//        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
//        send_raw_imu2(chan);
        break;
    case MSG_RAW_IMU3:
//        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
//        send_raw_imu3(chan);
        break;
    case MSG_GPS_STATUS:
        CHECK_PAYLOAD_SIZE(GPS_STATUS);
        copter.send_gps_status(chan);
        break;
	case MSG_CURRENT_WAYPOINT:
		CHECK_PAYLOAD_SIZE(WAYPOINT_CURRENT);
		copter.send_current_waypoint(chan);
		break;
	/*
	* 发送参数的，这个比较重要，跟地面站通信用的，千万不要删除，谨慎修改
	*/
	case MSG_NEXT_PARAM:
		CHECK_PAYLOAD_SIZE(PARAM_VALUE);
		copter.gcs0.queued_param_send();//这个函数其实没有调用，还是在gcs_mavlink::update函数中调用的
		break;
	/*
	* 发送下一航点的，发送参数的，这个比较重要，地面站个驾驶仪发送航点时跟地面站通信用的，千万不要删除，谨慎修改
	* 下面这个回传函数是地面站给驾驶仪发送航点时，驾驶仪需要不断地向地面站请求发送航点
	*/
	case MSG_NEXT_WAYPOINT:
		CHECK_PAYLOAD_SIZE(WAYPOINT_REQUEST);
		copter.gcs0.queued_waypoint_send();
		break;
    case MSG_STATUSTEXT:
//        CHECK_PAYLOAD_SIZE(STATUSTEXT);
//        send_statustext(chan);
        break;
    case MSG_AHRS:
//        CHECK_PAYLOAD_SIZE(AHRS);
//        send_ahrs(chan);
        break;
	case MSG_RETRY_DEFERRED:
	break; // just here to prevent a warning
	}
    return true;
}


/*
 *
 * 20170922下面是从我编译的apm3.3中抄过来的
 */

void
GCS_MAVLINK::init(char * port)
{

	chan = MAVLINK_COMM_0;//20170922
//    GCS_Class::init(port);
//    if (port == &Serial) {
//        mavlink_comm_0_port = port;
//        chan = MAVLINK_COMM_0;
//    }else{
//        mavlink_comm_1_port = port;
//        chan = MAVLINK_COMM_1;
//    }
//    _queued_parameter = NULL;
}


uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
        return 1;
    // Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
    // If it is addressed to our system ID we assume it is for us
    return 0; // no error
}

uint16_t
GCS_MAVLINK::_count_parameters()
{
	// if we haven't cached the parameter count yet...
	if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
		do {
				_parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
	}
	return _parameter_count;
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
	struct Location tell_command = {};				// command for telemetry
//	static uint8_t mav_nav = 255;							// For setting mode (some require receipt of 2 messages...)
//
//    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    DEBUG_PRINTF("handleMessage    :    msg->msgid=%d\n",msg->msgid);


    switch (msg->msgid) {
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: //66
		{
			// decode
			mavlink_request_data_stream_t packet;
			mavlink_msg_request_data_stream_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

			int freq = 0; // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

			DEBUG_PRINTF("handleMessage    :    REQUEST_DATA_STREAM freq=%d\n",freq);
			DEBUG_PRINTF("handleMessage    :    REQUEST_DATA_STREAM    packet.req_stream_id = %d\n",packet.req_stream_id);
			DEBUG_PRINTF("handleMessage    :    REQUEST_DATA_STREAM    freq = %d\n",freq);

			switch(packet.req_stream_id){

				case MAV_DATA_STREAM_ALL:
					streamRateRawSensors		= freq;//20171003   频率freq是2
					streamRateExtendedStatus	= freq;//20171003   频率freq是2
					streamRateRCChannels		= freq;//20171003   频率freq是2
					streamRateRawController		= freq;
					streamRatePosition			= freq;//20171003   频率freq是3
					streamRateExtra1			= freq;//20171003   频率freq是10
					streamRateExtra2			= freq;//20171003   频率freq是10
					//streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
					streamRateExtra3			= freq;	// Don't save!!  //20171003   频率freq是2
					break;

				case MAV_DATA_STREAM_RAW_SENSORS:
					streamRateRawSensors = freq;		// We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
					break;
				case MAV_DATA_STREAM_EXTENDED_STATUS:
					//streamRateExtendedStatus.set_and_save(freq);
					streamRateExtendedStatus = freq;

					//DEBUG_PRINTF("handleMessage    :    MAV_DATA_STREAM_EXTENDED_STATUS freq=%d\n",freq);//20171002已测试 地面站连接驾驶仪时，地面站在请求驾驶仪回传全部参数会每一个参数都设置频率freq
					break;

				case MAV_DATA_STREAM_RC_CHANNELS:
					streamRateRCChannels  = freq;
					break;

				case MAV_DATA_STREAM_RAW_CONTROLLER:
					streamRateRawController = freq;
					break;

				//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
				//	streamRateRawSensorFusion.set_and_save(freq);
				//	break;

				case MAV_DATA_STREAM_POSITION:
					streamRatePosition = freq;
					//DEBUG_PRINTF("handleMessage    :    MAV_DATA_STREAM_POSITION freq=%d\n",freq);//20171002已测试 地面站连接驾驶仪时，地面站在请求驾驶仪回传全部参数会每一个参数都设置频率freq
					break;

				case MAV_DATA_STREAM_EXTRA1:
					streamRateExtra1 = freq;
					break;

				case MAV_DATA_STREAM_EXTRA2:
					streamRateExtra2 = freq;
					break;

				case MAV_DATA_STREAM_EXTRA3:
					streamRateExtra3 = freq;
					break;

				default:
					break;
			}
			break;
		}

    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST: //43
	{
		//send_text_P(SEVERITY_LOW,PSTR("waypoint request list"));

		DEBUG_PRINTF("handleMessage    :    请求回传所有航点\n");
		// decode
		mavlink_waypoint_request_list_t packet;
		mavlink_msg_waypoint_request_list_decode(msg, &packet);
		if (mavlink_check_target(packet.target_system, packet.target_component))
			break;

		mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
		mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
		// Define the system type, in this case an airplane
		uint8_t system_type = MAV_TYPE_FIXED_WING;
		uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

		// Initialize the required buffers
		mavlink_message_t msg;
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		//copter.g.command_total=10;
		mavlink_msg_mission_count_pack(mavlink_system.sysid , mavlink_system.compid,&msg,\
				                                                     system_type,autopilot_type,copter.g.command_total);

		// Copy the message to the send buffer
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);


//		// Start sending waypoints
//		mavlink_msg_waypoint_count_send(
//			chan,msg->sysid,
//			msg->compid,
//			copter.g.command_total); // includes home

		waypoint_timelast_send		= gettimeofday_ms();
		waypoint_sending			= true;
		waypoint_receiving			= false;
		waypoint_dest_sysid			= msg.sysid;
		waypoint_dest_compid		= msg.compid;
		break;
	}

	// XXX read a WP from EEPROM and send it to the GCS
		case MAVLINK_MSG_ID_WAYPOINT_REQUEST: // 40
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint request"));
			DEBUG_PRINTF("handleMessage    :    请求回传单个航点\n");
			// Check if sending waypiont
			//if (!waypoint_sending) break;
			// 5/10/11 - We are trying out relaxing the requirement that we be in waypoint sending mode to respond to a waypoint request.  DEW

			// decode
			mavlink_waypoint_request_t packet;
			mavlink_msg_waypoint_request_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

			//DEBUG_PRINTF("handleMessage    :    即将要回传的航点的编号=%d\n",packet.seq);
			printf("handleMessage    :    即将要回传的航点的编号=%d\n",packet.seq);

			// send waypoint
			tell_command = copter.get_cmd_with_index(packet.seq);
			printf("handleMessage    :    即将要回传的航点的经度 = %d\n",tell_command.lng);

			// set frame of waypoint
			uint8_t frame;

			if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
				frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
			} else {
				frame = MAV_FRAME_GLOBAL; // reference frame
			}

			float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

			// time that the mav should loiter in milliseconds
			uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == (uint16_t)copter.g.command_index)
				current = 1;

			uint8_t autocontinue = 1; // 1 (true), 0 (false)

			float x = 0, y = 0, z = 0;

			if (tell_command.id < MAV_CMD_NAV_LAST)
			{
				// command needs scaling
				x = tell_command.lat/1.0e7; // local (x), global (latitude)
				//y = tell_command.lng/1.0e7; // local (y), global (longitude)
#ifdef SITL_LINUX
				y = -tell_command.lng/1.0e7; // local (y), global (longitude)//20170928这个其实是不需要反号的，但是我的初始经度是-122，所以反号设置为正
#else
				y = tell_command.lng/1.0e7; // local (y), global (longitude)
#endif
				// ACM is processing alt inside each command. so we save and load raw values. - this is diffrent to APM
				z = tell_command.alt/1.0e2; // local (z), global/relative (altitude)
			}

			// Switch to map APM command fields inot MAVLink command fields
			switch (tell_command.id) {

				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_CONDITION_CHANGE_ALT:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_CONDITION_YAW:
					param3 = tell_command.p1;
					param1 = tell_command.alt;
					param2 = tell_command.lat;
					param4 = tell_command.lng;
					break;

				case MAV_CMD_NAV_TAKEOFF:
					param1 = 0;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					param1 = tell_command.p1;	// ACM loiter time is in 1 second increments
					break;

				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					param1 = tell_command.lat;
					break;

				case MAV_CMD_DO_JUMP:
					param2 = tell_command.lat;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					param4 = tell_command.lng;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					param3 = tell_command.lat;
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_NAV_WAYPOINT:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;
			}

			mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
			mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
			// Define the system type, in this case an airplane
			uint8_t system_type = MAV_TYPE_FIXED_WING;
			uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

			// Initialize the required buffers
			mavlink_message_t msg;
			uint8_t buf[MAVLINK_MAX_PACKET_LEN];

			mavlink_msg_mission_item_pack(mavlink_system.sysid,mavlink_system.compid,&msg,system_type,autopilot_type,\
					                                                    packet.seq,frame,tell_command.id,current,autocontinue,param1,param2,param3,param4,\
					                                                    //tell_command.lat,tell_command.lng,tell_command.alt );
					                                                    x,y,z );//这里其实应该是发送tell_command.lat,tell_command.lng,tell_command.alt 的，但是我反号了，就发送xyz

			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);

			// update last waypoint comm stamp
			//waypoint_timelast_send = millis();
			waypoint_timelast_send = gettimeofday_ms();
			break;
		}



		case MAVLINK_MSG_ID_WAYPOINT_ACK: //47
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint ack"));

			DEBUG_PRINTF("回传航点结束\n");
			// decode
			mavlink_waypoint_ack_t packet;
			mavlink_msg_waypoint_ack_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// turn off waypoint send
			waypoint_sending = false;
			break;
		}

		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{

			  // decode
				mavlink_param_request_read_t packet;
				mavlink_msg_param_request_read_decode(msg, &packet);
//				if (mavlink_check_target(packet.target_system,packet.target_component)) break;
//				if (packet.param_index != -1) {
//					gcs_send_text_P(SEVERITY_LOW, PSTR("Param by index not supported"));
//					break;
//				}

			//DEBUG_PRINTF("******************************************************************************请求回传单个参数\n");
			//printf("******************************************************************************请求回传单个参数\n");
			DEBUG_PRINTF("packet.param_index=%d\n",packet.param_index);
			DEBUG_PRINTF("packet.param_id=%s\n",packet.param_id);


			mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
			mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

			// Define the system type, in this case an airplane
			uint8_t system_type = MAV_TYPE_FIXED_WING;
			uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

			uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
			//uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
			uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight


			// Initialize the required buffers
			mavlink_message_t msg;
			uint8_t buf[MAVLINK_MAX_PACKET_LEN];

			////不用下面的这句
		//mavlink_msg_param_request_read_pack(mavlink_system.sysid,mavlink_system.compid,&msg,system_type,autopilot_type,packet.param_id,packet.param_index);

			//mavlink_msg_param_value_send
			mavlink_msg_param_value_pack(mavlink_system.sysid,mavlink_system.compid,&msg,\
					                      packet.param_id,1.0,MAVLINK_TYPE_FLOAT,1,packet.param_index);


			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);


		//handle_param_request_read(msg);
		break;
		}




		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: // 21
		{
			// gcs_send_text_P(SEVERITY_LOW,PSTR("param request list"));

			DEBUG_PRINTF("******************************************************************************请求回传所有参数\n");

			/*
			 * 参数的发送放在了gcs_update里面，这里只是设置以下参数队列中的第一个排队的，_queued_parameter_wang
			 * 航点的回传也是放在了gcs_update里面，
			 *
			 */

			// decode
			mavlink_param_request_list_t packet;
			mavlink_msg_param_request_list_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// Start sending parameters - next call to ::update will kick the first one out

			_queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
			_queued_parameter_index = 0;
			_queued_parameter_count = _count_parameters();


			/*
			 * 20171002
			 */
			_queued_parameter_wang = &param_all[0];
			_queued_parameter_index = 0;
			_queued_parameter_count = param_all_cnt;//20171002这个参数在Parameters.h文件中声明定义

			break;
		}

		case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
		{
			// We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			// if(msg->sysid != copter.g.sysid_my_gcs) break;
			//copter.failsafe.last_heartbeat_ms = hal.scheduler->millis();
			//copter.pmTest1++;
		break;
		}


	    case MAVLINK_MSG_ID_PARAM_SET:     // 23
		{
			//	    			AP_Var				  *vp;
			//	    			AP_Meta_class::Type_id  var_type;

			// decode
			mavlink_param_set_t packet;
			mavlink_msg_param_set_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
			break;

			// set parameter
			float value=0.0;

			char key[ONBOARD_PARAM_NAME_LENGTH+1];
			strncpy(key, (char *)packet.param_id, ONBOARD_PARAM_NAME_LENGTH);
			key[ONBOARD_PARAM_NAME_LENGTH] = 0;

			value=packet.param_value;

			DEBUG_PRINTF("handleMessage    :    key = %s\n",key);
			DEBUG_PRINTF("handleMessage    :    param value = %f\n",value);

			int i=0;
			for(i=0;i<_queued_parameter_count;i++)
			{
				if(!strcmp(key,param_all[i].name))
				{
					//key param_all.name相同
					DEBUG_PRINTF("handleMessage    :    字符串相同 i=%d\n",i);
					param_all[i].value=value;
					break;//找到相同的后，就停止查询
				}
			}

			/*
			 * 然后该怎么把参数赋值给参数呢？我这里先用switch语句，然后按照Parameters.h中定义的变量的顺序
			 */
			switch(param_all[i].key)
			{
			case k_param_format_version:
				//Parameters::k_format_version=(uint16_t)value;
				break;
			case k_param_software_type:
				//Parameters::k_software_type=(uint16_t)value;
			break;
			case k_param_waypoint_radius:
				copter.g.waypoint_radius=(int8_t)value;
				copter.battery_voltage1=12;
				DEBUG_PRINTF("handleMessage    :    copter.g.waypoint_radius=%d\n",copter.g.waypoint_radius);
				break;
			case k_param_p_stabilize_roll:
				copter.g.pi_stabilize_roll.set_kP(value);
				DEBUG_PRINTF("handleMessage    :    g.pi_stabilize_roll  P=%f\n",value);
				break;
			case k_param_p_stabilize_pitch:
				copter.g.pi_stabilize_pitch.set_kP(value);
				DEBUG_PRINTF("handleMessage    :    g.pi_stabilize_pitch  P=%f\n",value);
				break;
			case k_param_p_stabilize_yaw:
				copter.g.pi_stabilize_yaw.set_kP(value);
				DEBUG_PRINTF("handleMessage    :    g.pi_stabilize_yaw  P=%f\n",value);
				break;
			case k_param_pid_rate_roll_p:
				copter.g.pid_rate_roll.set_kP(value);
				break;
			case k_param_pid_rate_roll_i:
				copter.g.pid_rate_roll.set_kI(value);
				break;
			case k_param_pid_rate_roll_d:
				copter.g.pid_rate_roll.set_kD(value);
				break;
			case k_param_pid_rate_pitch_p:
				copter.g.pid_rate_pitch.set_kP(value);
				break;
			case k_param_pid_rate_pitch_i:
				copter.g.pid_rate_pitch.set_kI(value);
				break;
			case k_param_pid_rate_pitch_d:
				copter.g.pid_rate_pitch.set_kD(value);
				break;
			case k_param_pid_rate_yaw_p:
				copter.g.pid_rate_yaw.set_kP(value);
				break;
			case k_param_pid_rate_yaw_i:
				copter.g.pid_rate_yaw.set_kI(value);
				break;
			case k_param_pid_rate_yaw_d:
				copter.g.pid_rate_yaw.set_kD(value);
				break;

			default:
				break;

			}


			mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
			mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process


			// Initialize the required buffers
			mavlink_message_t msg;
			uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//			static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//			                               const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index);

			mavlink_msg_param_value_pack(mavlink_system.sysid,mavlink_system.compid,&msg,\
					                                                    key,value,MAVLINK_TYPE_FLOAT,_queued_parameter_count,_queued_parameter_index);
			// Copy the message to the send buffer
			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);





			/*
			* 现在已经找到了参数的名称保存在key里面
			* 需要通过key找到驾驶仪中参数的变量名称，然后把packet.param_value赋值给这个参数变量
			* 这里面有个问题就是这个packet.param_value是具有不同类型的，有可能是int 也有可能是float
			* 所以我需要进行如下操作
			* 1把所有的参数名称写成字符串数组列表，这样与key比对，能够得到该key在数组中的位置
			* 2得到位置后，用switch语句，case这个位置值，根据case的值分别给参数赋值
			* 3这样可能麻烦一点，但是好歹能够使用数组来保存参数了
			* 4别忘了设置参数后，需要把设置好的参数发回给地面站，相应一下
			*/

			//	    				// Report back the new value if we accepted the change
			//	    				// we send the value we actually set, which could be
			//	    				// different from the value sent, in case someone sent
			//	    				// a fractional value to an integer type
			//	    				mavlink_msg_param_value_send(
			//	    					chan,
			//	    					(int8_t *)key,
			//	    					vp->cast_to_float(),
			//	    					_count_parameters(),
			//	    					-1); // XXX we don't actually know what its index is...
			//
			//	    			}



			/*
			* 下面其实就是通过key这个字符串，也就是参数的名字，找到该字符串在flash中的位置，
			* 然后对vp这个指针根据 var_type也就是数据的类型，强制转换，从而保存地面站发送过来的参数值
			* 再接收到参数值后还需要返回mavlink_msg_param_value_send
			*static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
			const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index) 实际用这个函数打包
			参考网址
			http://qgroundcontrol.org/mavlink/parameter_protocol
			*/


			// find the requested parameter
			//	    			vp = AP_Var::find(key);
			//
			//	    			if ((NULL != vp) &&							 		// exists
			//	    					!isnan(packet.param_value) &&			   // not nan
			//	    					!isinf(packet.param_value)) {			   // not inf
			//
			//	    				// add a small amount before casting parameter values
			//	    				// from float to integer to avoid truncating to the
			//	    				// next lower integer value.
			//	    				float rounding_addition = 0.01;
			//
			//	    				// fetch the variable type ID
			//	    				var_type = vp->meta_type_id();
			//
			//	    				// handle variables with standard type IDs
			//	    				if (var_type == AP_Var::k_typeid_float) {
			//	    					((AP_Float *)vp)->set_and_save(packet.param_value);
			//	    #if LOGGING_ENABLED == ENABLED
			//	    					Log_Write_Data(1, (float)((AP_Float *)vp)->get());
			//	    #endif
			//	    				} else if (var_type == AP_Var::k_typeid_float16) {
			//	    					((AP_Float16 *)vp)->set_and_save(packet.param_value);
			//	    #if LOGGING_ENABLED == ENABLED
			//	    					Log_Write_Data(2, (float)((AP_Float *)vp)->get());
			//	    #endif
			//	    				} else if (var_type == AP_Var::k_typeid_int32) {
			//	    					if (packet.param_value < 0) rounding_addition = -rounding_addition;
			//	    					((AP_Int32 *)vp)->set_and_save(packet.param_value+rounding_addition);}


			//	    				// Report back the new value if we accepted the change
			//	    				// we send the value we actually set, which could be
			//	    				// different from the value sent, in case someone sent
			//	    				// a fractional value to an integer type
			//	    				mavlink_msg_param_value_send(
			//	    					chan,
			//	    					(int8_t *)key,
			//	    					vp->cast_to_float(),
			//	    					_count_parameters(),
			//	    					-1); // XXX we don't actually know what its index is...
			//
			//	    			}

			break;
		} // end case

	    /*
	     * 20170928地面站在给驾驶仪发送航点时应该会先发送这个航点数的吧
	     */
	    case MAVLINK_MSG_ID_WAYPOINT_COUNT: // 44
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint count"));

			// decode
			mavlink_waypoint_count_t packet;
			mavlink_msg_waypoint_count_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// start waypoint receiving
			if (packet.count > MAX_WAYPOINTS) {
				packet.count = MAX_WAYPOINTS;
			}
			copter.g.command_total=packet.count;
			//copter.g.command_total=10;
			/*
			 * 20170928已经断点测试，这个函数在地面站给驾驶仪设置航点时，会设置g.command_total微packet.count
			 */
			DEBUG_PRINTF("handleMessage    ：    copter.g.command_total=%d\n",copter.g.command_total);




			//waypoint_timelast_receive = millis();
			waypoint_receiving   = true;
			waypoint_sending	 = false;
			waypoint_request_i   = 0;
			waypoint_timelast_request = 0;
			break;
		}


		/*
		 * 解析收到的航点保存到数组里
		 */
	    // XXX receive a WP from GCS and store in EEPROM
		case MAVLINK_MSG_ID_WAYPOINT: //39
		{

			// decode
			mavlink_waypoint_t packet;
			mavlink_msg_waypoint_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// defaults
			tell_command.id = packet.command;

			/*
			switch (packet.frame){

				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2; // in as m converted to cm
						tell_command.options = 0; // absolute altitude
						break;
					}

				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
				//case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
				default:
					{
						tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
						break;
					}
			}
			*/

			// we only are supporting Abs position, relative Alt
			tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
			tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
			tell_command.alt = packet.z * 1.0e2;
			tell_command.options = 1; // store altitude relative!! Always!!

			switch (tell_command.id) {					// Switch to map APM command fields inot MAVLink command fields
				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_DO_SET_HOME:
				case MAV_CMD_DO_SET_ROI:
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_CONDITION_YAW:
					tell_command.p1 = packet.param3;
					tell_command.alt = packet.param1;
					tell_command.lat = packet.param2;
					tell_command.lng = packet.param4;
					break;

				case MAV_CMD_NAV_TAKEOFF:
					tell_command.p1 = 0;
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					tell_command.p1 = packet.param1 * 100;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					tell_command.p1 = packet.param1;	// APM loiter time is in ten second increments
					break;

				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					tell_command.lat = packet.param1;
					break;

				case MAV_CMD_DO_JUMP:
					tell_command.lat = packet.param2;
					tell_command.p1  = packet.param1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					tell_command.lng = packet.param4;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					tell_command.lat = packet.param3;
					tell_command.alt = packet.param2;
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_NAV_WAYPOINT:
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					tell_command.alt = packet.param2;
					tell_command.p1 = packet.param1;
					break;
			}

			if(packet.current == 2){ 				//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
//				guided_WP = tell_command;
//
//				// add home alt if needed
//				if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT){
//					guided_WP.alt += home.alt;
//				}
//
//				set_mode(GUIDED);
//
//				// make any new wp uploaded instant (in case we are already in Guided mode)
//				set_next_WP(&guided_WP);
//
//				// verify we recevied the command
//				mavlink_msg_waypoint_ack_send(
//						chan,
//						msg->sysid,
//						msg->compid,
//						0);

			} else {
				// Check if receiving waypoints (mission upload expected)
				if (!waypoint_receiving) break;

				static int seq_temp=0;
				seq_temp=packet.seq;

				seq_temp=seq_temp+0;
				// check if this is the requested waypoint
				if (packet.seq != waypoint_request_i)
					break;

				DEBUG_PRINTF("*********************************************************************************************保存航点\n");
				/*
				 * 这里是具体的把航点保存到flash中，但是我保存到数组中
				 */
				if(packet.seq != 0)//20170928这个if需要吗？为什么seq=0不需要呢
					copter.set_cmd_with_index(tell_command, packet.seq);

				// update waypoint receiving state machine
				//waypoint_timelast_receive = millis();
				waypoint_timelast_request = 0;
				waypoint_request_i++;

				if (waypoint_request_i == (uint16_t)copter.g.command_total){
					uint8_t type = 0; // ok (0), error(1)

					mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
					mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

					// Define the system type, in this case an airplane
					uint8_t system_type = MAV_TYPE_FIXED_WING;
					uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
					// Initialize the required buffers
					mavlink_message_t msg;
					uint8_t buf[MAVLINK_MAX_PACKET_LEN];


					mavlink_msg_mission_ack_pack(mavlink_system.sysid,mavlink_system.compid,&msg,system_type,autopilot_type,MAV_MISSION_ACCEPTED);

					/*
					 * 上面是pack并不包括发送，但是一般pack打包后都需要发送的，下面的老是忘记
					 */
					// Copy the message to the send buffer
					uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
					send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf,len);
//					mavlink_msg_waypoint_ack_send(
//						chan,
//						msg->sysid,
//						msg->compid,
//						type);

					//send_text(SEVERITY_LOW,PSTR("flight plan received"));
					waypoint_receiving = false;
					// XXX ignores waypoint radius for individual waypoints, can
					// only set WP_RADIUS parameter
				}
			}
			break;
		}


    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        //handle_set_mode(msg, FUNCTOR_BIND(&copter, &Copter::set_mode, bool, uint8_t));

    	// decode
		mavlink_set_mode_t packet;
		mavlink_msg_set_mode_decode(msg, &packet);

		switch(packet.base_mode){

			case 2:
				DEBUG_PRINTF("handleMessage    :    地面站请求设置控制模式，增稳\n");
				//set_mode(STABILIZE);
				break;

			case 3:
				//set_mode(GUIDED);
				break;

			case 4:
//				if(mav_nav == 255 || mav_nav == MAV_NAV_WAYPOINT) 	set_mode(AUTO);
//				if(mav_nav == MAV_NAV_RETURNING)					set_mode(RTL);
//				if(mav_nav == MAV_NAV_LOITER)						set_mode(LOITER);
//				mav_nav = 255;
				break;

			case 0:
				//set_mode(STABILIZE);
				break;
		}
        break;
    }


//		case MAVLINK_MSG_ID_MISSION_COUNT:     // 44
//		{
//			//send_text_P(SEVERITY_LOW,PSTR("waypoint count"));
//			DEBUG_PRINTF("发送任务个数------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
//
//			// decode
//			mavlink_mission_count_t packet;
//			mavlink_msg_mission_count_decode(msg, &packet);
//			if (mavlink_check_target(packet.target_system,packet.target_component)) break;
//
////			// start waypoint receiving
////			if (packet.count > MAX_WAYPOINTS) {
////				packet.count = MAX_WAYPOINTS;
////			}
//			copter.g.command_total=10;
//
//			//waypoint_timelast_receive = millis();
//			waypoint_receiving   = true;
//			waypoint_sending         = false;
//			waypoint_request_i   = 0;
//			waypoint_timelast_request = 0;
//			break;
//		}
		default:
		//handle_common_message(msg);
		break;

//    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
//    {
//        //handle_set_mode(msg, FUNCTOR_BIND(&copter, &Copter::set_mode, bool, uint8_t));
//        break;
//    }
//
//    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:         // MAV ID: 20
//    {
//        handle_param_request_read(msg);
//        break;
//    }
//
//    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:         // MAV ID: 21
//    {
//        // mark the firmware version in the tlog
//        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));
//
//#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
//        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
//#endif
//        send_text_P(SEVERITY_LOW, PSTR("Frame: " FRAME_CONFIG_STRING));
//        handle_param_request_list(msg);
//        break;
//    }
//

//
//    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
//    {
//        handle_mission_write_partial_list(copter.mission, msg);
//        break;
//    }
//
//    // GCS has sent us a mission item, store to EEPROM
//    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
//    {
//        if (handle_mission_item(msg, copter.mission)) {
//            copter.DataFlash.Log_Write_EntireMission(copter.mission);
//        }
//        break;
//    }
//
//    // read an individual command from EEPROM and send it to the GCS
//    case MAVLINK_MSG_ID_MISSION_REQUEST:     // MAV ID: 40
//    {
//        handle_mission_request(copter.mission, msg);
//        break;
//    }
//
//    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
//    {
//        handle_mission_set_current(copter.mission, msg);
//        break;
//    }
//
//    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
//    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
//    {
//        handle_mission_request_list(copter.mission, msg);
//        break;
//    }
//
//    // GCS provides the full number of commands it wishes to upload
//    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
//    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
//    {
//        handle_mission_count(copter.mission, msg);
//        break;
//    }
//
//    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
//    {
//        handle_mission_clear_all(copter.mission, msg);
//        break;
//    }
//
//    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:    // MAV ID: 66
//    {
//        handle_request_data_stream(msg, false);
//        break;
//    }
//
//    case MAVLINK_MSG_ID_GIMBAL_REPORT:
//    {
//#if MOUNT == ENABLED
//        handle_gimbal_report(copter.camera_mount, msg);
//#endif
//        break;
//    }
//
//    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
//    {
//        // allow override of RC channel values for HIL
//        // or for complete GCS control of switch position
//        // and RC PWM values.
//        if(msg->sysid != copter.g.sysid_my_gcs) break;                         // Only accept control from our gcs
//        mavlink_rc_channels_override_t packet;
//        int16_t v[8];
//        mavlink_msg_rc_channels_override_decode(msg, &packet);
//
//        v[0] = packet.chan1_raw;
//        v[1] = packet.chan2_raw;
//        v[2] = packet.chan3_raw;
//        v[3] = packet.chan4_raw;
//        v[4] = packet.chan5_raw;
//        v[5] = packet.chan6_raw;
//        v[6] = packet.chan7_raw;
//        v[7] = packet.chan8_raw;
//
//        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
//        copter.failsafe.rc_override_active = hal.rcin->set_overrides(v, 8);
//
//        // a RC override message is considered to be a 'heartbeat' from the ground station for failsafe purposes
//        copter.failsafe.last_heartbeat_ms = hal.scheduler->millis();
//        break;
//    }
//
//    // Pre-Flight calibration requests
//    case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
//    {
//        // decode packet
//        mavlink_command_long_t packet;
//        mavlink_msg_command_long_decode(msg, &packet);
//
//        switch(packet.command) {
//
//        case MAV_CMD_START_RX_PAIR:
//            // initiate bind procedure
//            if (!hal.rcin->rc_bind(packet.param1)) {
//                result = MAV_RESULT_FAILED;
//            } else {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_NAV_TAKEOFF: {
//            // param3 : horizontal navigation by pilot acceptable
//            // param4 : yaw angle   (not supported)
//            // param5 : latitude    (not supported)
//            // param6 : longitude   (not supported)
//            // param7 : altitude [metres]
//
//            float takeoff_alt = packet.param7 * 100;      // Convert m to cm
//
//            if(copter.do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
//                result = MAV_RESULT_ACCEPTED;
//            } else {
//                result = MAV_RESULT_FAILED;
//            }
//            break;
//        }
//
//
//        case MAV_CMD_NAV_LOITER_UNLIM:
//            if (copter.set_mode(LOITER)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
//            if (copter.set_mode(RTL)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_NAV_LAND:
//            if (copter.set_mode(LAND)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_CONDITION_YAW:
//            // param1 : target angle [0-360]
//            // param2 : speed during change [deg per second]
//            // param3 : direction (-1:ccw, +1:cw)
//            // param4 : relative offset (1) or absolute angle (0)
//            if ((packet.param1 >= 0.0f)   &&
//            	(packet.param1 <= 360.0f) &&
//            	(is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
//            	copter.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
//                result = MAV_RESULT_ACCEPTED;
//            } else {
//                result = MAV_RESULT_FAILED;
//            }
//            break;
//
//        case MAV_CMD_DO_CHANGE_SPEED:
//            // param1 : unused
//            // param2 : new speed in m/s
//            // param3 : unused
//            // param4 : unused
//            if (packet.param2 > 0.0f) {
//                copter.wp_nav.set_speed_xy(packet.param2 * 100.0f);
//                result = MAV_RESULT_ACCEPTED;
//            } else {
//                result = MAV_RESULT_FAILED;
//            }
//            break;
//
//        case MAV_CMD_DO_SET_HOME:
//            // param1 : use current (1=use current location, 0=use specified location)
//            // param5 : latitude
//            // param6 : longitude
//            // param7 : altitude (absolute)
//            result = MAV_RESULT_FAILED; // assume failure
//            if(is_equal(packet.param1,1.0f) || (is_zero(packet.param5) && is_zero(packet.param6) && is_zero(packet.param7))) {
//                if (copter.set_home_to_current_location_and_lock()) {
//                    result = MAV_RESULT_ACCEPTED;
//                }
//            } else {
//                // sanity check location
//                if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
//                    break;
//                }
//                Location new_home_loc;
//                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
//                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
//                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
//                if (!copter.far_from_EKF_origin(new_home_loc)) {
//                    if (copter.set_home_and_lock(new_home_loc)) {
//                        result = MAV_RESULT_ACCEPTED;
//                    }
//                }
//            }
//            break;
//
//        case MAV_CMD_DO_FLIGHTTERMINATION:
//            if (packet.param1 > 0.5f) {
//                copter.init_disarm_motors();
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_DO_SET_ROI:
//            // param1 : regional of interest mode (not supported)
//            // param2 : mission index/ target id (not supported)
//            // param3 : ROI index (not supported)
//            // param5 : x / lat
//            // param6 : y / lon
//            // param7 : z / alt
//            // sanity check location
//            if (fabsf(packet.param5) > 90.0f || fabsf(packet.param6) > 180.0f) {
//                break;
//            }
//            Location roi_loc;
//            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
//            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
//            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
//            copter.set_auto_yaw_roi(roi_loc);
//            result = MAV_RESULT_ACCEPTED;
//            break;
//
//        case MAV_CMD_MISSION_START:
//            if (copter.motors.armed() && copter.set_mode(AUTO)) {
//                copter.set_auto_armed(true);
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_PREFLIGHT_CALIBRATION:
//            // exit immediately if armed
//            if (copter.motors.armed()) {
//                result = MAV_RESULT_FAILED;
//                break;
//            }
//            if (is_equal(packet.param1,1.0f)) {
//                // gyro offset calibration
//                copter.ins.init_gyro();
//                // reset ahrs gyro bias
//                if (copter.ins.gyro_calibrated_ok_all()) {
//                    copter.ahrs.reset_gyro_drift();
//                    result = MAV_RESULT_ACCEPTED;
//                } else {
//                    result = MAV_RESULT_FAILED;
//                }
//            } else if (is_equal(packet.param3,1.0f)) {
//                // fast barometer calibration
//                copter.init_barometer(false);
//                result = MAV_RESULT_ACCEPTED;
//            } else if (is_equal(packet.param4,1.0f)) {
//                result = MAV_RESULT_UNSUPPORTED;
//            } else if (is_equal(packet.param5,1.0f)) {
//                // 3d accel calibration
//                float trim_roll, trim_pitch;
//                // this blocks
//                AP_InertialSensor_UserInteract_MAVLink interact(this);
//                if(copter.ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
//                    // reset ahrs's trim to suggested values from calibration routine
//                    copter.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
//                    result = MAV_RESULT_ACCEPTED;
//                } else {
//                    result = MAV_RESULT_FAILED;
//                }
//            } else if (is_equal(packet.param5,2.0f)) {
//                // accel trim
//                float trim_roll, trim_pitch;
//                if(copter.ins.calibrate_trim(trim_roll, trim_pitch)) {
//                    // reset ahrs's trim to suggested values from calibration routine
//                    copter.ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
//                    result = MAV_RESULT_ACCEPTED;
//                } else {
//                    result = MAV_RESULT_FAILED;
//                }
//            } else if (is_equal(packet.param6,1.0f)) {
//                // compassmot calibration
//                result = copter.mavlink_compassmot(chan);
//            }
//            break;
//
//        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
//            if (is_equal(packet.param1,2.0f)) {
//                // save first compass's offsets
//                copter.compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
//                result = MAV_RESULT_ACCEPTED;
//            }
//            if (is_equal(packet.param1,5.0f)) {
//                // save secondary compass's offsets
//                copter.compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_COMPONENT_ARM_DISARM:
//            if (is_equal(packet.param1,1.0f)) {
//                // attempt to arm and return success or failure
//                if (copter.init_arm_motors(true)) {
//                    result = MAV_RESULT_ACCEPTED;
//                }
//            } else if (is_zero(packet.param1) && (copter.mode_has_manual_throttle(copter.control_mode) || copter.ap.land_complete || is_equal(packet.param2,21196.0f)))  {
//                // force disarming by setting param2 = 21196 is deprecated
//                copter.init_disarm_motors();
//                result = MAV_RESULT_ACCEPTED;
//            } else {
//                result = MAV_RESULT_UNSUPPORTED;
//            }
//            break;
//
//        case MAV_CMD_DO_SET_SERVO:
//            if (copter.ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_DO_REPEAT_SERVO:
//            if (copter.ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_DO_SET_RELAY:
//            if (copter.ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_DO_REPEAT_RELAY:
//            if (copter.ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
//            if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
//                AP_Notify::events.firmware_update = 1;
//                copter.update_notify();
//                hal.scheduler->delay(50);
//                // when packet.param1 == 3 we reboot to hold in bootloader
//                hal.scheduler->reboot(is_equal(packet.param1,3.0f));
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//
//        case MAV_CMD_DO_FENCE_ENABLE:
//#if AC_FENCE == ENABLED
//            result = MAV_RESULT_ACCEPTED;
//            switch ((uint16_t)packet.param1) {
//                case 0:
//                    copter.fence.enable(false);
//                    break;
//                case 1:
//                    copter.fence.enable(true);
//                    break;
//                default:
//                    result = MAV_RESULT_FAILED;
//                    break;
//            }
//#else
//            // if fence code is not included return failure
//            result = MAV_RESULT_FAILED;
//#endif
//            break;
//
//#if PARACHUTE == ENABLED
//        case MAV_CMD_DO_PARACHUTE:
//            // configure or release parachute
//            result = MAV_RESULT_ACCEPTED;
//            switch ((uint16_t)packet.param1) {
//                case PARACHUTE_DISABLE:
//                    copter.parachute.enabled(false);
//                    copter.Log_Write_Event(DATA_PARACHUTE_DISABLED);
//                    break;
//                case PARACHUTE_ENABLE:
//                    copter.parachute.enabled(true);
//                    copter.Log_Write_Event(DATA_PARACHUTE_ENABLED);
//                    break;
//                case PARACHUTE_RELEASE:
//                    // treat as a manual release which performs some additional check of altitude
//                    copter.parachute_manual_release();
//                    break;
//                default:
//                    result = MAV_RESULT_FAILED;
//                    break;
//            }
//            break;
//#endif
//
//        case MAV_CMD_DO_MOTOR_TEST:
//            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
//            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
//            // param3 : throttle (range depends upon param2)
//            // param4 : timeout (in seconds)
//            result = copter.mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4);
//            break;
//
//#if EPM_ENABLED == ENABLED
//        case MAV_CMD_DO_GRIPPER:
//            // param1 : gripper number (ignored)
//            // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
//            if(!copter.epm.enabled()) {
//                result = MAV_RESULT_FAILED;
//            } else {
//                result = MAV_RESULT_ACCEPTED;
//                switch ((uint8_t)packet.param2) {
//                    case GRIPPER_ACTION_RELEASE:
//                        copter.epm.release();
//                        break;
//                    case GRIPPER_ACTION_GRAB:
//                        copter.epm.grab();
//                        break;
//                    default:
//                        result = MAV_RESULT_FAILED;
//                        break;
//                }
//            }
//            break;
//#endif
//
//        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
//            if (is_equal(packet.param1,1.0f)) {
//                copter.gcs[chan-MAVLINK_COMM_0].send_autopilot_version(FIRMWARE_VERSION);
//                result = MAV_RESULT_ACCEPTED;
//            }
//            break;
//        }
//
//        default:
//            result = MAV_RESULT_UNSUPPORTED;
//            break;
//        }
//
//        // send ACK or NAK
//        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
//
//        break;
//    }
//
//    case MAVLINK_MSG_ID_COMMAND_ACK:        // MAV ID: 77
//    {
//        copter.command_ack_counter++;
//        break;
//    }
//
//    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
//    {
//        // decode packet
//        mavlink_set_position_target_local_ned_t packet;
//        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);
//
//        // exit if vehicle is not in Guided mode or Auto-Guided mode
//        if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
//            break;
//        }
//
//        // check for supported coordinate frames
//        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
//            packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
//            packet.coordinate_frame != MAV_FRAME_BODY_NED &&
//            packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
//            break;
//        }
//
//        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
//        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
//        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
//
//        /*
//         * for future use:
//         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
//         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
//         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
//         */
//
//        // prepare position
//        Vector3f pos_vector;
//        if (!pos_ignore) {
//            // convert to cm
//            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
//            // rotate to body-frame if necessary
//            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
//                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
//                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
//            }
//            // add body offset if necessary
//            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
//                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
//                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
//                pos_vector += copter.inertial_nav.get_position();
//            } else {
//                // convert from alt-above-home to alt-above-ekf-origin
//                pos_vector.z = copter.pv_alt_above_origin(pos_vector.z);
//            }
//        }
//
//        // prepare velocity
//        Vector3f vel_vector;
//        if (!vel_ignore) {
//            // convert to cm
//            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
//            // rotate to body-frame if necessary
//            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
//                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
//            }
//        }
//
//        // send request
//        if (!pos_ignore && !vel_ignore && acc_ignore) {
//            copter.guided_set_destination_posvel(pos_vector, vel_vector);
//        } else if (pos_ignore && !vel_ignore && acc_ignore) {
//            copter.guided_set_velocity(vel_vector);
//        } else if (!pos_ignore && vel_ignore && acc_ignore) {
//            copter.guided_set_destination(pos_vector);
//        } else {
//            result = MAV_RESULT_FAILED;
//        }
//
//        break;
//    }
//
//    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
//    {
//        // decode packet
//        mavlink_set_position_target_global_int_t packet;
//        mavlink_msg_set_position_target_global_int_decode(msg, &packet);
//
//        // exit if vehicle is not in Guided mode or Auto-Guided mode
//        if ((copter.control_mode != GUIDED) && !(copter.control_mode == AUTO && copter.auto_mode == Auto_NavGuided)) {
//            break;
//        }
//
//        // check for supported coordinate frames
//        if (packet.coordinate_frame != MAV_FRAME_GLOBAL_INT &&
//            packet.coordinate_frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
//            packet.coordinate_frame != MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
//            break;
//        }
//
//        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
//        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
//        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
//
//        /*
//         * for future use:
//         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
//         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
//         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
//         */
//
//        Vector3f pos_ned;
//
//        if(!pos_ignore) {
//            Location loc;
//            loc.lat = packet.lat_int;
//            loc.lng = packet.lon_int;
//            loc.alt = packet.alt*100;
//            switch (packet.coordinate_frame) {
//                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
//                    loc.flags.relative_alt = true;
//                    loc.flags.terrain_alt = false;
//                    break;
//                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
//                    loc.flags.relative_alt = true;
//                    loc.flags.terrain_alt = true;
//                    break;
//                case MAV_FRAME_GLOBAL_INT:
//                default:
//                    loc.flags.relative_alt = false;
//                    loc.flags.terrain_alt = false;
//                    break;
//            }
//            pos_ned = copter.pv_location_to_vector(loc);
//        }
//
//        if (!pos_ignore && !vel_ignore && acc_ignore) {
//            copter.guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
//        } else if (pos_ignore && !vel_ignore && acc_ignore) {
//            copter.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
//        } else if (!pos_ignore && vel_ignore && acc_ignore) {
//            copter.guided_set_destination(pos_ned);
//        } else {
//            result = MAV_RESULT_FAILED;
//        }
//
//        break;
//    }
//
//#if HIL_MODE != HIL_MODE_DISABLED
//    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
//    {
//        mavlink_hil_state_t packet;
//        mavlink_msg_hil_state_decode(msg, &packet);
//
//        // set gps hil sensor
//        Location loc;
//        loc.lat = packet.lat;
//        loc.lng = packet.lon;
//        loc.alt = packet.alt/10;
//        Vector3f vel(packet.vx, packet.vy, packet.vz);
//        vel *= 0.01f;
//
//        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
//                   packet.time_usec/1000,
//                   loc, vel, 10, 0, true);
//
//        // rad/sec
//        Vector3f gyros;
//        gyros.x = packet.rollspeed;
//        gyros.y = packet.pitchspeed;
//        gyros.z = packet.yawspeed;
//
//        // m/s/s
//        Vector3f accels;
//        accels.x = packet.xacc * (GRAVITY_MSS/1000.0f);
//        accels.y = packet.yacc * (GRAVITY_MSS/1000.0f);
//        accels.z = packet.zacc * (GRAVITY_MSS/1000.0f);
//
//        ins.set_gyro(0, gyros);
//
//        ins.set_accel(0, accels);
//
//        copter.barometer.setHIL(packet.alt*0.001f);
//        copter.compass.setHIL(0, packet.roll, packet.pitch, packet.yaw);
//        copter.compass.setHIL(1, packet.roll, packet.pitch, packet.yaw);
//
//        break;
//    }
//#endif //  HIL_MODE != HIL_MODE_DISABLED
//
//    case MAVLINK_MSG_ID_RADIO:
//    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
//    {
//        handle_radio_status(msg, copter.DataFlash, copter.should_log(MASK_LOG_PM));
//        break;
//    }
//
//    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
//    case MAVLINK_MSG_ID_LOG_ERASE:
//        copter.in_log_download = true;
//        // fallthru
//    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
//        if (!copter.in_mavlink_delay && !copter.motors.armed()) {
//            handle_log_message(msg, copter.DataFlash);
//        }
//        break;
//    case MAVLINK_MSG_ID_LOG_REQUEST_END:
//        copter.in_log_download = false;
//        if (!copter.in_mavlink_delay && !copter.motors.armed()) {
//            handle_log_message(msg, copter.DataFlash);
//        }
//        break;
//
//#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
//    case MAVLINK_MSG_ID_SERIAL_CONTROL:
//        handle_serial_control(msg, copter.gps);
//        break;
//
//    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
//        handle_gps_inject(msg, copter.gps);
//        result = MAV_RESULT_ACCEPTED;
//        break;
//
//#endif
//
//#if CAMERA == ENABLED
//    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:      // MAV ID: 202
//        break;
//
//    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
//        copter.camera.control_msg(msg);
//        copter.log_picture();
//        break;
//#endif // CAMERA == ENABLED
//
//#if MOUNT == ENABLED
//    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:        // MAV ID: 204
//        copter.camera_mount.configure_msg(msg);
//        break;
//
//    case MAVLINK_MSG_ID_MOUNT_CONTROL:
//        copter.camera_mount.control_msg(msg);
//        break;
//#endif // MOUNT == ENABLED
//
//    case MAVLINK_MSG_ID_TERRAIN_DATA:
//    case MAVLINK_MSG_ID_TERRAIN_CHECK:
//#if AP_TERRAIN_AVAILABLE
//        copter.terrain.handle_data(chan, msg);
//#endif
//        break;
//
//#if AC_RALLY == ENABLED
//    // receive a rally point from GCS and store in EEPROM
//    case MAVLINK_MSG_ID_RALLY_POINT: {
//        mavlink_rally_point_t packet;
//        mavlink_msg_rally_point_decode(msg, &packet);
//
//        if (packet.idx >= copter.rally.get_rally_total() ||
//            packet.idx >= copter.rally.get_rally_max()) {
//            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
//            break;
//        }
//
//        if (packet.count != copter.rally.get_rally_total()) {
//            send_text_P(SEVERITY_LOW,PSTR("bad rally point message count"));
//            break;
//        }
//
//        RallyLocation rally_point;
//        rally_point.lat = packet.lat;
//        rally_point.lng = packet.lng;
//        rally_point.alt = packet.alt;
//        rally_point.break_alt = packet.break_alt;
//        rally_point.land_dir = packet.land_dir;
//        rally_point.flags = packet.flags;
//
//        if (!copter.rally.set_rally_point_with_index(packet.idx, rally_point)) {
//            send_text_P(SEVERITY_HIGH, PSTR("error setting rally point"));
//        }
//
//        break;
//    }
//
//    //send a rally point to the GCS
//    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
//        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.cpp 1")); // #### TEMP
//
//        mavlink_rally_fetch_point_t packet;
//        mavlink_msg_rally_fetch_point_decode(msg, &packet);
//
//        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.cpp 2")); // #### TEMP
//
//        if (packet.idx > copter.rally.get_rally_total()) {
//            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));
//            break;
//        }
//
//        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.cpp 3")); // #### TEMP
//
//        RallyLocation rally_point;
//        if (!copter.rally.get_rally_point_with_index(packet.idx, rally_point)) {
//           send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));
//           break;
//        }
//
//        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.cpp 4")); // #### TEMP
//
//        mavlink_msg_rally_point_send_buf(msg,
//                                         chan, msg->sysid, msg->compid, packet.idx,
//                                         copter.rally.get_rally_total(), rally_point.lat, rally_point.lng,
//                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir,
//                                         rally_point.flags);
//
//        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.cpp 5")); // #### TEMP
//
//        break;
//    }
//#endif // AC_RALLY == ENABLED
//
//    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
//        copter.gcs[chan-MAVLINK_COMM_0].send_autopilot_version(FIRMWARE_VERSION);
//        break;
//
//    case MAVLINK_MSG_ID_LED_CONTROL:
//        // send message to Notify
//        AP_Notify::handle_led_control(msg);
//        break;

    }     // end switch
} // end handle mavlink
#endif
