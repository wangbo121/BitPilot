/*
 * GCS_Mavlink_Copter.h
 *
 *  Created on: 2017-9-22
 *      Author: wangbo
 */

#ifndef GCS_MAVLINK_COPTER_H_
#define GCS_MAVLINK_COPTER_H_


//#include <BetterStream.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
//#define MAVLINK_SEPARATE_HELPERS

#ifndef MAVLINK10
// default to MAVLINK 1.0
#define MAVLINK10 ENABLED
#endif

//#if MAVLINK10 == ENABLED
//# include "include/mavlink/v1.0/ardupilotmega/version.h"
//#else
//# include "include/mavlink/v0.9/ardupilotmega/version.h"
//#endif

// this allows us to make mavlink_message_t much smaller
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

//#define MAVLINK_COMM_NUM_BUFFERS 2
//#if MAVLINK10==ENABLED
//# include "include/mavlink/v1.0/mavlink_types.h"
//#else
//# include "include/mavlink/v0.9/mavlink_types.h"
//#endif
//
///// MAVLink stream used for HIL interaction
//extern BetterStream	*mavlink_comm_0_port;
//
///// MAVLink stream used for ground control communication
//extern BetterStream	*mavlink_comm_1_port;

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
/*
 * 20170922这个是发送单个字节的从这里发送给串口应该就可以了
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		//mavlink_comm_0_port->write(ch);
		break;
	case MAVLINK_COMM_1:
		//mavlink_comm_1_port->write(ch);
		break;
	default:
		break;
	}
}

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
/*
 * 从串口读取单个字节的数据
 */
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;

    switch(chan) {
	case MAVLINK_COMM_0:
		//data = mavlink_comm_0_port->read();
		break;
	case MAVLINK_COMM_1:
		//data = mavlink_comm_1_port->read();
		break;
	default:
		break;
	}
    return data;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_available(mavlink_channel_t chan)
{
    uint16_t bytes = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		//bytes = mavlink_comm_0_port->available();
		break;
	case MAVLINK_COMM_1:
		//bytes = mavlink_comm_1_port->available();
		break;
	default:
		break;
	}
    return bytes;
}


/*
 * 这个返回int_MAX这个最大值就可以了
 */
/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available, -1 for error
static inline int comm_get_txspace(mavlink_channel_t chan)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		//return mavlink_comm_0_port->txspace();
		break;
	case MAVLINK_COMM_1:
		//return mavlink_comm_1_port->txspace();
		break;
	default:
		break;
	}
    return -1;
}

//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
//#if MAVLINK10==ENABLED
//# include "include/mavlink/v1.0/ardupilotmega/mavlink.h"
//#else
//# include "include/mavlink/v0.9/ardupilotmega/mavlink.h"
//#endif

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);


#endif /* GCS_MAVLINK_COPTER_H_ */
