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


Copter copter;
