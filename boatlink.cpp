/*
 * boatlink.c
 *
 *  Created on: 2016年5月16日
 *      Author: wangbo
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>
#include <unistd.h>

#include <fcntl.h>//创建文件

#include <sys/stat.h>

#include "global.h"
#include "uart.h"


#include "boatlink.h"

static int generate_packet(unsigned char*dst_buf, unsigned char *src_buf,unsigned char len,\
                    unsigned int packet_cnt, unsigned char message_type,\
                    unsigned char commu_method, unsigned char ack_req);

#define MOTOR_FORWARD 0

#define GENERATOR_STOP 0
#define GENERATOR_START 1
#define GENERATOR_AUTO 2

#define TURN_MODE_RUDDER 0
#define TURN_MODE_DIFFSPD 1
#define TURN_MODE_MIX 2

#define SWITCH_AUTO      0
#define SWITCH_CHANNEL_0 1
#define SWITCH_CHANNEL_1 2
#define SWITCH_STOP      3

struct GCS_AP_WP gcs_ap_wp;
struct GCS_AP_WP ap2gcs_wp;
struct GCS2AP_CMD gcs2ap_cmd;
struct GCS2AP_CMD gcs2ap_cmd_return;
struct AP2GCS_REAL ap2gcs_real;

struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

struct GCS2AP_RADIO gcs2ap_radio_all;

struct T_CONFIG boatpilot_config_previous;
struct T_CONFIG boatpilot_config;

struct T_BOATPILOT_LOG boatpilot_log;

 int decode_gcs2ap_waypoint(struct WAY_POINT *ptr_wp_data, struct GCS_AP_WP *ptr_gcs2ap_wp);



/*
 * 发送指定从wp_start起始的wp_num个航点
 */
int send_ap2gcs_waypoint_num(unsigned char wp_start,unsigned char wp_num)
{
    unsigned char buf_data[256];
    unsigned char buf_packet[256];

    unsigned int ret;

    printf("最终发送给地面站的航点个数=%d\n",wp_num);
    memset(&ap2gcs_wp.way_point0,0,5*sizeof(struct WAY_POINT));
    memcpy(&ap2gcs_wp.way_point0,&wp_data[wp_start],wp_num*sizeof(struct WAY_POINT));

    //20170728把帧头帧尾加入到数据结构中
    static unsigned char frame_len=76;
    static unsigned char frame_head_len=8;
    static unsigned char frame_checksum_len=2;
    static unsigned char frame_data_len;
    frame_data_len=frame_len-frame_head_len-frame_checksum_len;
    memcpy(buf_data,&ap2gcs_wp.pack_func_flag,frame_data_len);//20170729注意这里一定是ap2gcs_wp，上次因为写成gcs_ap_wp一直没有查处原因，之所以犯这样的错误，是因为是复制过来的，没仔细看

    ret=generate_packet(buf_packet, buf_data, frame_data_len,\
                        global_bool_boatpilot.ap2gcs_wp_cnt, COMMAND_AP2GCS_WP,\
                        0,1);

    //send_radio_data(buf_packet, ret);

    int i=0;
    for(i=0;i<wp_num;i++)
    {
        printf("驾驶仪-->地面站航点包的第%d个航点的编号=%d\n",wp_start+i,wp_data[wp_start+i].no);
        printf("驾驶仪-->地面站航点包的第%d个航点的经度=%d\n",wp_start+i,wp_data[wp_start+i].lng);
        printf("驾驶仪-->地面站航点包的第%d个航点的纬度=%d\n",wp_start+i,wp_data[wp_start+i].lat);
        printf("驾驶仪-->地面站航点包的第%d个航点的高度=%d\n",wp_start+i,wp_data[wp_start+i].alt);
        printf("驾驶仪-->地面站航点包的第%d个航点的速度=%d\n",wp_start+i,wp_data[wp_start+i].spd);
    }

    return 0;
}


int send_ap2gcs_cmd()
{
    unsigned char buf_data[256];////要发送的数据，没有加帧头帧尾的，即需要打包的数据
    unsigned char buf_packet[256];//把数据加上帧头帧尾打包后，存放在这里，再发出去
    int ret;

    //20170728把帧头帧尾加入到数据结构中
    static unsigned char frame_len=76;
    static unsigned char frame_head_len=8;
    static unsigned char frame_checksum_len=2;
    static unsigned char frame_data_len;
    frame_data_len=frame_len-frame_head_len-frame_checksum_len;
    memcpy(buf_data,&gcs2ap_radio_all.pack_func_flag,frame_data_len);

    ret=generate_packet(buf_packet, buf_data, frame_data_len,\
                        global_bool_boatpilot.ap2gcs_cmd_cnt, COMMAND_AP2GCS_CMD,\
                        0,1);

    //send_radio_data(buf_packet, ret);

    return 0;
}


int decode_gcs2ap_waypoint(struct WAY_POINT *ptr_wp_data, struct GCS_AP_WP *ptr_gcs2ap_wp)
{
    int write_len=0;

    unsigned char wp_total;//航点总数
    unsigned char wp_num_need_to_send;//要发送的航点数
    unsigned char wp_num_in_pack;//本航点包所含有的航点数
    unsigned char wp_packet_cnt;//本航点包的编号，记录已经保存了多少个航点,需要返回给实时数据用于确认航点
    static unsigned char wp_cnt;

    unsigned char wp_start_no;//本航点包的第一个航点的编号

    wp_total=ptr_gcs2ap_wp->pack_func_flag;
    wp_num_need_to_send=ptr_gcs2ap_wp->pack_func_info1;
    wp_num_in_pack=ptr_gcs2ap_wp->pack_func_info2;
    wp_packet_cnt=ptr_gcs2ap_wp->pack_func_info3;

    wp_start_no=ptr_gcs2ap_wp->way_point0.no;

    if(wp_num_need_to_send>wp_total)
    {
        printf("要发送的航点数大于总航点数，请重新发送\n");
        return -1;
    }
    else
    {
        //printf("wp_start_no=%d\n",wp_start_no);//20170410已经测试
        //printf("wp_num_in_pack=%d\n",wp_num_in_pack);//20170410已经测试
        memcpy(&(ptr_wp_data[wp_start_no]),&(ptr_gcs2ap_wp->way_point0),sizeof(struct WAY_POINT)*wp_num_in_pack);

        wp_cnt+=wp_num_in_pack;

        if(wp_cnt>=wp_num_need_to_send)
        {
            printf("航点接收完全\n");

            wp_cnt=0;

            /*保存航点到航点文件*/
           // write_len=write(fd_waypoint,(char *)ptr_wp_data,sizeof(struct WAY_POINT)*MAX_WAYPOINT_NUM);
            printf("write_len 写入了%d个字节的航点\n",write_len);
        }

        global_bool_boatpilot.wp_total_num = wp_total;//这个global_bool_boatpilot.wp_total_num必须留着，因为gcs2ap_radio_all中的wp_total可能为0，来说明航点无效
        //printf("航点总数=%d\n",global_bool_boatpilot.wp_total_num);//已测试20170413
        global_bool_boatpilot.wp_packet_cnt=wp_packet_cnt;//包的计数通过实时数据返回给地面站，地面站确认后，再继续发航点包

        printf("wp_num_in_pack=%d\n",wp_num_in_pack);
        int i=0;
        for(i=0;i<wp_num_in_pack;i++)
        {
            printf("GCS-->boatpilot传输航点包的第%d个航点的编号=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].no);
            printf("GCS-->boatpilot传输航点包的第%d个航点的经度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].lng);
            printf("GCS-->boatpilot传输航点包的第%d个航点的纬度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].lat);
            printf("GCS-->boatpilot传输航点包的第%d个航点的高度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].alt);
            printf("GCS-->boatpilot传输航点包的第%d个航点的速度=%d\n",wp_start_no+i,ptr_wp_data[wp_start_no+i].spd);
        }
    }

    return 0;
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

    packet[4] = ADDRESS_BOATPILOT;
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
