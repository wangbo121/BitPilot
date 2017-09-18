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

#ifdef LINUX_OS
/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>

#include <unistd.h>

#include <fcntl.h>//创建文件

#include <sys/stat.h>
#endif

#include "global.h"
#include "uart.h"


#include "boatlink.h"
//#include "copter.h"

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



 struct Location wp_total_array_temp[255]={0};
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



    //for(int i=0;i<4;i++)
    for(int i=0;i<global_bool_boatpilot.wp_total_num;i++)

    {
    	wp_data[i].no=i;

    	wp_data[i].lng=-(wp_total_array_temp[i].lng*1e-2);//wp_total_array_temp是我mavlink定义的航点数组
    	//wp_data[i].lng=(wp_total_array_temp[i].lng*1e-2);//wp_total_array_temp是我mavlink定义的航点数组
    	wp_data[i].lat=wp_total_array_temp[i].lat*1e-2;
    }


    memcpy(&ap2gcs_wp.way_point0,&wp_data[wp_start],wp_num*sizeof(struct WAY_POINT));//回传航点的时候把飞机的航点回传了





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
    send_uart_data(uart_device_ap2gcs.uart_name, (char *)buf_packet,ret);

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

int decode_gcs2ap_cmd(struct GCS2AP_RADIO *ptr_gcs2ap_radio, struct GCS2AP_CMD *ptr_gcs2ap_cmd)
{
    ptr_gcs2ap_radio->pack_func_flag=ptr_gcs2ap_cmd->pack_func_flag;
    ptr_gcs2ap_radio->pack_func_info1=ptr_gcs2ap_cmd->pack_func_info1;
    ptr_gcs2ap_radio->pack_func_info2=ptr_gcs2ap_cmd->pack_func_info2;
    ptr_gcs2ap_radio->pack_func_info3=ptr_gcs2ap_cmd->pack_func_info3;

    if((ptr_gcs2ap_radio->pack_func_flag & 0x01)==0)
    {
        /* pack_func_flag最低位等于0:请求同步，等于1:正常命令包 */
        /*
         * 命令包优先，如果pack_func_flag的从右往左第二位也等于1--请求航点回传，
         * 则忽视，优先回传命令包
         * 不是回传命令包时，我们会判断是否回传航点包，并同时更新地面站传到驾驶仪的命令
         */
        global_bool_boatpilot.send_ap2gcs_cmd_req=TRUE;
        printf("地面站请求同步，回传gcs2ap_radio中保存的关于cmd命令包的数据\n");
    }
    else
    {
    	//global_bool_boatpilot.wp_total_num=4;
        /*
         * 不是回传命令时，则需要获取cmd的值到gcs2ap_radio中
         * 也就是说进入这个判断则意味着地面站在发送正常命令包
         * 需要把一些标志为如回传命令，发送特定航点标志清零
         */
        global_bool_boatpilot.send_ap2gcs_cmd_req=FALSE;
        global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;

        if(((ptr_gcs2ap_radio->pack_func_flag >> 1) & 0x01))
        {
            printf("地面站请求回传航点\n");

            if(global_bool_boatpilot.wp_total_num>=1)
            {
                if(ptr_gcs2ap_radio->pack_func_info1==255)
                {
                    printf("地面站要求回传全部航点\n");

                    //printf("ptr_gcs2ap_radio->pack_func_info3=%d\n",ptr_gcs2ap_radio->pack_func_info3);//20170729已测试通过
                    //printf("global_bool_boatpilot.ap2gcs_wp_cnt=%d\n",global_bool_boatpilot.ap2gcs_wp_cnt);//20170729已测试通过

                    global_bool_boatpilot.bool_is_sending_wp_ap2gcs=TRUE;//正在发送航点
                    global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;//不是发送指定的某些航点，把这个标志量归零

                    if(global_bool_boatpilot.wp_total_num/5)
                    {
                        //航点总数目大于5个了
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+5-1;
                    }
                    else if(global_bool_boatpilot.wp_total_num%5)
                    {
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+global_bool_boatpilot.wp_total_num-1;
                    }
                    ap2gcs_wp.pack_func_info1=global_bool_boatpilot.wp_total_num;//当地面站要求回传全部航点时，要发送的航点总数就是总的航点数目
                }
                else
                {
                    //地面站请求回传航点一次后，就会把ptr_gcs2ap_radio->pack_func_info1=255置零
                    printf("地面站请求回传特定航点，起始航点是%d，航点数是%d\n",ptr_gcs2ap_radio->pack_func_info1,ptr_gcs2ap_radio->pack_func_info2);
                    if(ptr_gcs2ap_radio->pack_func_info2>5)
                    {
                        global_bool_boatpilot.send_ap2gcs_specific_wp_req=TRUE;
                        global_bool_boatpilot.send_ap2gcs_wp_start_num=0;
                        global_bool_boatpilot.send_ap2gcs_wp_end_num=0;

                        /*
                         * 地面站请求回传特定航点时，所请求回传的航点数大于5了，
                         * 但是驾驶仪目前在回传特定航点时最多只能回传5个，拒绝回传
                         * 只回传0航点
                         */
                        printf("地面站请求回传特定航点时，所请求回传的航点数大于5了，只回传0航点\n");
                    }
                    else
                    {
                        global_bool_boatpilot.send_ap2gcs_specific_wp_req=TRUE;

                        if(global_bool_boatpilot.wp_total_num/(ptr_gcs2ap_radio->pack_func_info1+ptr_gcs2ap_radio->pack_func_info2))
                        {
                            //wp_data数组中有足够多的航点，能够回传所请求的航点数
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=ptr_gcs2ap_radio->pack_func_info1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+ptr_gcs2ap_radio->pack_func_info2-1;
                        }
                        else if(global_bool_boatpilot.wp_total_num%(ptr_gcs2ap_radio->pack_func_info1+ptr_gcs2ap_radio->pack_func_info2))
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=ptr_gcs2ap_radio->pack_func_info1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+((global_bool_boatpilot.wp_total_num-1)-global_bool_boatpilot.send_ap2gcs_wp_start_num);
                        }
                        ap2gcs_wp.pack_func_info1=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                        printf("wp_total_num=%d,info1=%d,info2=%d\n",global_bool_boatpilot.wp_total_num,ap2gcs_wp.pack_func_info1,ptr_gcs2ap_radio->pack_func_info2);//20170508已测试
                    }
                }

                /*
                 * 到了这一级判断，则
                 * 1.地面站请求回传航点了
                 * 2.并且航点数>=1了
                 * 所以总是要回传一个航点包的
                 */
                ap2gcs_wp.pack_func_flag=global_bool_boatpilot.wp_total_num;
                ap2gcs_wp.pack_func_info2=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                global_bool_boatpilot.send_ap2gcs_wp_req=TRUE;
            }
            else
            {
                //总的航点数小于1，所以什么也不做
                //printf("虽然地面站请求回传航点，但是驾驶仪中保存的航点数小于1，无法回传\n");
            }
        }
        else if((global_bool_boatpilot.ap2gcs_wp_cnt==ptr_gcs2ap_radio->pack_func_info3))
        {
            if(global_bool_boatpilot.wp_total_num>=1)
            {
                if(global_bool_boatpilot.send_ap2gcs_specific_wp_req)
                {
                    /*
                     * 如果是请求发送指定航点，怎么判断结束呢
                     * 因为请求发送指定航点时，数目肯定是小于等于5个的，所以不需要判断是否回传结束
                     * 直接回传发送一次就好
                     */
                    global_bool_boatpilot.send_ap2gcs_specific_wp_req=FALSE;
                }
                else
                {
                    //在不是回传指定航点的情况下，判断回传全部航点是否结束，如果没有结束，继续回传
                    if(global_bool_boatpilot.send_ap2gcs_wp_end_num==(global_bool_boatpilot.wp_total_num-1))
                    {
                        if(global_bool_boatpilot.bool_is_sending_wp_ap2gcs)
                        {
                            printf("已经回传结束全部航点\n");
                        }
                        global_bool_boatpilot.bool_is_sending_wp_ap2gcs=FALSE;
                    }
                    else if(global_bool_boatpilot.bool_is_sending_wp_ap2gcs)
                    {
                        printf("已经开始回传航点，但是还没有回传结束\n");
                        int remain;
                        remain=global_bool_boatpilot.wp_total_num-(global_bool_boatpilot.send_ap2gcs_wp_end_num+1);

                        if(remain/5)
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=global_bool_boatpilot.send_ap2gcs_wp_end_num+1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+4;
                        }
                        else if(remain%5)
                        {
                            global_bool_boatpilot.send_ap2gcs_wp_start_num=global_bool_boatpilot.send_ap2gcs_wp_end_num+1;
                            global_bool_boatpilot.send_ap2gcs_wp_end_num=global_bool_boatpilot.send_ap2gcs_wp_start_num+remain-1;
                        }

                        ap2gcs_wp.pack_func_info1=global_bool_boatpilot.wp_total_num;

                        ap2gcs_wp.pack_func_flag=global_bool_boatpilot.wp_total_num;
                        ap2gcs_wp.pack_func_info2=global_bool_boatpilot.send_ap2gcs_wp_end_num-global_bool_boatpilot.send_ap2gcs_wp_start_num+1;
                        global_bool_boatpilot.send_ap2gcs_wp_req=TRUE;
                    }
                }
            }
            else
            {
                //总的航点数小于1，所以什么也不做
                printf("虽然地面站请求回传航点，但是驾驶仪中保存的航点数小于1，无法回传\n");
            }
        }

        ptr_gcs2ap_radio->workmode=ptr_gcs2ap_cmd->workmode;
        ptr_gcs2ap_radio->rc_thruster=ptr_gcs2ap_cmd->rc_thruster;
        ptr_gcs2ap_radio->rc_rudder=ptr_gcs2ap_cmd->rc_rudder;
        ptr_gcs2ap_radio->rud_p=ptr_gcs2ap_cmd->rud_p;
        ptr_gcs2ap_radio->rud_i=ptr_gcs2ap_cmd->rud_i;
        ptr_gcs2ap_radio->rud_d=ptr_gcs2ap_cmd->rud_d;
        ptr_gcs2ap_radio->cte_p=ptr_gcs2ap_cmd->cte_p;
        ptr_gcs2ap_radio->cte_i=ptr_gcs2ap_cmd->cte_i;
        ptr_gcs2ap_radio->cte_d=ptr_gcs2ap_cmd->cte_d;
        ptr_gcs2ap_radio->rudder_setup_reverse=ptr_gcs2ap_cmd->rudder_setup_reverse;
        ptr_gcs2ap_radio->thruster_setup_reverse=ptr_gcs2ap_cmd->thruster_setup_reverse;
        ptr_gcs2ap_radio->generator_on=ptr_gcs2ap_cmd->generator_on;
        ptr_gcs2ap_radio->thruster_backward=ptr_gcs2ap_cmd->thruster_backward;
        ptr_gcs2ap_radio->motor_lock=ptr_gcs2ap_cmd->motor_lock;
        ptr_gcs2ap_radio->middle_motor_on=ptr_gcs2ap_cmd->middle_motor_on;
        ptr_gcs2ap_radio->navigation_mode=ptr_gcs2ap_cmd->navigation_mode;
        ptr_gcs2ap_radio->charge_start=ptr_gcs2ap_cmd->charge_start;
        ptr_gcs2ap_radio->rocket_hat=ptr_gcs2ap_cmd->rocket_hat;
        ptr_gcs2ap_radio->rocket_launch=ptr_gcs2ap_cmd->rocket_launch;
        ptr_gcs2ap_radio->turn_mode=ptr_gcs2ap_cmd->turn_mode;
        ptr_gcs2ap_radio->diffspd_coef=ptr_gcs2ap_cmd->diffspd_coef;
        ptr_gcs2ap_radio->diffspd_lim=ptr_gcs2ap_cmd->diffspd_lim;
        ptr_gcs2ap_radio->cruise_throttle_percent=ptr_gcs2ap_cmd->cruise_throttle_percent;
        ptr_gcs2ap_radio->throttle_change_time=ptr_gcs2ap_cmd->throttle_change_time;
        ptr_gcs2ap_radio->arrive_radius=ptr_gcs2ap_cmd->arrive_radius;
        ptr_gcs2ap_radio->cte_max_degree=ptr_gcs2ap_cmd->cte_max_degree;
        ptr_gcs2ap_radio->rudder_calib=ptr_gcs2ap_cmd->rudder_calib;
        ptr_gcs2ap_radio->rudder_calib_cnt=ptr_gcs2ap_cmd->rudder_calib_cnt;
        ptr_gcs2ap_radio->set_switch_channel=ptr_gcs2ap_cmd->set_switch_channel;
        ptr_gcs2ap_radio->set_switch_low_limit=ptr_gcs2ap_cmd->set_switch_low_limit;
        ptr_gcs2ap_radio->set_switch_high_limit=ptr_gcs2ap_cmd->set_switch_high_limit;
        ptr_gcs2ap_radio->set_charge_channel=ptr_gcs2ap_cmd->set_charge_channel;
        ptr_gcs2ap_radio->set_charge_voltage=ptr_gcs2ap_cmd->set_charge_voltage;
        ptr_gcs2ap_radio->set_charge_current=ptr_gcs2ap_cmd->set_charge_current;
        ptr_gcs2ap_radio->rudder_dead_zone_angle_degree=ptr_gcs2ap_cmd->rudder_dead_zone_angle_degree;
        ptr_gcs2ap_radio->slave_config=ptr_gcs2ap_cmd->slave_config;

        ptr_gcs2ap_radio->wp_flag=ptr_gcs2ap_cmd->wp_flag;
        ptr_gcs2ap_radio->wp_next=ptr_gcs2ap_cmd->wp_next;
        ptr_gcs2ap_radio->spd=ptr_gcs2ap_cmd->spd;
        ptr_gcs2ap_radio->alt=ptr_gcs2ap_cmd->alt;
        ptr_gcs2ap_radio->lng=ptr_gcs2ap_cmd->lat;
        ptr_gcs2ap_radio->lat=ptr_gcs2ap_cmd->lat;
    }

#if 0
    printf("ptr_gcs2ap_radio->workmode=%d\n",ptr_gcs2ap_radio->workmode);
    printf("ptr_gcs2ap_radio->rc_thruster=%d\n",ptr_gcs2ap_radio->rc_thruster);
    printf("ptr_gcs2ap_radio->rc_rudder=%d\n",ptr_gcs2ap_radio->rc_rudder);
    printf("ptr_gcs2ap_radio->rud_p=%d\n",ptr_gcs2ap_radio->rud_p);
    printf("ptr_gcs2ap_radio->rud_i=%d\n",ptr_gcs2ap_radio->rud_i);
    printf("ptr_gcs2ap_radio->rud_d=%d\n",ptr_gcs2ap_radio->rud_d);
    printf("ptr_gcs2ap_radio->cte_p=%d\n",ptr_gcs2ap_radio->cte_p);
    printf("ptr_gcs2ap_radio->cte_i=%d\n",ptr_gcs2ap_radio->cte_i);
    printf("ptr_gcs2ap_radio->cte_d=%d\n",ptr_gcs2ap_radio->cte_d);
    printf("ptr_gcs2ap_radio->rudder_setup_reverse=%d\n",ptr_gcs2ap_radio->rudder_setup_reverse);
    printf("ptr_gcs2ap_radio->thruster_setup_reverse=%d\n",ptr_gcs2ap_radio->thruster_setup_reverse);
    printf("ptr_gcs2ap_radio->generator_on=%d\n",ptr_gcs2ap_radio->generator_on);
    printf("ptr_gcs2ap_radio->thruster_backward=%d\n",ptr_gcs2ap_radio->thruster_backward);
    printf("ptr_gcs2ap_radio->motor_lock=%d\n",ptr_gcs2ap_radio->motor_lock);
    printf("ptr_gcs2ap_radio->middle_motor_on=%d\n",ptr_gcs2ap_radio->middle_motor_on);
    printf("ptr_gcs2ap_radio->navigation_mode=%d\n",ptr_gcs2ap_radio->navigation_mode);
    printf("ptr_gcs2ap_radio->charge_start=%d\n",ptr_gcs2ap_radio->charge_start);
    printf("ptr_gcs2ap_radio->rocket_hat=%d\n",ptr_gcs2ap_radio->rocket_hat);
    printf("ptr_gcs2ap_radio->rocket_launch=%d\n",ptr_gcs2ap_radio->rocket_launch);
    printf("ptr_gcs2ap_radio->turn_mode=%d\n",ptr_gcs2ap_radio->turn_mode);
    printf("ptr_gcs2ap_radio->diffspd_coef=%d\n",ptr_gcs2ap_radio->diffspd_coef);
    printf("ptr_gcs2ap_radio->diffspd_lim=%d\n",ptr_gcs2ap_radio->diffspd_lim);
    printf("ptr_gcs2ap_radio->cruise_throttle_percent=%d\n",ptr_gcs2ap_radio->cruise_throttle_percent);
    printf("ptr_gcs2ap_radio->throttle_change_time=%d\n",ptr_gcs2ap_radio->throttle_change_time);
    printf("ptr_gcs2ap_radio->arrive_radius=%d\n",ptr_gcs2ap_radio->arrive_radius);
    printf("ptr_gcs2ap_radio->cte_max_degree=%d\n",ptr_gcs2ap_radio->cte_max_degree);
    printf("ptr_gcs2ap_radio->rudder_calib=%d\n",ptr_gcs2ap_radio->rudder_calib);
    printf("ptr_gcs2ap_radio->rudder_calib_cnt=%d\n",ptr_gcs2ap_radio->rudder_calib_cnt);
    printf("ptr_gcs2ap_radio->set_switch_channel=%d\n",ptr_gcs2ap_radio->set_switch_channel);
    printf("ptr_gcs2ap_radio->set_switch_low_limit=%d\n",ptr_gcs2ap_radio->set_switch_low_limit);
    printf("ptr_gcs2ap_radio->set_switch_high_limit=%d\n",ptr_gcs2ap_radio->set_switch_high_limit);
    printf("ptr_gcs2ap_radio->set_charge_channel=%d\n",ptr_gcs2ap_radio->set_charge_channel);
    printf("ptr_gcs2ap_radio->set_charge_voltage=%d\n",ptr_gcs2ap_radio->set_charge_voltage);
    printf("ptr_gcs2ap_radio->set_charge_current=%d\n",ptr_gcs2ap_radio->set_charge_current);
    printf("ptr_gcs2ap_radio->wp_flag=%d\n",ptr_gcs2ap_radio->wp_flag);
    printf("ptr_gcs2ap_radio->wp_next=%d\n",ptr_gcs2ap_radio->wp_next);
    printf("ptr_gcs2ap_radio->spd=%d\n",ptr_gcs2ap_radio->spd);
    printf("ptr_gcs2ap_radio->alt=%d\n",ptr_gcs2ap_radio->alt);
    printf("ptr_gcs2ap_radio->lng=%d\n",ptr_gcs2ap_radio->lng);
    printf("ptr_gcs2ap_radio->lat=%d\n",ptr_gcs2ap_radio->lat);
#endif


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
