/*
 * boatlink.h
 *
 *  Created on: 2016年5月16日
 *      Author: wangbo
 */

#ifndef HEADERS_BOATLINK_H_
#define HEADERS_BOATLINK_H_

#define ADDRESS_BOATPILOT 0x01
#define ADDRESS_GCS 0x0F

#define COMMAND_GCS2AP_CMD 0x01
#define COMMAND_GCS2AP_WAYPOINT 0x02

#define COMMAND_AP2GCS_REAL 0x10
#define COMMAND_AP2GCS_CMD 0x11
#define COMMAND_AP2GCS_WP 0x12

#define COMMAND_AP2GCS_TEST 100




struct WAY_POINT
{
    unsigned char no;
    unsigned char type;
    unsigned char spd;
    unsigned char alt;

    unsigned int lng;
    unsigned int lat;
};

struct GCS_AP_WP
{
    unsigned char head1;
    unsigned char head2;
    unsigned char len;
    unsigned char cnt;
    unsigned char sysid;
    unsigned char type;
    unsigned char commu_method;
    unsigned char ack_req;//8字节

    unsigned char pack_func_flag;//航点总数，可能大于一个包,即wp_total
    unsigned char pack_func_info1;//本包起始航点号,即wp_start_no
    unsigned char pack_func_info2;//本包中包含的航点数,即wp_num
    unsigned char pack_func_info3;//本航点包的编号,1…n,即wppack_no.在向AP下发航点时,APGCS方收到此编号与已发送的编号相同后再发下一个航点包

    struct WAY_POINT way_point0;//每个航点12个
    struct WAY_POINT way_point1;
    struct WAY_POINT way_point2;
    struct WAY_POINT way_point3;
    struct WAY_POINT way_point4;//72

    unsigned char spare0;
    unsigned char spare1;
    unsigned char spare2;
    unsigned char checksum;//76个字节
};

struct GCS2AP_CMD
{
    unsigned char head1;
    unsigned char head2;
    unsigned char len;
    unsigned char cnt;
    unsigned char sysid;
    unsigned char type;
    unsigned char commu_method;
    unsigned char ack_req;//8字节

    unsigned char pack_func_flag;//包功能标志
    unsigned char pack_func_info1;//包功能辅助信息1.A:在发送请求回传航点时，此字节为回传航点的起始编号,0表示从起始航点开始,255表示回传全部航点.清零同上。
    unsigned char pack_func_info2;//包功能辅助信息2.A:在发送请求回传航点时，此字节为回传航点的个数，清零同上。
    unsigned char pack_func_info3;//包功能辅助信息3.A:在发送请求回传航点时，此字节为接收到的航点包编号，清零同上。
    unsigned char workmode;//工作模式，1:遥控;2:自驾;3:混控;0:停车(全部输出为0,电门锁生效)
    unsigned char rc_thruster;//[0-255]推进器pwm（遥控器输出）
    unsigned char rc_rudder;//[0-255]方向舵pwm（遥控器输出）
    unsigned char rud_p;//[0.1],转弯参数P
    unsigned char rud_i;//[0.1],转弯参数I
    unsigned char rud_d;//[0.1],转弯参数D
    unsigned char cte_p;//[0.1],CTE参数P
    unsigned char cte_i;//[0.1],CTE参数I
    unsigned char cte_d;//[0.1],CTE参数D
    unsigned char rudder_setup_reverse;//方向舵反向
    unsigned char thruster_setup_reverse;//推进器设置，0:正向，D0=1右侧反向:D1=1:左侧推进器反向
    unsigned char generator_on;//发电机设置，0:停止;1:工作;2:自动 //16字节
    unsigned char thruster_backward;//推进器,0:前进;1:后退
    unsigned char motor_lock;//电门锁,0:电门锁生效 D0=1：右侧电机电门解锁D1=1，左侧电机电门锁解锁
    unsigned char middle_motor_on;//小电机,0:关闭;1:打开
    unsigned char navigation_mode;//导航模式,1用航迹导航，0用偏航即船头导航
    unsigned char charge_start;//1充电机开始充电，0不充电
    unsigned char rocket_hat;//火箭仓盖,1:打开，0:关闭
    unsigned char rocket_launch;//火箭发射命令,1:请求发射
    unsigned char turn_mode;//转弯模式,D0=1:方向舵有效;D1=1:差速有效  //24字节
    unsigned char diffspd_coef;//[%]差速调整系数,(0.1-1.9)*100，即：两侧不调为100，若=90表示左侧出力为90%，右侧为110%，若110表示：左侧110%,右侧90%
    unsigned char diffspd_lim;//[%]差速调整限幅,0-100,即:50表示单侧最大增50%,最小50%
    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//[10米],到达半径
    unsigned char cte_max_degree;//[度],偏航距最大补偿角
    unsigned char rudder_calib;//舵标定标志,D0=1:到达左舵限位;D1=1:到达右侧限位;D2=1:舵位在中位
    unsigned char rudder_calib_cnt;
    unsigned char set_switch_channel;//[1..2]切换器通道
    unsigned char set_switch_low_limit;//[V],切换器放电电压低值
    unsigned char set_switch_high_limit;//[V],切换器放电电压高值
    unsigned char set_charge_channel;//[1..2]充电机通道
    unsigned char set_charge_voltage;//[V]充电机电压
    unsigned char set_charge_current;//[A]充电机电流
    unsigned char rudder_dead_zone_angle_degree;//[度]方向舵控制闭环时所用的死区角度数
    unsigned char master_config;
    unsigned char slave_config;//D0=0:充电机允许 D1=0:切换器允许 D2=1:电流传感器1允许 D3=1:电流传感器2允许 D4=0:气象站工作允许 D5=0:火箭工作允许
    unsigned char spare1;
    unsigned char spare2;
    unsigned char spare3;

    unsigned int spare4;//56字节
    unsigned int spare5;//60

    unsigned char wp_flag;//总航点数
    unsigned char wp_next;//下一个航点编号
    unsigned char spd;//[knot*0.1]航点设定航速
    unsigned char alt;//64

    unsigned int lng;//[度*0.00001]，航点GPS经度坐标，整型，精确到米
    unsigned int lat;//[度*0.00001]，航点GPS纬度坐标，整型，精确到米  //72字节

    unsigned char spare6;
    unsigned char spare7;
    unsigned char spare8;
    unsigned char checksum;
};


struct GCS2AP_RADIO
{
    unsigned char head1;
    unsigned char head2;
    unsigned char len;
    unsigned char cnt;
    unsigned char sysid;
    unsigned char type;
    unsigned char commu_method;
    unsigned char ack_req;//8字节

    unsigned char pack_func_flag;//包功能标志
    unsigned char pack_func_info1;//包功能辅助信息1.A:在发送请求回传航点时，此字节为回传航点的起始编号,0表示从起始航点开始,255表示回传全部航点.清零同上。
    unsigned char pack_func_info2;//包功能辅助信息2.A:在发送请求回传航点时，此字节为回传航点的个数，清零同上。
    unsigned char pack_func_info3;//包功能辅助信息3.A:在发送请求回传航点时，此字节为接收到的航点包编号，清零同上。
    unsigned char workmode;//工作模式，1:遥控;2:自驾;3:混控;0:停车(全部输出为0,电门锁生效)
    unsigned char rc_thruster;//[0-255]推进器pwm（遥控器输出）
    unsigned char rc_rudder;//[0-255]方向舵pwm（遥控器输出）
    unsigned char rud_p;//[0.1],转弯参数P
    unsigned char rud_i;//[0.1],转弯参数I
    unsigned char rud_d;//[0.1],转弯参数D
    unsigned char cte_p;//[0.1],CTE参数P
    unsigned char cte_i;//[0.1],CTE参数I
    unsigned char cte_d;//[0.1],CTE参数D
    unsigned char rudder_setup_reverse;//方向舵反向
    unsigned char thruster_setup_reverse;//推进器设置，0:正向，D0=1右侧反向:D1=1:左侧推进器反向
    unsigned char generator_on;//发电机设置，0:停止;1:工作;2:自动 //16字节
    unsigned char thruster_backward;//推进器,0:前进;1:后退
    unsigned char motor_lock;//电门锁,0:电门锁生效 D0=1：右侧电机电门解锁D1=1，左侧电机电门锁解锁
    unsigned char middle_motor_on;//小电机,0:关闭;1:打开
    unsigned char navigation_mode;//导航模式,1用航迹导航，0用偏航即船头导航
    unsigned char charge_start;//1充电机开始充电，0不充电
    unsigned char rocket_hat;//火箭仓盖,1:打开，0:关闭
    unsigned char rocket_launch;//火箭发射命令,1:请求发射
    unsigned char turn_mode;//转弯模式,D0=1:方向舵有效;D1=1:差速有效  //24字节
    unsigned char diffspd_coef;//[%]差速调整系数,(0.1-1.9)*100，即：两侧不调为100，若=90表示左侧出力为90%，右侧为110%，若110表示：左侧110%,右侧90%
    unsigned char diffspd_lim;//[%]差速调整限幅,0-100,即:50表示单侧最大增50%,最小50%
    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//[10米],到达半径
    unsigned char cte_max_degree;//[度],偏航距最大补偿角
    unsigned char rudder_calib;//舵标定标志,D0=1:到达左舵限位;D1=1:到达右侧限位;D2=1:舵位在中位
    unsigned char rudder_calib_cnt;
    unsigned char set_switch_channel;//[1..2]切换器通道
    unsigned char set_switch_low_limit;//[V],切换器放电电压低值
    unsigned char set_switch_high_limit;//[V],切换器放电电压高值
    unsigned char set_charge_channel;//[1..2]充电机通道
    unsigned char set_charge_voltage;//[V]充电机电压
    unsigned char set_charge_current;//[A]充电机电流
    unsigned char rudder_dead_zone_angle_degree;//[度]方向舵控制闭环时所用的死区角度数
    unsigned char master_config;
    unsigned char slave_config;//D0=0:充电机允许 D1=0:切换器允许 D2=1:电流传感器1允许 D3=1:电流传感器2允许 D4=0:气象站工作允许 D5=0:火箭工作允许
    unsigned char spare1;
    unsigned char spare2;
    unsigned char spare3;

    unsigned int spare4;//56字节
    unsigned int spare5;//60

    /*
     * wp_flag总航点数 这个作为自动驾驶下的模式选择
     * 0:巡航 255:guide 254:loiter 253:退出逗留模式 其他:该航点有效
     * 意思是需要更改航点列表中的航点 用该命令包中的航点数据代替对应编号的航点数据
     */
    unsigned char wp_flag;
    unsigned char wp_next;//下一个航点编号
    unsigned char spd;//[knot*0.1]航点设定航速
    unsigned char alt;//64

    unsigned int lng;//[度*0.00001]，航点GPS经度坐标，整型，精确到米
    unsigned int lat;//[度*0.00001]，航点GPS纬度坐标，整型，精确到米  //72字节

    unsigned char spare6;
    unsigned char spare7;
    unsigned char spare8;
    unsigned char checksum;

    /**************************/
    /******************************************************************************/

    //以下是通过命令包的位操作字节翻译后，需要得到的值
    unsigned char mmotor_on_pos;//电机启动时，遥控器的数值，也就是遥控器的最大值，因为费老师在地面站已经标定过了，所以并没有必要
    unsigned char mmotor_off_pos;//电机关闭时，遥控器的数值，也就是遥控器的最小值，因为费老师在地面站已经标定过了，所以并没有必要
    unsigned char rudder_left_pos;//方向舵处于最左边时，遥控器的数值，因为费老师在地面站已经标定过了，所以并没有必要
    unsigned char rudder_right_pos;//方向舵处于最右边时，遥控器的数值，因为费老师在地面站已经标定过了，所以并没有必要
    unsigned char rudder_mid_pos;//方向舵处于中间时，遥控器的数值，因为费老师在地面站已经标定过了，所以并没有必要

    unsigned char auto_work_mode;//自动模式下的工作模式，mission guide loiter
    unsigned char wp_guide_no;//设定的航点编号
    unsigned char spare;//76
};

/*
 * 一些通过地面站发送的参数需要记录在驾驶仪的，
 * 地面站只需要同步，不需要重新设置
 */
struct T_CONFIG
{
    unsigned char work_mode;
    unsigned char rud_p;
    unsigned char rud_i;
    unsigned char rud_d;
    unsigned char cte_p;
    unsigned char cte_i;
    unsigned char cte_d;
    unsigned char rudder_setup_reverse;//8
    unsigned char thruster_setup_reverse;

    unsigned char cruise_throttle_percent;//[0-100%]巡航油门百分比数
    unsigned char throttle_change_time;//[秒],油门改变百分之10所用的时间
    unsigned char arrive_radius;//[10米],到达半径
    unsigned char cte_max_degree;//[度],偏航距最大补偿角

    unsigned char rudder_left_pos;
    unsigned char rudder_right_pos;
    unsigned char rudder_mid_pos;//16

    unsigned char set_switch_channel;//[1..2]切换器通道
    unsigned char set_switch_low_limit;//[V],切换器放电电压低值
    unsigned char set_switch_high_limit;//[V],切换器放电电压高值
    unsigned char set_charge_channel;//[1..2]充电机通道
    unsigned char set_charge_voltage;//[V]充电机电压
    unsigned char set_charge_current;//[A]充电机电流
    unsigned char rudder_dead_zone_angle_degree;//[度]方向舵控制闭环时所用的死区角度数
    unsigned char total_wp_num;//40字节

    unsigned char current_target_wp_num;
    unsigned char spare0_char;
    unsigned char spare1_char;
    unsigned char spare2_char;//40字节
};

struct T_BOATPILOT_LOG_GLOBAL
{
    unsigned char bool_get_gcs2ap_cmd;//电台获取到地面站的命令包
    unsigned char bool_get_gcs2ap_waypoint;//电台获取到地面站的航点包
    unsigned char bool_gcs2ap_beidou;//0:不解析北斗数据，1:解析北斗接收的地面站数据包，并采用
    unsigned char bool_generator_on;//发电机 0:不工作，1:工作
    unsigned char bool_is_sending_wp_ap2gcs;//电台 在回传全部航点时，作为正在回传的标志
    unsigned char bool_beidou_get_gcs2ap_cmd;
    unsigned char bool_beidou_get_gcs2ap_waypoint;
    unsigned char bool_loiter_mode;//停留模式=true
    unsigned char bool_shutdown_master;//主控重启
    unsigned char bool_shutdown_slave;//副控重启
    unsigned char bool_rudder_calib_success;//方向舵极限值标定成功
    unsigned char bool_is_calib_rudder;//正在标定状态，方向舵可以随意摆动
    unsigned char turn_mode;//转弯方式，0:方向舵，1:差速 2:方向舵和差速同时混合转弯
    unsigned char s2m_generator_onoff_req_previous;//上一次副控向主控请求发电机的工作状态
    unsigned char radio_recv_packet_cnt;//电台 地面站发送给驾驶仪的数据包的计数，计数不同时，驾驶仪才接收地面站的数据，否则则舍弃
    unsigned char radio_recv_packet_cnt_previous;
    unsigned char udp_recv_packet_cnt;//只是用来记录主控接收到副控的udp的数据包计数，没有用作别的用途
    unsigned char wp_total_num;//wp_data[]数组记录了航点，wp_total_num则用来表示每次自动驾驶时地面站发给驾驶仪的航点总个数，最小是1
    unsigned char send_ap2gcs_wp_req;//驾驶仪 发送航点请求
    unsigned char ap2gcs_wp_cnt_previous;//驾驶仪 发送航点计数
    unsigned char ap2gcs_wp_cnt;
    unsigned char send_ap2gcs_real_req;//驾驶仪 发送实时数据请求
    unsigned char ap2gcs_real_cnt_previous;//驾驶仪 发送实时数据计数
    unsigned char ap2gcs_real_cnt;
    unsigned char send_ap2gcs_cmd_req;//驾驶仪 发送(回传)命令请求
    unsigned char ap2gcs_cmd_cnt_previous;//驾驶仪 发送(回传)命令计数
    unsigned char ap2gcs_cmd_cnt;
    unsigned char send_m2s_udp_req;//主控给副控发送udp数据包请求
    unsigned char m2s_udp_cnt_previous;//主控给副控发送udp数据包计数
    unsigned char m2s_udp_cnt;
    unsigned char bd_send_ap2gcs_wp_req;
    unsigned char bd_ap2gcs_wp_cnt_previous;
    unsigned char bd_ap2gcs_wp_cnt;
    unsigned char bd_send_ap2gcs_real_req;
    unsigned char bd_ap2gcs_real_cnt_previous;
    unsigned char bd_ap2gcs_real_cnt;
    unsigned char bd_send_ap2gcs_cmd_req;
    unsigned char bd_ap2gcs_cmd_cnt_previous;
    unsigned char bd_ap2gcs_cmd_cnt;
    unsigned char rudder_calib_cnt_previous;//标定方向舵的左极限右极限和中间值
    unsigned char launch_req_ack_cnt_previous;//副控确认火箭可发射后，向主控提出发射火箭请求
    unsigned char save_boatpilot_log_req;//保存无人船日志请求，这个是定时保存，每秒存储一个日志结构
    unsigned char wp_packet_cnt;//地面站发送给驾驶仪的第wp_packet_cnt个航点数据包
    unsigned char assign_config_req;//当驾驶仪接收到地面站的命令包时，驾驶仪认为需要更新配置(也就是一些参数设置)
    unsigned char assign_config_cnt_previous;
    unsigned char assign_config_cnt;
    unsigned char save_config_req;//驾驶仪中的config结构发生变化时，就提出保存config请求
    unsigned char set_switch_channel_previous;//切换器 上一次的通道
    unsigned char voltage_llim_previous;//切换器 放电电压最低值
    unsigned char voltage_hlim_previous;//切换器 放电电压最高值
    unsigned char bat0_is_discharing;//切换器 电池0通道正在放电
    unsigned char bat1_is_discharing;//切换器 电池1通道正在放电
    unsigned char charger_set_channel_previous;//充电机 上一次的通道
    unsigned char charger_set_voltage_previous;//充电机 上一次的电压
    unsigned char charger_set_current_previous;//充电机 上一次的电流
    unsigned char charge_start_previous;//充电机 上次的开机关机状态，只有检测到与当前状态不同时才改变
    unsigned char wp_next;//自动驾驶时，由地面站设置的或者改变的当前目标航点，最小值0
    unsigned char send_ap2gcs_wp_start_num;//发送航点数据包时，起始航点数，最小值0
    unsigned char send_ap2gcs_wp_end_num;//发送航点数据包时，航点数木，最小值1
    unsigned char send_ap2gcs_specific_wp_req;//地面站请求驾驶仪回传特定的某几个航点，不是回传全部航点
    unsigned char master_state;//主控的状态 D7:电台等待超时 D6:gps等待超时 D5:modbus继电器模拟量等待超时 D4:modbus码盘等待超时 D3:udp等待超时 D2:北斗等待超时 D1: D0:
    unsigned char slave_state;//副控的状态
    unsigned char slave_config_previous;//地面站配置副控，现在用来控制是否读取485的电流通道
    unsigned char spare0;//64字节

    short left_motor_voltage;//单位[0.1伏特]
    short right_motor_voltage;//单位[0.1伏特]
    short rudder_angle_degree;//单位[度] 范围[-45-+45度]这是通过码盘读回来的真实的方向舵的角度值，通过实时数据返回给地面站
    short cte_error_check_radian;//psi_r根据偏航距得到的修正方向舵角[-3.14*1000-+3.14*1000]
    short current_to_target_radian;//[-180*1000-+180*1000][0.001弧度]
    short command_radian;//[-180*1000-+180*1000][0.001弧度]
    short dir_target_degree;//[0.01度]当前位置与目标航点位置的方位角bearing angle
    short dir_nav_degree;//80字节//[0.01度]制导算法得到的导航目标航迹角course angle或者航向角heading angle

    int cte_distance_error;//84字节//[0.01米]

#if 0
    //float rudder_middle_position;//[0-720]方向舵标定时，定义的方向舵的中位的2倍，初始值设置为360线(驾驶仪认为是180度)，码盘的360度对应的是720线
    float rudder_middle_position;//方向舵处于中间位置时对应的码盘的读数
    float rudder_left_limit_position;//[0-360]方向舵标定时的左极限，初始默认180-45=135度
    float rudder_right_limit_position;//[0-360]方向舵标定时的右极限，初始默认180+45=225度
    float rudder_delta_fabs;//[0-90]20170508    [0-45]方向舵标定时，为保证向左和向右转动时左右极限值对称，故采用此变化范围
#else
    short rudder_middle_position;//方向舵处于中间位置时对应的码盘的读数
    short rudder_left_limit_position;//[0-720]方向舵标定时的左极限位置，对应的码盘读数
    short rudder_right_limit_position;//[0-720]方向舵标定时的右极限位置，对应的码盘读数
    short rudder_delta_fabs;//92字节//[0-90]方向舵标定时，为保证向左和向右转动时左右极限值对称，故有此变量
#endif
};

struct T_BOATPILOT_LOG
{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char stuffing;//填充字节，保证数据包字节数为4的整数倍

    struct GCS2AP_RADIO gcs2ap_radio;
    struct AP2GCS_REAL ap2gcs_real;
    struct T_BOATPILOT_LOG_GLOBAL global;//全局变量的需要记录的中间值
};

extern struct GCS_AP_WP gcs_ap_wp;
extern struct GCS_AP_WP ap2gcs_wp;
extern struct GCS2AP_CMD gcs2ap_cmd;
extern struct GCS2AP_CMD gcs2ap_cmd_return;
extern struct AP2GCS_REAL ap2gcs_real;

/*
 * 存放电台传过来的航点数据包，最大航点数为255
 */
#define MAX_WAYPOINT_NUM 255
extern struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

/*
 * 存放电台传过来的除了航点数据包的所有数据的数据结构
 */
extern struct GCS2AP_RADIO gcs2ap_radio_all;



int decode_gcs2ap_waypoint(struct WAY_POINT *ptr_wp_data, struct GCS_AP_WP *ptr_gcs2ap_wp);

#endif /* HEADERS_BOATLINK_H_ */
