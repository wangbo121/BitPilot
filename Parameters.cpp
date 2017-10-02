/*
 * Parameters.cpp
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#include "copter.h"
#include "Parameters.h"

uint8_t param_all_cnt=16;

/*
 * 把这里修改结束后，还需要把GCS_Mavlink.cpp中_queued_parameter_count = ;这句代码改为param_all的数组内元素的个数
 * 后来改成不需要改了，改上面的param_all_cnt就可以了
 */
T_PARAM param_all[]={//value                                                key                                                              name                                                 *next param
										{113.0,                                              0,                                                                 "SYSID_SW_MREV",                          &param_all[1]},
										{10.0,                                                 1,                                                                "SYSID_SW_TYPE",                             &param_all[2]},
										{WP_RADIUS_DEFAULT,                   k_param_waypoint_radius ,                   "WP_RADIUS",                                   &param_all[3]},
										{3.69,                                                 k_param_p_stabilize_roll ,                       "STB_ROLL_P",                                   &param_all[4]},
										{3.69,                                                 k_param_p_stabilize_pitch ,                     "STB_PITCH_P",                                 &param_all[5]},
										{3.69,                                                 k_param_p_stabilize_yaw ,                       "STB_YAW_P",                                   &param_all[6]},
										{0.15,                                                 k_param_pid_rate_roll_p ,                        "ROLL_RATE_P",                                 &param_all[7]},
										{0.1,                                                 k_param_pid_rate_roll_i ,                        "ROLL_RATE_I",                                  &param_all[8]},
										{0.004,                                                 k_param_pid_rate_roll_d ,                        "ROLL_RATE_D",                                 &param_all[9]},
										{0.15,                                                 k_param_pid_rate_pitch_p ,                        "PITCH_RATE_P",                             &param_all[10]},
										{0.1,                                                 k_param_pid_rate_pitch_i ,                        "PITCH_RATE_I",                              &param_all[11]},
										{0.004,                                                 k_param_pid_rate_pitch_d ,                        "PITCH_RATE_D",                             &param_all[12]},
										{0.02,                                                 k_param_pid_rate_yaw_p ,                        "YAW_RATE_P",                                 &param_all[13]},
										{0.2,                                                 k_param_pid_rate_yaw_i ,                        "YAW_RATE_I",                                  &param_all[14]},
										{0.0,                                                 k_param_pid_rate_yaw_d ,                        "YAW_RATE_D",                                 &param_all[15]},
										{0.0,                                                    255,                                                            "JUNK_END",                                       NULL}
										};

