/*
 * Parameters.cpp
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#include "copter.h"
#include "Parameters.h"



/*
 * 把这里修改结束后，还需要把GCS_Mavlink.cpp中_queued_parameter_count = ;这句代码改为param_all的数组内元素的个数
 */
T_PARAM param_all[]={//value                                                key                                                              name                                                 *next param
										{113.0,                                              0,                                                                 "SYSID_SW_MREV",                          &param_all[1]},
										{10.0,                                                 1,                                                                "SYSID_SW_TYPE",                             &param_all[2]},
										{WP_RADIUS_DEFAULT,                   k_param_waypoint_radius ,                   "WP_RADIUS",                                   &param_all[3]},
										{0.0,                                                    255,                                                            "JUNK_END",                                       NULL}
										};

