/*
 * setup.cpp
 *
 *  Created on: 2017-10-1
 *      Author: wangbo
 */

#include "copter.h"

/*
 * 这个文件主要是missionplanner地面站初始设置中的函数，暂时不需要，
 * 因为我们已经配置好了，四旋翼，校准也放在驾驶仪里面写，跟地面站不交互
 */

#if CLI_ENABLED == ENABLED

#define PWM_CALIB_MIN 1000
#define PWM_CALIB_MAX 2000
#define PWM_HIGHEST_MAX 2200
#define PWM_LOWEST_MAX 1200
#define PWM_HIGHEST_MIN 1800
#define PWM_LOWEST_MIN 800

#endif
