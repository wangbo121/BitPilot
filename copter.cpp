/*
 * copter.cpp
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

#include "copter.h"

const BIT_HAL::HAL& hal = BIT_HAL::get_HAL();

/*
 * 构造函数是没有返回值的
 */
Copter::Copter(void)
{

}

void Copter::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
#if 0
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
#endif
}

Copter copter;
