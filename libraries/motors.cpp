/*
 * motors.cpp
 *
 *  Created on: 2017-8-2
 *      Author: wangbo
 */

/*
 * 使用硬件抽象层必须包括的头文件和hal的extern声明
 */
#include "BIT_HAL.h"
extern const BIT_HAL::HAL& hal;


#include "motors.h"
// Constructor
BIT_Motors::BIT_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _roll_in(0.0f),
    _pitch_in(0.0f),
    _yaw_in(0.0f),
    _throttle_in(0.0f),
    _throttle_avg_max(0.0f),
    //_throttle_filter(),
    _spool_desired(DESIRED_SHUT_DOWN),
    _batt_voltage(0.0f),
    _batt_current(0.0f),
    _air_density_ratio(1.0f),
    _motor_map_mask(0),
    _motor_fast_mask(0)
{
    // init other flags
    _flags.armed = false;
    _flags.interlock = false;
    _flags.initialised_ok = false;

    // setup throttle filtering
    //_throttle_filter.set_cutoff_frequency(0.0f);
    //_throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;
};
void
BIT_Motors::output()
{
#if 0
    // capture desired roll, pitch, yaw and throttle from receiver
    ->calc_pwm();
    _pitch_in->calc_pwm();
    _throttle_in->calc_pwm();
    _yaw_in->calc_pwm();
#endif
}

void BIT_Motors::armed(bool arm)
{
    if (_flags.armed != arm) {
        _flags.armed = arm;
       // BIT_Notify::flags.armed = arm;
        if (!arm) {
            //save_params_on_disarm();
        }
    }
};

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void BIT_Motors::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
}

/*
  write to an output channel
 */
void BIT_Motors::rc_write(uint8_t chan, uint16_t pwm)
{
    /*
     * wangbo20170802
     * 这里应该就是输出给电机了
     */
    hal.rcout->write(chan, pwm);
}

/*
  set frequency of a set of channels
 */
void BIT_Motors::rc_set_freq(uint32_t mask, uint16_t freq_hz)
{
    mask = rc_map_mask(mask);
    if (freq_hz > 50) {
        _motor_fast_mask |= mask;
    }
    //hal.rcout->set_freq(mask, freq_hz);
    if ((_pwm_type == PWM_TYPE_ONESHOT ||
         _pwm_type == PWM_TYPE_ONESHOT125) &&
        freq_hz > 50 &&
        mask != 0) {
        // tell HAL to do immediate output
       // hal.rcout->set_output_mode(BIT_HAL::RCOutput::MODE_PWM_ONESHOT);
    } else if (_pwm_type == PWM_TYPE_BRUSHED) {
        //hal.rcout->set_output_mode(BIT_HAL::RCOutput::MODE_PWM_BRUSHED);
    }
}
#if 0
void BIT_Motors::rc_enable_ch(uint8_t chan)
{
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    //hal.rcout->enable_ch(chan);
}
#endif

/*
  map an internal motor mask to real motor mask
 */
uint32_t BIT_Motors::rc_map_mask(uint32_t mask) const
{
    uint32_t mask2 = 0;
    for (uint8_t i=0; i<32; i++) {
        uint32_t bit = 1UL<<i;
        if (mask & bit) {
            if ((i < BIT_MOTORS_MAX_NUM_MOTORS) && (_motor_map_mask & bit)) {
                // we have a mapped motor number for this channel
                mask2 |= (1UL << _motor_map[i]);
            } else {
                mask2 |= bit;
            }
        }
    }
    return mask2;
}

#if 0
// convert input in -1 to +1 range to pwm output
int16_t BIT_Motors::calc_pwm_output_1to1(float input, const SRV_Channel *servo)
{
    int16_t ret;

    input = constrain_float(input, -1.0f, 1.0f);

    if (servo->get_reversed()) {
        input = -input;
    }

    if (input >= 0.0f) {
        ret = ((input * (servo->get_output_max() - servo->get_trim())) + servo->get_trim());
    } else {
        ret = ((input * (servo->get_trim() - servo->get_output_min())) + servo->get_trim());
    }

    return constrain_int16(ret, servo->get_output_min(), servo->get_output_max());
}

// convert input in 0 to +1 range to pwm output
int16_t BIT_Motors::calc_pwm_output_0to1(float input, const SRV_Channel *servo)
{
    int16_t ret;

    input = constrain_float(input, 0.0f, 1.0f);

    if (servo->get_reversed()) {
        input = 1.0f-input;
    }

    ret = input * (servo->get_output_max() - servo->get_output_min()) + servo->get_output_min();

    return constrain_int16(ret, servo->get_output_min(), servo->get_output_max());
}

/*
  add a motor, setting up _motor_map and _motor_map_mask as needed
 */
void BIT_Motors::add_motor_num(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < BIT_MOTORS_MAX_NUM_MOTORS ) {
        uint8_t chan;
        SRV_Channel::Aux_servo_function_t function;
        if (motor_num < 8) {
            function = (SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_motor1+motor_num);
        } else {
            function = (SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_motor9+(motor_num-8));
        }
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (SRV_Channels::find_channel(function, chan) && chan != motor_num) {
            _motor_map[motor_num] = chan;
            _motor_map_mask |= 1U<<motor_num;
        }
    }
}
#endif
