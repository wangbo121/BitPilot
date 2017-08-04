/*
 * rc_channel.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include <iostream>
#include <math.h>
#include "rc_channel.h"

#define ANGLE 0
#define RANGE 1

// setup the control preferences
void
BIT_RC_Channel::set_range(int low, int high)
{
	_type 	= RANGE;
	_high 	= high;
	_low 	= low;
}

void
BIT_RC_Channel::set_angle(int angle)
{
	_type 	= ANGLE;
	_high 	= angle;
}

void
BIT_RC_Channel::set_reverse(uint8_t reverse)
{
	if (reverse) _reverse = -1;
	else _reverse = 1;
}

bool
BIT_RC_Channel::get_reverse(void)
{
	if (_reverse==-1) return 1;
	else return 0;
}

void
BIT_RC_Channel::set_filter(bool filter)
{
	_filter = filter;
}

// call after first read
void
BIT_RC_Channel::trim()
{
	radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
void
BIT_RC_Channel::set_pwm(int pwm)
{
	//std::cout<<"pwm="<<pwm<<std::endl;
	radio_in = pwm;
	control_in = pwm_to_angle();
}

int
BIT_RC_Channel::control_mix(float value)
{
	return (1 - fabs(control_in / _high)) * value + control_in;
}

// are we below a threshold?
bool
BIT_RC_Channel::get_failsafe(void)
{
	return (radio_in < (radio_min - 50));
}

// returns just the PWM without the offset from radio_min
void
BIT_RC_Channel::calc_pwm(void)
{
	//std::cout<<"radio_min="<<radio_min<<std::endl;
	//std::cout<<"radio_max="<<radio_max<<std::endl;
	//std::cout<<"servo_out="<<servo_out<<std::endl;

	pwm_out 	= angle_to_pwm();
	radio_out 	= pwm_out + radio_trim;

	radio_out = constrain(radio_out, radio_min, radio_max);
}

// ------------------------------------------

void
BIT_RC_Channel::load_eeprom(void)
{
	//radio_min 	= eeprom_read_word((uint16_t *)	_address);
	//radio_max	= eeprom_read_word((uint16_t *)	(_address + 2));
	load_trim();
}

void
BIT_RC_Channel::save_eeprom(void)
{
	//eeprom_write_word((uint16_t *)	_address, 			radio_min);
	//eeprom_write_word((uint16_t *)	(_address + 2), 	radio_max);
	save_trim();
}

// ------------------------------------------
void
BIT_RC_Channel::save_trim(void)
{
	//eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
	//_ee.write_int((_address + 4), 	radio_trim);
}

void
BIT_RC_Channel::load_trim(void)
{
	//radio_trim 	= eeprom_read_word((uint16_t *)	(_address + 4));
	//_ee.write_int((_address + 4), 	radio_trim);
}

// ------------------------------------------

void
BIT_RC_Channel::zero_min_max()
{
	radio_min = radio_max = radio_in;
}

void
BIT_RC_Channel::update_min_max()
{
	//radio_min = min(radio_min, radio_in);
	//radio_max = max(radio_max, radio_in);
}

// ------------------------------------------

int16_t
BIT_RC_Channel::pwm_to_angle()
{

	int16_t radio_trim_high = radio_trim + dead_zone;
	int16_t radio_trim_low  = radio_trim - dead_zone;

	// prevent div by 0
	if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
		return 0;


	if(radio_in < radio_trim)
		return  ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_trim - radio_min);
	else
		return  ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_max  - radio_trim);

		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim));
		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_trim - radio_min));
}


int16_t
BIT_RC_Channel::angle_to_pwm()
{
	// setup the control preferences

	//set_angle(4500);
	//std::cout<<"_high="<<_high<<std::endl;

	if(servo_out > 0)
	{
		//std::cout<<"servo_out > 0"<<std::endl;
		return ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;

	}
	else
		return ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;


		//return (((float)servo_out / (float)_high) * (float)(radio_max - radio_trim));
		//return (((float)servo_out / (float)_high) * (float)(radio_trim - radio_min));
}

// ------------------------------------------

int16_t
BIT_RC_Channel::pwm_to_range()
{
	//return (_low + ((_high - _low) * ((float)(radio_in - radio_min) / (float)(radio_max - radio_min))));
	return (_low + ((long)(_high - _low) * (long)(radio_in - radio_min)) / (long)(radio_max - radio_min));
}

int16_t
BIT_RC_Channel::range_to_pwm()
{
	//return (((float)(servo_out - _low) / (float)(_high - _low)) * (float)(radio_max - radio_min));
	return ((long)(servo_out - _low) * (long)(radio_max - radio_min)) / (long)(_high - _low);
}

// ------------------------------------------

float
BIT_RC_Channel::norm_input()
{
	if(radio_in < radio_trim)
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
	else
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
}

float
BIT_RC_Channel::norm_output()
{
	if(radio_out < radio_trim)
		return (float)(radio_out - radio_trim) / (float)(radio_trim - radio_min);
	else
		return (float)(radio_out - radio_trim) / (float)(radio_max  - radio_trim);
}

float
BIT_RC_Channel::constrain(float m,float a,float b)
{
	if(m<=a)        m=a;
	else if(m>=b)   m=b;

	return m;
}

