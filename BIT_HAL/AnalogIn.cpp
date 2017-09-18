/*
 * AnalogIn.cpp
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */


#include "BIT_HAL.h"
#include "AnalogIn.h"

#include <stdio.h>

#ifdef LINUX_OS
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#define ANLOGIN_DEBUGGING 0

// the airspeed port has some additional scaling applied to allow it
// to go above 3.3V. This value was found by checking the ADC output
// for a range of known inputs from 1.3V to 5.0V
#define PX4_AIRSPEED_VOLTAGE_SCALING (6.76f/4096.0f)

// pin4 in the SPI port is analog input 13, marked as analog3 on the
// PX4IO schematic v1.3, and is scaled quite strangely
#define PX4_ANALOG3_VOLTAGE_SCALING (16.88f/4096.0f)

#define PX4_VOLTAGE_SCALING (3.3f/4096.0f)

extern const AP_HAL::HAL& hal;


AP_HAL::AnalogSource::AnalogSource(int16_t pin, float initial_value) :
	_pin(pin),
    _value(initial_value)
{
}

float AP_HAL::AnalogSource::read_average()
{

}

float AP_HAL::AnalogSource::read_latest()
{
    return _latest_value;
}

/*
  return voltage in Volts
 */
float AP_HAL::AnalogSource::voltage_average()
{
    if (_pin == PX4_ANALOG_AIRSPEED_PIN) {
        return PX4_AIRSPEED_VOLTAGE_SCALING * read_average();
    }
    if (_pin == PX4_ANALOG_ANALOG3_PIN) {
        return PX4_ANALOG3_VOLTAGE_SCALING * read_average();
    }
    return PX4_VOLTAGE_SCALING * read_average();
}

void AP_HAL::AnalogSource::set_pin(uint8_t pin)
{

}

void AP_HAL::AnalogSource::_add_value(float v)
{
    _latest_value = v;
    _sum_value += v;
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}


AP_HAL::AnalogIn::AnalogIn()
{}

void AP_HAL::AnalogIn::init()
{

}

/*
  called at 1kHz
 */
void AP_HAL::AnalogIn::_analogin_timer(uint32_t now)
{
}

AP_HAL::AnalogSource* AP_HAL::AnalogIn::channel(int16_t pin)
{

    return NULL;
}

