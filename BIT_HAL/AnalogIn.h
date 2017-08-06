/*
 * AnalogIn.h
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */

#ifndef ANALOGIN_H_
#define ANALOGIN_H_

#include "BIT_HAL_Namespace.h"
#include <inttypes.h>

#define PX4_ANALOG_MAX_CHANNELS 8

// these are virtual pins that read from the ORB
#define PX4_ANALOG_BATTERY_VOLTAGE_PIN 100
#define PX4_ANALOG_BATTERY_CURRENT_PIN 101

#define PX4_ANALOG_AIRSPEED_PIN         11
#define PX4_ANALOG_ANALOG2_PIN          12 // on SPI port pin 3
#define PX4_ANALOG_ANALOG3_PIN          13 // on SPI port pin 4

class AP_HAL::AnalogSource {
public:
     float read_average() ;
     float read_latest() ;
     void set_pin(uint8_t p) ;

    // optionally allow setting of a pin that stops the device from
    // reading. This is needed for sonar devices where you have more
    // than one sonar, and you want to stop them interfering with each
    // other. It assumes that if held low the device is stopped, if
    // held high the device starts reading.
     void set_stop_pin(uint8_t p) ;

    // optionally allow a settle period in milliseconds. This is only
    // used if a stop pin is set. If the settle period is non-zero
    // then the analog input code will wait to get a reading for that
    // number of milliseconds. Note that this will slow down the
    // reading of analog inputs.
     void set_settle_time(uint16_t settle_time_ms) ;

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
     float voltage_average() ;

    // return a voltage from 0.0 to 5.0V, assuming a ratiometric
    // sensor
     float voltage_average_ratiometric() ;

     AnalogSource(int16_t pin, float initial_value);






 private:
	 // what pin it is attached to
	 int16_t _pin;

	 // what value it has
	 float _value;
	 float _latest_value;
	 uint8_t _sum_count;
	 float _sum_value;
	 void _add_value(float v);
};

class AP_HAL::AnalogIn {
public:
	AnalogIn();
     void init() ;
     AP_HAL::AnalogSource* channel(int16_t n) ;

private:
     static int _adc_fd;
	 static int _battery_handle;
	 static uint64_t _battery_timestamp;
	 static AnalogSource* _channels[PX4_ANALOG_MAX_CHANNELS];
	 static void _analogin_timer(uint32_t now);
	 static uint32_t _last_run;
};

#define ANALOG_INPUT_BOARD_VCC 254
#define ANALOG_INPUT_NONE 255



#endif /* ANALOGIN_H_ */
