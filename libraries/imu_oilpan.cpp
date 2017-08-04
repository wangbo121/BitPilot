/*
 * imu_oilpan.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "imu.h"
#include "imu_oilpan.h"
// XXX secret knowledge about the APM/oilpan wiring
//
#define A_LED_PIN   37
#define C_LED_PIN   35

// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t BIT_IMU_Oilpan::_sensors[6]        = { 1, 2, 0, 4, 5, 6};	// Channel assignments on the APM oilpan
const int8_t  BIT_IMU_Oilpan::_sensor_signs[6]	= {	1,-1,-1, 1,-1,-1};  // Channel orientation vs. normal

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
//
const float   BIT_IMU_Oilpan::_gyro_temp_curve[3][3] = {
	{1658,0,0},			// Values to use if no temp compensation data available
	{1658,0,0},			// Based on average values for 20 sample boards
	{1658,0,0}
};

void
BIT_IMU_Oilpan::init()
{
}

/**************************************************/

void
BIT_IMU_Oilpan::init_gyro()
{
    _init_gyro();

}

void
BIT_IMU_Oilpan::_init_gyro()
{
	int flashcount = 0;
	int tc_temp;
	float adc_in;
	float prev[3] = {0,0,0};
	float total_change;
	float max_offset;

}

void
BIT_IMU_Oilpan::init_accel()
{
    _init_accel();
}

void
BIT_IMU_Oilpan::_init_accel()
{
	int flashcount = 0;
	float adc_in;
	float prev[6] = {0,0,0};
	float total_change;
	float max_offset;


}

/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------

float
BIT_IMU_Oilpan::_sensor_compensation(uint8_t channel, int temperature) const
{
    // do gyro temperature compensation
    if (channel < 3) {

        return  _gyro_temp_curve[channel][0] +
                _gyro_temp_curve[channel][1] * temperature +
                _gyro_temp_curve[channel][2] * temperature * temperature;
    }

    // do fixed-offset accelerometer compensation
    return 2041;    // Average raw value from a 20 board sample
}

float
BIT_IMU_Oilpan::_sensor_in(uint8_t channel, int temperature)
{
    float   adc_in;

    /*
     * 这里是写从传感器设备把数据读入到adc_in中
     */
    return adc_in;
}


bool
BIT_IMU_Oilpan::update(void)
{
	int tc_temp;
	//int tc_temp = _adc->Ch(_gyro_temp_ch);
	//上面这句话是读取imu的数据，然后下面赋值

	// convert corrected gyro readings to delta acceleration
	//
	_gyro.x = ToRad(_gyro_gain_x) * _sensor_in(0, tc_temp);
	_gyro.y = ToRad(_gyro_gain_y) * _sensor_in(1, tc_temp);
	_gyro.z = ToRad(_gyro_gain_z) * _sensor_in(2, tc_temp);

	// convert corrected accelerometer readings to acceleration
	//
	_accel.x = _accel_scale * _sensor_in(3, tc_temp);
	_accel.y = _accel_scale * _sensor_in(4, tc_temp);
	_accel.z = _accel_scale * _sensor_in(5, tc_temp);

	// always updated
	return true;
}

