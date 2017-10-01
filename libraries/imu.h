/*
 * imu.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef IMU_H_
#define IMU_H_

#include <inttypes.h>

#include <math.h>
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"

#include <math.h>
#include <inttypes.h>



#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

class IMU
{

public:
	/// Constructor
	//IMU() {}
	IMU() ;

	enum Start_style {
		COLD_START ,
		WARM_START
	};

	/// Perform startup initialisation.
	///
	/// Called to initialise the state of the IMU.
	///
	/// For COLD_START, implementations using real sensors can assume
	/// that the airframe is stationary and nominally oriented.
	///
	/// For WARM_START, no assumptions should be made about the
	/// orientation or motion of the airframe.  Calibration should be
	/// as for the previous COLD_START call.
	///
	/// @param style	The initialisation startup style.
	///
	 void	init() ;

	/// Perform cold startup initialisation for just the accelerometers.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work.
	///
	 void	init_accel() ;

	/// Perform cold-start initialisation for just the gyros.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work
	///
	 void	init_gyro() ;

	/// Give the IMU some cycles to perform/fetch an update from its
	/// sensors.
	///
	/// @returns	True if some state was updated.
	///
	 bool	update(void) ;

	/// Fetch the current gyro values
	///
	/// @returns	vector of rotational rates in radians/sec
	///
	Vector3f		get_gyro(void) { return _gyro; }

	/// Fetch the current accelerometer values
	///
	/// @returns	vector of current accelerations in m/s/s
	///
	Vector3f		get_accel(void) { return _accel; }

	/// A count of bad sensor readings
	///
	/// @todo This should be renamed, as there's no guarantee that sensors
	///       are using ADCs, etc.
	///
	uint8_t 	adc_constraints;

//protected:
public:
	/// Most recent accelerometer reading obtained by ::update
	Vector3f		_accel;

	/// Most recent gyro reading obtained by ::update
	Vector3f		_gyro;//[rad/s]单位是弧度每秒

public:




	// for jason
	float		gx()				{ return _sensor_cal[0]; }
	float		gy()				{ return _sensor_cal[1]; }
	float		gz()				{ return _sensor_cal[2]; }
	float		ax()				{ return _sensor_cal[3]; }
	float		ay()				{ return _sensor_cal[4]; }
	float		az()				{ return _sensor_cal[5]; }

	void		ax(const float v)		{ _sensor_cal[3] = v; }
	void		ay(const float v)		{ _sensor_cal[4] = v; }
	void		az(const float v)		{ _sensor_cal[5] = v; }


private:

	float    _sensor_cal[6];    ///< Calibrated sensor offsets

	 void        _init_accel();  ///< no-save implementation
	 void        _init_gyro();   ///< no-save implementation

	float 		        _sensor_compensation(uint8_t channel, int temp) const;
	float		        _sensor_in(uint8_t channel, int temperature);

	// constants
	static const uint8_t	_sensors[6];            ///< ADC channel mappings for the sensors
	static const int8_t    	_sensor_signs[6];       ///< ADC result sign adjustment for sensors
	static const uint8_t	_gyro_temp_ch = 3; 		///< ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3]; ///< Temperature compensation curve for the gyro

	// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
	// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
	// Tested value : 418
	//
	static const float      _gravity ;       ///< 1G in the raw data coming from the accelerometer
													// Value based on actual sample data from 20 boards
	static const float      _accel_scale ; ///< would like to use _gravity here, but cannot

	// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s, 0.8mV/ADC step => 0.8/3.33 .4
	// Tested values : 0.4026, ?, 0.4192
	//
	static const float      _gyro_gain_x ;     // X axis Gyro gain
	static const float      _gyro_gain_y ;    // Y axis Gyro gain
	static const float      _gyro_gain_z ;    // Z axis Gyro gain

	// Maximum possible value returned by an offset-corrected sensor channel
	//
	static const float      _adc_constraint ;

	// Gyro and Accelerometer calibration criterial
	//
	static const float		_gyro_total_cal_change ;		// Experimentally derived - allows for some minor motion
	static const float		_gyro_max_cal_offset ;
	static const float		_accel_total_cal_change;
	static const float		_accel_max_cal_offset ;
};



#endif /* IMU_H_ */
