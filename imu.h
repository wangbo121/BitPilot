/*
 * imu.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef IMU_H_
#define IMU_H_

#include <math.h>
#include <inttypes.h>

#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"

#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

class IMU
{

public:
	/// Constructor
	IMU() {}

	enum Start_style {
		COLD_START = 0,
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
	virtual void	init(Start_style style) = 0;

	/// Perform cold startup initialisation for just the accelerometers.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work.
	///
	virtual void	init_accel() = 0;

	/// Perform cold-start initialisation for just the gyros.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work
	///
	virtual void	init_gyro() = 0;

	/// Give the IMU some cycles to perform/fetch an update from its
	/// sensors.
	///
	/// @returns	True if some state was updated.
	///
	virtual bool	update(void) = 0;

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

protected:
	/// Most recent accelerometer reading obtained by ::update
	Vector3f		_accel;

	/// Most recent gyro reading obtained by ::update
	Vector3f		_gyro;
};



#endif /* IMU_H_ */
