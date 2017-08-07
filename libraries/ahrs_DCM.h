/*
 * ahrs_DCM.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef AHRS_DCM_H_
#define AHRS_DCM_H_

#include <inttypes.h>

#include <math.h>
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"

#include "compass.h"

#include "gps.h"
#include "imu.h"

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

class AP_DCM
{
public:
#if 1
	// Constructors
	AP_DCM(IMU &imu, GPS &gps, Compass &compass ) :
		_compass(compass),
		_gps(gps),
		_imu(imu),
		_dcm_matrix(1, 0, 0,
					0, 1, 0,
					0, 0, 1),
		_course_over_ground_x(0),
		_course_over_ground_y(1)
	{}
#endif

	//AP_DCM();


	// Accessors
	Vector3f	get_gyro(void) {return _omega_integ_corr; }		// We return the raw gyro vector corrected for bias
	Vector3f	get_accel(void) { return _accel_vector; }
	Matrix3f	get_dcm_matrix(void) {return _dcm_matrix; }
	Matrix3f	get_dcm_transposed(void) {Matrix3f temp = _dcm_matrix;  return temp.transpose();}

	void		set_centripetal(bool b) {_centripetal = b;}
	bool		get_centripetal(void) {return _centripetal;}
	void		set_compass(Compass &compass);

	float radians(float deg);
	float degrees(float rad);
	float constrain(float m,float a,float b);


	// Methods
	void 		update_DCM(float _G_Dt);

	float		get_health(void);

	int32_t		roll_sensor;					// Degrees * 100
	int32_t		pitch_sensor;					// Degrees * 100
	int32_t		yaw_sensor;						// Degrees * 100

	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians

	uint8_t 	gyro_sat_count;
	uint8_t 	renorm_sqrt_count;
	uint8_t 	renorm_blowup_count;

private:
	// Methods
	void 		read_adc_raw(void);
	void 		accel_adjust(void);
	float 		read_adc(int select);
	void 		matrix_update(float _G_Dt);
	void 		normalize(void);
	Vector3f 	renorm(Vector3f const &a, int &problem);
	void 		drift_correction(void);
	void 		euler_angles(void);

	// members
	Compass 	& _compass;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		&_gps;                     // note: this is a reference to a pointer owned by the caller

	IMU 		&_imu;

	Matrix3f	_dcm_matrix;

	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector
	Vector3f	_omega_P;					// Omega Proportional correction
	Vector3f 	_omega_I;					// Omega Integrator correction
	Vector3f 	_omega_integ_corr;			// Partially corrected Gyro_Vector data - used for centrepetal correction
	Vector3f 	_omega;						// Corrected Gyro_Vector data
	Vector3f 	_error_roll_pitch;
	Vector3f 	_error_yaw;
	float 		_errorCourse;
	float 		_course_over_ground_x; 		// Course overground X axis
	float 		_course_over_ground_y; 		// Course overground Y axis
	float		_health;
	bool		_centripetal;
};



#endif /* AHRS_DCM_H_ */
