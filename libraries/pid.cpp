/*
 * pid.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include "pid.h"

#ifndef  M_PI
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
#endif

AP_PID::AP_PID()
{

}
const float        AP_PID::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
float
AP_PID::get_pid(int32_t error, uint16_t dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {
		float derivative = (error - _last_error) / delta_time;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1/(2*M_PI*_fCut);
		derivative = _last_derivative +
		        (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = derivative;

		// add in derivative component
		output 				+= _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		output 				+= _integrator;
	}

	return output;
}

void
AP_PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}

int32_t AP_PID::get_p(int32_t error)
{
    return (float)error * _kp;
}

int32_t AP_PID::get_i(int32_t error, float dt)
{
    if(dt != 0) {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
    return _integrator;
}

int32_t        AP_PID:: get_d(int32_t error, float dt)
{
	if ((_kd != 0) && (dt != 0)) {
		_derivative = (error - _last_error) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
					  (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error            = error;
		_last_derivative    = _derivative;

		// add in derivative component
		return _kd * _derivative;
	}
	return 0;

}

int32_t AP_PID::get_pi(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt);
}


#if 0
float AP_PID::get_p()
{
   float  result = (_input * _kp);
    return result;
}

float AP_PID::get_i()
{
    if(!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += ((float)_input * _ki) * _dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        _pid_info.I = _integrator;
        return _integrator;
    }
    return 0;
}

float AP_PID::get_d()
{
    // derivative component
    _pid_info.D = (_kd * _derivative);
    return _pid_info.D;
}

float AP_PID::get_ff(float requested_rate)
{
    _pid_info.FF = (float)requested_rate * _ff;
    return _pid_info.FF;
}


float AP_PID::get_pi()
{
    return get_p() + get_i();
}

float AP_PID::get_pid()
{
    return get_p() + get_i() + get_d();
}
#endif
