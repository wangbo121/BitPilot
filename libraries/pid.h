/*
 * pid.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef PID_H_
#define PID_H_

#include <inttypes.h>
#include <math.h>		// for fabs()

//实例化的对象只对应(管理)一个 PID 控制器
class AP_PID {
public:
	AP_PID();

	float 	get_pid(int32_t error, uint16_t dt_ms, float scaler = 1.0);

	int32_t         get_pi(int32_t error, float dt);
	int32_t         get_p(int32_t error);
	int32_t         get_i(int32_t error, float dt);
	int32_t         get_d(int32_t input, float dt);

//	// get_pid - get results from pid controller
//	float       get_pid();
//	float       get_pi();
//	float       get_p();
//	float       get_i();
//	float       get_d();


	/// Reset the PID integrator
	///
	void	reset_I();

	void	set_kP(const float v)		{ _kp = v; }
	void	set_kI(const float v)		{ _ki = v; }
	void	set_kD(const float v)		{ _kd = v; }
	void	set_imax(const int16_t v)	{ _imax = v; }

	float	get_kP()			{ return _kp; }
	float	get_kI()			{ return _ki; }
	float	get_kD()			{ return _kd; }
	float	get_imax()			{ return _imax; }
    void        set_integrator(float i) {
        _integrator = i;
    }

	float	get_integrator() const	{ return _integrator; }

private:
	float				_kp;
	float				_ki;
	float				_kd;
	float				_imax;

	float				_integrator;		///< integrator value
	int32_t			_last_error;		///< last error for derivative
	float				_last_derivative; 	///< last derivative for low-pass filter
	float           _output;
	float           _derivative;

	/// Low pass filter cut frequency for derivative calculation.
	///
	static const float        _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
	// Examples for _filter:
	// f_cut = 10 Hz -> _filter = 15.9155e-3
	// f_cut = 15 Hz -> _filter = 10.6103e-3
	// f_cut = 20 Hz -> _filter =  7.9577e-3
	// f_cut = 25 Hz -> _filter =  6.3662e-3
	// f_cut = 30 Hz -> _filter =  5.3052e-3

	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	static const uint8_t _fCut = 20;
};

#endif /* PID_H_ */
