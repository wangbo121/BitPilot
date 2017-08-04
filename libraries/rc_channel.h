/*
 * rc_channel.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef RC_CHANNEL_H_
#define RC_CHANNEL_H_

//#include <AP_Common.h>
#include <stdint.h>

/// @class	AP_RC_Channel
/// @brief	Object managing one RC channel
class AP_RC_Channel{
  public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the channel trim parameters.
	/// @param name     Optional name for the group.
	///
	AP_RC_Channel(uint16_t address) :
		_address(address),
		_high(1),
		_filter(true),
		_reverse(1),
		dead_zone(0),
		scale_output(1.0)
	{}

	AP_RC_Channel() :
		_high(1),
		_filter(true),
		_reverse(1),
		dead_zone(0),
		scale_output(1.0)
	{}

	// setup min and max radio values in CLI
	void 		update_min_max();
	void 		zero_min_max();

	// startup
	void 		load_eeprom(void);
	void 		save_eeprom(void);
	void 		save_trim(void);
	void 		load_trim(void);
	void		set_filter(bool filter);

	// setup the control preferences
	void 		set_range(int low, int high);
	void 		set_angle(int angle);
	void 		set_reverse(uint8_t reverse);
	bool		get_reverse(void);

	// read input from APM_RC - create a control_in value
	void 		set_pwm(int pwm);

	// pwm is stored here
	int16_t		radio_in;//这个是从rc_channel类读取过来的数值，范围是1000～2000

	// call after first set_pwm
	void 		trim();

	// did our read come in 50µs below the min?
	bool		get_failsafe(void);

	// value generated from PWM
	int16_t 	control_in;//这个把radio_in这个变量，通过pwm_to_angle转为角度值的控制量输入
	int16_t 	dead_zone; // used to keep noise down and create a dead zone.

	int			control_mix(float value);

	// current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
	int16_t 	servo_out;//这个是最终输出给舵机的角度值-45～45

	// generate PWM from servo_out value
	void 		calc_pwm(void);

	float constrain(float m,float a,float b);

	// PWM is without the offset from radio_min
	int16_t 	pwm_out;//这个的范围是-500～500
	int16_t 	radio_out;//这个范围是1000～2000

	int16_t		radio_min;
	int16_t		radio_trim;
	int16_t		radio_max;

	// includes offset from PWM
	//int16_t 	get_radio_out(void);

	int16_t		pwm_to_angle();
	float		norm_input();
	float		norm_output();
	int16_t		angle_to_pwm();
	int16_t		pwm_to_range();
	int16_t		range_to_pwm();

	float		scale_output;

  private:
	bool		_filter;
	int8_t 		_reverse;
	int16_t		_address;
	bool 		_type;
	int16_t 	_high;
	int16_t 	_low;

};






#endif /* RC_CHANNEL_H_ */