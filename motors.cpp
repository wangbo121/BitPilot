/*
 * motors.cpp
 *
 *  Created on: 2017-10-1
 *      Author: wangbo
 */

#include "copter.h"



// write out the servo PWM values
// ------------------------------
void Copter::set_servos_4()
{

}

void Copter::motors_output()
{
	int16_t             motor_out[AP_MOTORS_MAX_NUM_MOTORS];

    //int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float              _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)

#if 0
    //这个是x型的
    /*
     * 这里给factor赋值-1 0 或者1
     */
    _roll_factor[0]  =  -1;  _pitch_factor[0]  = +1;  _yaw_factor[0]  = +1;
    _roll_factor[1]  =  -1;  _pitch_factor[1]  =  -1;  _yaw_factor[1]  =  -1;
    _roll_factor[2]  = +1;  _pitch_factor[2]  =  -1;  _yaw_factor[2]  = +1;
    _roll_factor[3]  = +1;  _pitch_factor[3]  = +1;  _yaw_factor[3]  =  -1;
#else

    //这个是针对+型机架的系数
	_roll_factor[0]  =  -1;  _pitch_factor[0]  =   0;  _throttle_factor[0]= +1;  _yaw_factor[0]  = +1;
	_roll_factor[1]  = +1;  _pitch_factor[1]  =   0;  _throttle_factor[1]= +1;  _yaw_factor[1]  = +1;
	_roll_factor[2]  =   0;  _pitch_factor[2]  = +1;  _throttle_factor[2]= +1;  _yaw_factor[2]  =  -1;
	_roll_factor[3]  =   0;  _pitch_factor[3]  =  -1;  _throttle_factor[3]= +1;  _yaw_factor[3]  =  -1;
#endif

	g.channel_roll.calc_pwm();
	g.channel_pitch.calc_pwm();
	g.channel_rudder.calc_pwm();
	g.channel_throttle.calc_pwm();

	DEBUG_PRINTF("motors_output    :    g.channel_roll.servo_out = %d\n",g.channel_roll.servo_out);
	DEBUG_PRINTF("motors_output    :    g.channel_pitch.servo_out = %d\n",g.channel_pitch.servo_out);
	DEBUG_PRINTF("motors_output    :    g.channel_rudder.servo_out = %d\n",g.channel_rudder.servo_out);
	DEBUG_PRINTF("motors_output    :    g.channel_throttle.servo_out = %d\n",g.channel_throttle.servo_out);

	DEBUG_PRINTF("motors_output    :    g.channel_roll.pwm_out = %d\n",g.channel_roll.pwm_out);
	DEBUG_PRINTF("motors_output    :    g.channel_pitch.pwm_out = %d\n",g.channel_pitch.pwm_out);
	DEBUG_PRINTF("motors_output    :    g.channel_rudder.pwm_out = %d\n",g.channel_rudder.pwm_out);
	DEBUG_PRINTF("motors_output    :    g.channel_throttle.pwm_out = %d\n",g.channel_throttle.pwm_out);

	DEBUG_PRINTF("motors_output    :    g.channel_roll.radio_out = %d\n",g.channel_roll.radio_out);
	DEBUG_PRINTF("motors_output    :    g.channel_pitch.radio_out = %d\n",g.channel_pitch.radio_out);
	DEBUG_PRINTF("motors_output    :    g.channel_rudder.radio_out = %d\n",g.channel_rudder.radio_out);
	DEBUG_PRINTF("motors_output    :    g.channel_throttle.radio_out = %d\n",g.channel_throttle.radio_out);

	/*
	 * 20170930切记我们输出需要的是throttle的radio_out，而其他的都是需要的pwm_out
	 */

	for(int i=0;i<4;i++)
	{

		/*
		 * 一定要注意这里的throttle是用的radio_out，radio_out=pwm_out+radio_trim，
		 * calc是把servo_out的-4500～+4500转为pwm的-500～+500
		 * radio_out=pwm_out+radio_trim=pwm_out+1500 radio_out的范围是1000-2000
		 */
		motor_out[i]=g.channel_throttle.radio_out*_throttle_factor[i]+ \
								g.channel_roll.pwm_out*_roll_factor[i]+\
								g.channel_pitch.pwm_out* _pitch_factor[i]+\
								g.channel_rudder.pwm_out * _yaw_factor[i];
	}

#ifdef LINUX_OS
	memcpy(motor_out_flightgear,motor_out,sizeof(motor_out));//模型fdm需要的四个输入量就是电机的1000-2000的信号量
#endif

	for(int i=0;i<4;i++)
	{
		g._rc.output_ch_pwm(i,motor_out[i]);//把控制量输出到all_external_device_output
		//或者用下面的motors也是可以的
		//motors.rc_write(i,motor_out[i]);
		//std::cout<<"motor_out_flightgear["<<i<<"]="<<motor_out_flightgear[i]<<std::endl;
		//sleep(1);
	}
}
