/*
 * radio.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include <inttypes.h>

int8_t failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling

#include "copter.h"

void Copter::init_rc_in()
{
	/*
		 * 1 RC_Channel对象的内部变量的初始化
		 * 这个在angle_to_pwm,pwm_to_angle等中都有用
		 */
		g.channel_roll.radio_min = 1000;
		g.channel_pitch.radio_min = 1000;
		g.channel_throttle.radio_min = 1000;
		g.channel_rudder.radio_min = 1000;
		g.rc_5.radio_min = 1000;
		g.rc_6.radio_min = 1000;
		g.rc_7.radio_min = 1000;
		g.rc_8.radio_min = 1000;

		g.channel_roll.radio_max = 2000;
		g.channel_pitch.radio_max = 2000;
		g.channel_throttle.radio_max = 2000;
		g.channel_rudder.radio_max = 2000;
		g.rc_5.radio_max = 2000;
		g.rc_6.radio_max = 2000;
		g.rc_7.radio_max = 2000;
		g.rc_8.radio_max = 2000;

		g.channel_roll.radio_trim = 1500;
		g.channel_pitch.radio_trim = 1500;
		//g.channel_throttle.radio_trim = 1500;
		g.channel_rudder.radio_trim = 1500;
		// 3 is not trimed  这里arducopter中注释说，throttle是没有trim的，但是没有的话，我的运行就有错误，到底是有没有呢
		//20170814发现在初始化的时候油门通道是不一样的，用的是set_range
		g.rc_5.radio_trim = 1500;//下面这些一般都是舵机，都是有trim中立位的，只有油门是没有中立值的
		g.rc_6.radio_trim = 1500;
		g.rc_7.radio_trim = 1500;
		g.rc_8.radio_trim = 1500;


		g.channel_roll.set_reverse(0);//不取反
		g.channel_pitch.set_reverse(0);
		g.channel_rudder.set_reverse(0);
		g.channel_throttle.set_reverse(0);
		g.rc_5.set_reverse(0);

		// set rc dead zones
		g.channel_roll.dead_zone 	= 60;
		g.channel_pitch.dead_zone 	= 60;
		g.channel_throttle.dead_zone = 6;
		g.channel_rudder.dead_zone 	= 60;

		g.channel_roll.dead_zone 	= 0;
		g.channel_pitch.dead_zone 	= 0;
		g.channel_throttle.dead_zone = 0;
		g.channel_rudder.dead_zone 	= 0;


    // set rc channel ranges
    g.channel_roll.set_angle(4500);//set_angle做了两件事，一是把_high变量赋值，一是设置type是角度[-4500~+4500]还是范围[0~+1000]
    g.channel_pitch.set_angle(4500);
    g.channel_throttle.set_range(0, 1000);//注意这里是set_range;
    g.channel_rudder.set_angle(4500);

    //set auxiliary ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);
}

void Copter::init_rc_out()
{
#if 0

	AP_RC.OutputCh(CH_1, 	g.channel_roll.radio_trim);					// Initialization of servo outputs
	AP_RC.OutputCh(CH_2, 	g.channel_pitch.radio_trim);
	AP_RC.OutputCh(CH_3, 	g.channel_throttle.radio_min);
	AP_RC.OutputCh(CH_4, 	g.channel_rudder.radio_trim);

	AP_RC.OutputCh(CH_5, 	g.rc_5.radio_trim);
	AP_RC.OutputCh(CH_6, 	g.rc_6.radio_trim);
	AP_RC.OutputCh(CH_7,   g.rc_7.radio_trim);
    AP_RC.OutputCh(CH_8,   g.rc_8.radio_trim);

	AP_RC.Init();		// APM Radio initialization

	AP_RC.OutputCh(CH_1, 	g.channel_roll.radio_trim);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	g.channel_pitch.radio_trim);
	APM_RC.OutputCh(CH_3, 	g.channel_throttle.radio_min);
	APM_RC.OutputCh(CH_4, 	g.channel_rudder.radio_trim);

	APM_RC.OutputCh(CH_5, 	g.rc_5.radio_trim);
	APM_RC.OutputCh(CH_6, 	g.rc_6.radio_trim);
	APM_RC.OutputCh(CH_7,   g.rc_7.radio_trim);
    APM_RC.OutputCh(CH_8,   g.rc_8.radio_trim);
#endif
}

void Copter::read_radio()
{
	/*
	 * //set_pwm做两件事，
	 * 1是给radion_in赋值，作为从rc_channel读取回来的数
	 * 2是把control_in = pwm_to_angle(radio)，也就是把读取回来的pwm转为-4500～+4500角度控制值
	 */
	g.channel_roll.set_pwm(ap_rc.input_ch(CH_1));
	g.channel_pitch.set_pwm(ap_rc.input_ch(CH_2));
	g.channel_throttle.set_pwm(ap_rc.input_ch(CH_3));
	g.channel_rudder.set_pwm(ap_rc.input_ch(CH_4));
	g.rc_5.set_pwm(ap_rc.input_ch(CH_5));

	/*
	 * 20170818已测试
	 */
	/*
	std::cout<<"g.channel_roll.control_in="<<g.channel_roll.control_in<<std::endl;
	std::cout<<"g.channel_pitch.control_in="<<g.channel_pitch.control_in<<std::endl;
	std::cout<<"g.channel_throttle.control_in="<<g.channel_throttle.control_in<<std::endl;
	std::cout<<"g.channel_rudder.control_in="<<g.channel_rudder.control_in<<std::endl;
	std::cout<<"g.rc_5.control_in="<<g.rc_5.control_in<<std::endl;
	*/
}


void throttle_failsafe(uint16_t pwm)
{
#if 0
	if(g.throttle_fs_enabled == 0)
		return;

	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (pwm < g.throttle_fs_value){
		// we detect a failsafe from radio
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9){
			SendDebug_P("MSG FS ON ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 10) {
			ch3_failsafe = true;
			//set_failsafe(true);
			//failsafeCounter = 10;
		}else if (failsafeCounter > 10){
			failsafeCounter = 11;
		}

	}else if(failsafeCounter > 0){
		// we are no longer in failsafe condition
		// but we need to recover quickly
		failsafeCounter--;
		if (failsafeCounter > 3){
			failsafeCounter = 3;
		}
		if (failsafeCounter == 1){
			SendDebug_P("MSG FS OFF ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 0) {
			ch3_failsafe = false;
			//set_failsafe(false);
			//failsafeCounter = -1;
		}else if (failsafeCounter <0){
			failsafeCounter = -1;
		}
	}
#endif
}

void trim_control_surfaces()
{
#if 0
	read_radio();
	// Store control surface trim values
	// ---------------------------------
	if(mix_mode == 0){
		g.channel_roll.radio_trim = g.channel_roll.radio_in;
		g.channel_pitch.radio_trim = g.channel_pitch.radio_in;
		g.channel_rudder.radio_trim = g.channel_rudder.radio_in;
	}else{
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		//Recompute values here using new values for elevon1_trim and elevon2_trim
		//We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
		uint16_t center 			= 1500;
		g.channel_roll.radio_trim 	= center;
		g.channel_pitch.radio_trim 	= center;
	}

	// save to eeprom
	g.channel_roll.save_eeprom();
	g.channel_pitch.save_eeprom();
	g.channel_throttle.save_eeprom();
	g.channel_rudder.save_eeprom();
#endif
}

void trim_radio()
{
#if 0
	for (int y = 0; y < 30; y++) {
		read_radio();
	}

	// Store the trim values
	// ---------------------
	if(mix_mode == 0){
		g.channel_roll.radio_trim 		= g.channel_roll.radio_in;
		g.channel_pitch.radio_trim 		= g.channel_pitch.radio_in;
		//g.channel_throttle.radio_trim 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_trim 	= g.channel_rudder.radio_in;

	} else {
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		uint16_t center = 1500;
		g.channel_roll.radio_trim 	= center;
		g.channel_pitch.radio_trim 	= center;
		g.channel_rudder.radio_trim = g.channel_rudder.radio_in;
	}

	// save to eeprom
	g.channel_roll.save_eeprom();
	g.channel_pitch.save_eeprom();
	//g.channel_throttle.save_eeprom();
	g.channel_rudder.save_eeprom();
#endif
}


void Copter::trim_radio()
{
	uint16_t center = 1500;
	g.channel_roll.radio_trim 	= center;
	g.channel_pitch.radio_trim 	= center;
	g.channel_rudder.radio_trim = center;
}
