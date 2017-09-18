/*
 * rc.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

/*
 * 使用硬件抽象层必须包括的头文件和hal的extern声明
 */
#include "BIT_HAL.h"
extern const AP_HAL::HAL& hal;



#include "rc.h"

#include "all_external_device.h"

// Variable definition for interrupt
volatile uint16_t timer1count   = 0;
volatile uint16_t timer2count   = 0;
volatile uint16_t timer3count   = 0;
volatile uint16_t timer4count   = 0;

volatile int16_t timer1diff     = 1500 * 2;
volatile int16_t timer2diff     = 1500 * 2;
volatile int16_t timer3diff     = 1100 * 2;
volatile int16_t timer4diff     = 1500 * 2;

//volatile uint16_t raw[8];
#define CH1_READ 1
#define CH2_READ 2
#define CH3_READ 4
#define CH4_READ 8

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3

volatile int8_t		_rc_ch_read;
volatile uint16_t	_timer_out;
volatile uint16_t	_timer_ovf_a;
volatile uint16_t	_timer_ovf_b;
volatile uint16_t	_timer_ovf;


AP_RC::AP_RC()
{

}

void
AP_RC::init()
{



	/*
	 * 在我看来初始化，就应该是下面的这些set_ch_pwm呀，
	 * 但是他们为什么用的上面了，是为了封装吗？
	 */
	/*
	set_ch_pwm(0, 1500);
	set_ch_pwm(1, 1500);
	set_ch_pwm(2, 1500);
	set_ch_pwm(3, 1500);
	*/

}

uint16_t
AP_RC::input_ch(uint8_t ch)
{
	/*
	 * 这里是从hal获取通道的pwm值
	 */
	switch(ch)
	{
	case CH_1:
		//return 1500;
		return (uint16_t)all_external_device_input.rc_raw_in_0;
		return 1500;
		return hal.rcin->read(CH_1);
		break;
	case CH_2:
		return (uint16_t)all_external_device_input.rc_raw_in_1;
		return 1500;//1200;
		return hal.rcin->read(CH_2);
		break;
	case CH_3:
		return (uint16_t)all_external_device_input.rc_raw_in_2;
		return 1500;//1300;
		return hal.rcin->read(CH_3);
		break;
	case CH_4:
		//return 1800;//1700;
		return (uint16_t)all_external_device_input.rc_raw_in_3;
		return 1500;
		return hal.rcin->read(CH_4);
		break;
	case CH_5:
		//return 1400;//特技，完全手控模式
		return (uint16_t)all_external_device_input.rc_raw_in_4;
		return 1700;//增稳
		return hal.rcin->read(CH_5);
		break;
	case CH_6:
		return hal.rcin->read(CH_6);
		break;
	case CH_7:
		return hal.rcin->read(CH_7);
		break;
	case CH_8:
		return hal.rcin->read(CH_8);
		break;

	default:
		break;
	}
}

void
AP_RC::output_ch_pwm(uint8_t ch, uint16_t pwm)
{

	switch(ch)
	{
	case CH_1:
		break;
	case CH_2:
		break;
	case CH_3:
		break;
	case CH_4:
		break;
	}
}


//Interrupt Service Routines（中断服务程序） wangbo20170801
/*
 * 下面的都是串口中断服务，也就是通过串口读取遥控器通道的值，我们的该怎么写呢？
 */
// radio PWM input timers
#if 0
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	if(PIND & B00000100){	   // ch 1 (pin 2) is high
		if ((_rc_ch_read & CH1_READ) != CH1_READ){
			_rc_ch_read |= CH1_READ;
			timer1count = cnt;
		}
	}else if ((_rc_ch_read & CH1_READ) == CH1_READ){	// ch 1 (pin 2) is Low, and we were reading
		_rc_ch_read &= B11111110;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
			 timer1diff = (cnt + 40000 - timer1count) >> 1;	 // Timer1 TOP = 40000
		else
			timer1diff = (cnt - timer1count) >> 1;
	}

	if(PIND & B00001000){	   // ch 2 (pin 3) is high
		if ((_rc_ch_read & CH2_READ) != CH2_READ){
			_rc_ch_read |= CH2_READ;
			timer2count = cnt;
		}
	}else if ((_rc_ch_read & CH2_READ) == CH2_READ){	// ch 1 (pin 2) is Low
		_rc_ch_read &= B11111101;
		if (cnt < timer2count)   // Timer1 reset during the read of this pulse
			 timer2diff = (cnt + 40000 - timer2count) >> 1;	 // Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer2count) >> 1;
	}
}
#endif

