/*
 * rc.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef RC_H_
#define RC_H_

#include <inttypes.h>

/*
 * 这个文件主要是从硬件radio获取pwm值，但是呢
 * 我们获取的有可能不是直接的pwm值，而是其他的比如串口数据
 * 所以还需要上层的把从rc.cpp中获取的转换为控制角度等
 * 这个就是rc_channel.cpp文件的功能了
 *
 */

class AP_RC
{
  private:
  public:
	AP_RC();
	void 		init();
	void 		output_ch_pwm(uint8_t ch, uint16_t pwm);
	uint16_t 	input_ch(uint8_t ch);
};




#endif /* RC_H_ */
