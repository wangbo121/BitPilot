/*
 * compass_hmc5843.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */



#include <math.h>

#include "compass_hmc5843.h"

#define COMPASS_ADDRESS       0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// Public Methods //////////////////////////////////////////////////////////////
bool BIT_Compass_HMC5843::init()
{
  int numAttempts = 0;
  int success = 0;

  //delay(10);

  // calibration initialisation
  calibration[0] = 1.0;
  calibration[1] = 1.0;
  calibration[2] = 1.0;


  return(success);
}

// Read Sensor data
void BIT_Compass_HMC5843::read()
{
  int i = 0;
  unsigned char  buff[6];
  Vector3f rot_mag;

  /*
   * 这里是通过串口或者其他方式从缓存中读取数据到buff
   */

  if (i==6)  // All bytes received?
  {
    // MSB byte first, then LSB, X,Y,Z
    mag_x = -((((int)buff[0]) << 8) | buff[1]) * calibration[0];    // X axis
    mag_y = ((((int)buff[2]) << 8) | buff[3]) * calibration[1];    // Y axis
    mag_z = -((((int)buff[4]) << 8) | buff[5]) * calibration[2];    // Z axis

	rot_mag =  Vector3f(mag_x,mag_y,mag_z);
	rot_mag = rot_mag + _offset;
	mag_x = rot_mag.x;
	mag_y = rot_mag.y;
	mag_z = rot_mag.z;
  }
}
