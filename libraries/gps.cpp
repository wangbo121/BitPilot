/*
 * gps.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */


// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "gps.h"

void
GPS::update(void)
{
	bool	result;

	// call the GPS driver to process incoming data
	result = read();


}

void
GPS::setHIL(long _time, float _latitude, float _longitude, float _altitude,
            float _ground_speed, float _ground_course, float _speed_3d, uint8_t _num_sats)
{
}

// XXX this is probably the wrong way to do it, too
void
GPS::_error(const char *msg)
{

}

