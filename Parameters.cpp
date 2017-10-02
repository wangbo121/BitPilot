/*
 * Parameters.cpp
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#include "copter.h"
#include "Parameters.h"

T_PARAM param_all[]={//def     key         name                             *next param
										{113.0,        0,          "SYSID_SW_MREV",         &param_all[1]},
										{10.0,          1,          "SYSID_SW_TYPE",            NULL}


										};

