/*
 * AP_HAL_Namespace.h
 *
 *  Created on: 2017-8-4
 *      Author: wangbo
 */

#pragma once

#ifndef AP_HAL_NAMESPACE_H_
#define AP_HAL_NAMESPACE_H_

#include <inttypes.h>

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;


    class AnalogSource;
    class AnalogIn;

    //class DigitalSource;
   //class GPIO;

    //class ConsoleDriver;

    class Storage;

    class RCInput;
    class RCOutput;

    class Scheduler;

    /* Typdefs for function pointers (Procedure, Timed Procedure) */
	typedef void(*Proc)(void);
	typedef void(*TimedProc)(uint32_t);

    // Must be implemented by the concrete HALs.
    const HAL& get_HAL();
}




#endif /* AP_HAL_NAMESPACE_H_ */
