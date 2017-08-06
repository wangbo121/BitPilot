/*
 * Scheduler.h
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "BIT_HAL_Namespace.h"

#include <stdint.h>


class AP_HAL::Scheduler {
public:
    Scheduler() ;
     void     init() ;
     void     delay(uint16_t ms) ;
     uint32_t millis() ;
     uint32_t micros() ;
     void     delay_microseconds(uint16_t us) ;
     void     register_delay_callback(AP_HAL::Proc,
                        uint16_t min_time_ms) ;

    // register a high priority timer task
     void     register_timer_process(AP_HAL::TimedProc) ;

    // register a low priority IO task
     void     register_io_process(AP_HAL::TimedProc) ;

    // suspend and resume both timer and IO processes
     void     suspend_timer_procs() ;
     void     resume_timer_procs() ;

     bool     in_timerprocess() ;

     void     register_timer_failsafe(AP_HAL::TimedProc,
                        uint32_t period_us) ;

     bool     system_initializing() ;
     void     system_initialized() ;

     void     panic(const char *errormsg) ;
     void     reboot() ;
};



#endif /* SCHEDULER_H_ */
