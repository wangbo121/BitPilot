/*
 * Scheduler.cpp
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */

#include <BIT_HAL.h>



#include "Scheduler.h"

#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>

#include "RCOutput.h"
#include "RCInput.h"


extern const AP_HAL::HAL& hal;

extern bool _px4_thread_should_exit;

AP_HAL::Scheduler::Scheduler()
{}

void AP_HAL::Scheduler::init()
{

}

uint32_t AP_HAL::Scheduler::micros()
{
    //return (uint32_t)(hrt_absolute_time() - _sketch_start_time);
}

uint32_t AP_HAL::Scheduler::millis()
{
    //return hrt_absolute_time() / 1000;
}

void AP_HAL::Scheduler::delay_microseconds(uint16_t usec)
{

}

void AP_HAL::Scheduler::delay(uint16_t ms)
{

}

void AP_HAL::Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms)
{

}

void AP_HAL::Scheduler::register_timer_process(AP_HAL::TimedProc proc)
{

}

void AP_HAL::Scheduler::register_io_process(AP_HAL::TimedProc proc)
{

}

void AP_HAL::Scheduler::register_timer_failsafe(AP_HAL::TimedProc failsafe, uint32_t period_us)
{

}

void AP_HAL::Scheduler::suspend_timer_procs()
{

}

void AP_HAL::Scheduler::resume_timer_procs()
{
}

void AP_HAL::Scheduler::reboot()
{

}



bool AP_HAL::Scheduler::in_timerprocess() {

}


