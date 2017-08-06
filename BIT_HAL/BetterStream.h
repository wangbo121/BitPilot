/*
 * BetterStream.h
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */

#ifndef BETTERSTREAM_H_
#define BETTERSTREAM_H_

#include <stdarg.h>
#include "BIT_HAL_Namespace.h"
#include "Stream.h"

/* prog_char_t: */
//#include <AP_Progmem.h>

/* AP_HAL::BetterStream is a pure virtual interface. It resembles
 * Michael Smith's BetterStream library for Arduino.
 * The Michael Smith BetterStream provided some implementations for AVR based
 * on _vprintf().
 * Please provide your own platform-specic implementation of vprintf, sprintf,
 * etc. to implement the printf functions.
 *
 * TODO: Segregate prog_char_t dependent functions to be available on AVR
 * platform only, with default implementations elsewhere.
 */
#include <stdarg.h>
#include "BIT_HAL_Namespace.h"

class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    virtual void print_P(const char *) = 0;
    virtual void println_P(const char *) = 0;
    virtual void printf(const char *, ...)
                        __attribute__ ((format(__printf__, 2, 3))) = 0;
    /* No format checking on printf_P: can't currently support that on AVR */
    virtual void _printf_P(const char *, ...) = 0;

#define printf_P(fmt, ...) _printf_P((const prog_char *)fmt, ## __VA_ARGS__)

    virtual void vprintf(const char *, va_list) = 0;
    virtual void vprintf_P(const char *, va_list) = 0;
};



#endif /* BETTERSTREAM_H_ */
