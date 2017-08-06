/*
 * Print.h
 *
 *  Created on: 2017-8-6
 *      Author: wangbo
 */

#ifndef PRINT_H_
#define PRINT_H_

#include "BIT_HAL_Namespace.h"

#include <inttypes.h>
#include <string.h>

/**
 * This is the Arduino (v1.0) Print class, with some changes:
 * - Removed methods for class String or _FlashStringHelper
 * - printFloat takes a float, not a double. (double === float on AVR, but
 *   not on other platforms)
 */

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class AP_HAL::Print {
  private:
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(float, uint8_t);
  public:
    Print() {}

    virtual size_t write(uint8_t) = 0;

    size_t write(const char *str) { return write((const uint8_t *)str, strlen(str)); }
    virtual size_t write(const uint8_t *buffer, size_t size);

    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(float , int = 2);
    size_t print(double , int = 2);

    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(float , int = 2);
    size_t println(double , int = 2);
    size_t println(void);

};

#endif /* PRINT_H_ */
