/*
 * utility.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef UTILITY_H_
#define UTILITY_H_

unsigned char Func_GetCommaPos(unsigned char *buf, unsigned char cx);
unsigned int Func_Pow(unsigned char m, unsigned char n);
int Func_Str2num(unsigned char *buf, unsigned char*dx);

int delay_ms(int ms);
int sleep_ms(int ms);

float gettimeofday_s();
float gettimeofday_ms();
float gettimeofday_us();
struct timeval diff_gettimeofday(struct timeval start,struct timeval end);
float diff_gettimeofday_value(float start,float end);

/*
 * 获取以系统启动瞬间为基准的时间
 * long sys_clock_gettime (clockid_t which_clock, struct timespec *tp);
 * 且在编译链接时需加上 -lrt ;因为在librt中实现了clock_gettime函数。
 * struct timespec ts;
 * clock_gettime(CLOCK_MONOTONIC,ts);
 */
float clock_gettime_s();
float clock_gettime_ms();
struct timespec diff_clock(struct timespec start, struct timespec end);

float get_bearing_to_north(float east,float north);



#endif /* UTILITY_H_ */
