/*
 * utility.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "utility.h"

unsigned char Func_GetCommaPos(unsigned char *buf, unsigned char cx)
{
	unsigned char *p = buf;
	while (cx)
	{
		if (*buf == '*' || *buf<' ' || *buf>'z') return 0xff;
		if (*buf == ',') cx--;
		buf++;
	}
	return buf - p;
}

unsigned int Func_Pow(unsigned char m, unsigned char n)
{
	unsigned int result = 1;
	while (n--) result *= m;
	return result;
}

int Func_Str2num(unsigned char *buf, unsigned char*dx)
{
	unsigned char *p = buf;
	unsigned int ires = 0;
	unsigned int fres = 0;
	unsigned char ilen = 0, flen = 0;
	unsigned char i;
	unsigned char mask = 0;
	int res;
	while (1)
	{
		if (*p == 32){ p++;}
		if (*p == '-'){ mask |= 0x02; p++; }
		if (*p == ',' || (*p == '*'))break;
		if (*p == '.'){ mask |= 0x01; p++; }
		else if (*p>'9' || (*p<'0'))
		{
			ilen = 0;
			flen = 0;
			break;
		}
		if (mask & 0x01) flen++;
		else ilen++;
		p++;
	}

	if (mask & 0x02) buf++;
	for (i = 0; i<ilen; i++)
	{
		ires += Func_Pow(10, ilen - 1 - i)*(buf[i] - '0');
	}
	if (flen>5)flen = 5;
	*dx = flen;
	for (i = 0; i<flen; i++)
	{
		fres += Func_Pow(10, flen - 1 - i)*(buf[ilen + 1 + i] - '0');
	}
	res = ires*Func_Pow(10, flen) + fres;
	if (mask & 0x02) res = -res;
	return res;
}

/*
 * 精确到毫秒的延时
 */
int delay_ms(int ms)
{
	struct timeval delay;
	delay.tv_sec = 0;
	delay.tv_usec = ms * 1000;
	select(0, NULL, NULL, NULL, &delay);

	return 0;
}

int sleep_ms(int ms)
{
	usleep(1000*ms);

	return 0;
}

/*
 * 获取系统时间
 * 从UTC(coordinated universal time)时间
 * 1970年1月1日00时00分00秒(也称为Linux系统的Epoch时间)到当前时刻的秒数
 */
float gettimeofday_s()
{
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1+(float)(current_time.tv_usec)*1e-6;
}

float gettimeofday_ms()
{
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1e3+(float)(current_time.tv_usec)*1e-3;
}

float gettimeofday_us()
{
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1e6+(float)(current_time.tv_usec)*1;
}

struct timeval diff_gettimeofday(struct timeval start,struct timeval end)
{
	struct timeval temp;

	if ((end.tv_usec-start.tv_usec)<0)
	{
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_usec = 1e6+end.tv_usec-start.tv_usec;
	}
	else
	{
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_usec = end.tv_usec-start.tv_usec;
	}

	return temp;

}

float diff_gettimeofday_value(float start,float end)
{
	return end-start;
}

/*
 * 获取以系统启动瞬间为基准的时间
 * long sys_clock_gettime (clockid_t which_clock, struct timespec *tp);
 */
/*
 *  且在编译链接时需加上 -lrt ;因为在librt中实现了clock_gettime函数。
 *  struct timespec ts;
 *  clock_gettime(CLOCK_MONOTONIC,ts);
 */
float clock_gettime_s()
{
	struct timespec current_time;

	clock_gettime(CLOCK_MONOTONIC, &current_time);

	return (float)(current_time.tv_sec)*1+(float)(current_time.tv_nsec)*1e-9;

}

float clock_gettime_ms()
{
    float time_s=0.0;

    time_s=clock_gettime_s();

    return time_s*1e3;
}

struct timespec diff_clock(struct timespec start, struct timespec end)
{
	struct timespec temp;

	if ((end.tv_nsec-start.tv_nsec)<0)
	{
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1e9+end.tv_nsec-start.tv_nsec;
	}
	else
	{
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}

	return temp;
}

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
float get_bearing_to_north(float east,float north)
{
    float ret_degree=0.0;

    unsigned char bool_north = 0, bool_east = 0;
    unsigned char bool_great_than_45deg = 0;
    float change_temp = 0.0;

    if (east > 0)
        bool_east = 1;
    else
        bool_east = 0;

    if (north > 0)
        bool_north = 1;
    else
        bool_north = 0;

    east = fabsf(east);
    north = fabsf(north);

    if (east > north)
    {
        change_temp = east;
        east = north;
        north = change_temp;
        bool_great_than_45deg = 1;
    }
    else
    {
        bool_great_than_45deg = 0;
    }

    //if ((north != 0) && (!isnan(atan(east / north))))
    if ((north != 0))
        ret_degree = atan(east / north);     //0~PI/4
    else
        ret_degree = M_PI / 2;

    if (bool_great_than_45deg)
        ret_degree = M_PI / 2 - ret_degree;                 //PI/4~PI/2
    if (bool_north && bool_east)
        ret_degree = ret_degree+0;                      //0~PI/4
    if (bool_north && !bool_east)
        ret_degree = 0 - ret_degree;
    if (!bool_north && !bool_east)
        ret_degree = ret_degree - M_PI;
    if (!bool_north && bool_east)
        ret_degree = M_PI - ret_degree;

    return ret_degree;
}


