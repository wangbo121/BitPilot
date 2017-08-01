/*
 * defines.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define DEBUG 0

// Radio channels
// Note channels are from 0!
//
// XXX these should be CH_n defines from RC.h at some point.
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4


#endif /* DEFINES_H_ */
