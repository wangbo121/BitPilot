/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include <iostream>

using namespace std;

#include "pid.h"



int main()
{

	cout<<"Welcome to BitPilot"<<endl;

	BIT_PID pid;

	pid.set_kP(2);
	float result = pid.get_pid(2,1000,1);

	cout<<"result="<<result<<endl;


	return 0;
}



