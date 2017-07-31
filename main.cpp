/*
 * main.cpp
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#include <iostream>

using namespace std;

#include "pid.h"
#include "vector2.h"



int main()
{

	cout<<"Welcome to BitPilot"<<endl;

	BIT_PID pid;

	pid.set_kP(2);
	float result = pid.get_pid(2,1000,1);

	cout<<"result="<<result<<endl;


	Vector2i vector2_1(1,2);
	Vector2i vector2_2(3,4);
	Vector2i vector_result;

	vector_result=vector2_1+vector2_2;

	cout<<vector_result.x<<endl;
	cout<<vector_result.y<<endl;



	return 0;
}



