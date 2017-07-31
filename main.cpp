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
#include "vector3.h"



int main()
{

	cout<<"Welcome to BitPilot"<<endl;

	BIT_PID pid;
	pid.set_kP(2);
	float pid_result = pid.get_pid(2,1000,1);
	cout<<"pid_result="<<pid_result<<endl;


	Vector2i vector2_1(1,2);
	Vector2i vector2_2(3,4);
	Vector2i vector2_result;

	vector2_result=vector2_1+vector2_2;

	cout<<"vector2_result.x="<<vector2_result.x<<endl;
	cout<<"vector2_result.y="<<vector2_result.y<<endl;

	Vector3i vector3_1(1,2,3);
	Vector3i vector3_2(3,4,5);
	Vector3i vector3_result;

	vector3_result=vector3_1+vector3_2;

	cout<<"vector3_result.x="<<vector3_result.x<<endl;
	cout<<"vector3_result.y="<<vector3_result.y<<endl;
	cout<<"vector3_result.z="<<vector3_result.z<<endl;





	return 0;
}



