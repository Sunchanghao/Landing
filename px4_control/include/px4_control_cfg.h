/*
 * px4_control_cfg.h
 *
 *  Created on: 2018-5-4
 *      Author: zipout
 *///Email		  : 1554459957@qq.com

#ifndef PX4_CONTROL_CFG_H_
#define PX4_CONTROL_CFG_H_
#include <ros/ros.h>
#include <iostream>
using namespace std;
 typedef struct
	{
		float vel_x;
		float vel_y;
		float vel_z;
	} S_SETPOINT_VEL;
	typedef struct
	{
		float difference;      //比例项
		float differential;    //微分项
		float tempDiffer;			 //上一时刻的比例项
		float intergral;       //积分项
	} S_PID_ITEM;
	typedef struct
	{
		float p;       //比例项系数
		float d;       //微分项系数.
		float i;       //积分项系数
	} S_PID;
#endif /* PX4_CONTROL_CFG_H_ */
