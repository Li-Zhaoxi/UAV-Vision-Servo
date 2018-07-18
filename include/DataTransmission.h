#pragma once

//#include "definition.h"
//#include <Windows.h>
//#include <tchar.h>
//#include <iostream>
//#include <opencv2\opencv.hpp>

#include <uavdef.h>

//using std::string;




namespace DataTran
{
	// 开启数传模块
	UAV_EXPORTS bool OpenSerialPort(unsigned char *_RS, double *_height);
	UAV_EXPORTS void SendData2UAV(unsigned char SS, float pX, float pY, float vX, float vY, float useTime);

	//UAV数据信息
	UAV_EXPORTS extern float pixelError;//设置像素误差
	UAV_EXPORTS extern float highPara_1;
	UAV_EXPORTS extern float highPara_2;
	UAV_EXPORTS extern float carHighValue; //小车的高度
	UAV_EXPORTS extern float cross_Length;

	UAV_EXPORTS extern int confirmNum;
	UAV_EXPORTS extern int confirmedNum;


//inValue为输入的高度值，Para_i为计算对应的参数值，calType为需要计算的状态
//举例：若calType为高度计算像素宽度状态，则实际状态宽度*当前函数返回值即可；若为高度估计距离状态，则实际像素距离*当前函数返回值
	UAV_EXPORTS float calPixcelAndDist(float inValue, float Para_1, float Para_2, unsigned char calType);
	UAV_EXPORTS float calPixcelAndDist(float inValue, unsigned char calType);

}