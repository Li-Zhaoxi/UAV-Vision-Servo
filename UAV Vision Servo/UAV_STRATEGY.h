#pragma once

#include <Windows.h>

#include "VIDEO_STREAM.h"

#include <RVIBE.h>
#include <Simplified_EDLines.h>
#include <EllipseDetectorYaed.h>
#include <C2_FTD.h>
#include <C1_FTD.h>


namespace UAV_STRATEGY
{
	// 无人机机身信息
	extern unsigned int image_type;
	extern double uav_height;
	//------------------------

	// PC发送消息至UAV
	extern unsigned char isSend;
	extern unsigned int pc_type;
	extern double data[4];
	extern double pilpline_time;
	//-----------------------------

	// UAV Servo State
	extern unsigned char RS;
	extern unsigned char IS;
	extern unsigned char SS;
	//-----------------------------




	// 算法初始化定义
	extern RVIBE rvibe;
	extern Simplified_EDLines sedlines;
	extern CEllipseDetectorYaed yaed[4];
	extern C2_FTD c2_ftd;
	extern C1_FTD c1_ftd;

	extern bool running_servo;

	extern HANDLE handle_strategy;
	// 创建策略，算法初始化，创建策略线程
	void CreateStrategy();
	// 策略回调函数
	DWORD WINAPI STRATEGY_CALLBACK(LPVOID lpParam);

	void DestoryStrategy();


	void UAV_SOLUTION_1();


	void SetChangeHeights(HWND hwnd_methods_height, double &methods_height, HWND hwnd_resolution_height, double &resolution_height);

}