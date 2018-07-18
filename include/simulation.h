#pragma once
#include <uavdef.h>

#include <Windows.h>

namespace SIMULATION
{
	//// 无人机接收到的信息
	//UAV_EXPORTS extern unsigned int receive_type;
	//UAV_EXPORTS extern double receive_time;
	//UAV_EXPORTS extern double receive_data[6];

	//// UAV -> PC104 状态，高度，可直接修改
	//UAV_EXPORTS extern unsigned int *send_type;
	//UAV_EXPORTS extern double *uav_height;

	extern HWND hwnd_simulation_uav;
	UAV_EXPORTS void createSimulation_UAV2PC(HWND hWndParent);
	
	UAV_EXPORTS void SetMessageSend(unsigned int *_image_type, double *_uav_height);
	
	UAV_EXPORTS void UAV_SIM_OPEN();
	UAV_EXPORTS void UAV_SIM_CLOSE();
	
	UAV_EXPORTS void GetMessageReceive(unsigned char *_isSend, unsigned int *_pc_type, double _data[4], double *_pipeline_time);
	UAV_EXPORTS void StartUAVSimulator();

}