#include <iostream>

#include "UAV_GUI.h"
#include "UAV_CONFIG.h"
#include "VIDEO_STREAM.h"
#include "UAV_STRATEGY.h"

#include <simulation.h>
#include <DataTransmission.h>
#include <logfiles_io.h>

INT_PTR Simulation_UAV_CALLBACK2(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_DESTROY:
	{
		PostQuitMessage(0);
		break;
	}
	default:
		break;
	}
	return (INT_PTR)FALSE;
}

int main()
{


	//UAV_CONFIG::SaveConfig("Config\\UAV_CONFIG.xml");
	//return 0;
	UAV_CONFIG::LoadConfig("Config\\UAV_CONFIG.xml");
	SetConsoleTitle(TEXT("降落目标检测框架 - v2.0"));
	UAV_GUI::hwnd_console = ::GetConsoleWindow();


	//  初始化程序实例
	UAV_GUI::initHINSTANCE();
	UAV_GUI::CreateMainGUI();
	
	// 创建仿真模块
	SIMULATION::createSimulation_UAV2PC(UAV_GUI::hwnd_main); // 创建仿真界面
	SIMULATION::SetMessageSend(&UAV_STRATEGY::image_type, &UAV_STRATEGY::uav_height); // 链接数据，UAV可直接修改机器数据
	SIMULATION::GetMessageReceive(&UAV_STRATEGY::isSend, &UAV_STRATEGY::pc_type, UAV_STRATEGY::data, &UAV_STRATEGY::pilpline_time);
	SIMULATION::StartUAVSimulator();


	// 串口模块
	//bool isOpen = DataTran::OpenSerialPort(&UAV_STRATEGY::RS, &UAV_STRATEGY::uav_height);
	//if (!isOpen)
	//	printf("Warning: Can't open serial port.\n");


	// 视频流初始化实例，在View1视图上显示对应的图像
	VIDEO_STREAM::initVideoStream(); //初始化后，将会获得当前情况下的灰度数据信息

	// 视频录制与数据存储
	if(!UAV_IO::InitLogFiles(true, true, true, true, true))
		printf("Warning: Can't create log files.\n");


	// 策略使用
	UAV_STRATEGY::CreateStrategy();



	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);

	}
	return 0;
}



