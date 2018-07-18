#include "simulation.h"
#include "resource.h"
#include <iostream>
#include <definition.h>
#include <time.h>
#include <tchar.h>

HINSTANCE hinst = NULL;
BOOL APIENTRY DllMain(HANDLE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		hinst = (HINSTANCE)hModule;
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

namespace SIMULATION
{
	unsigned int receive_type;
	double receive_time;
	double receive_data[6];

	unsigned int *send_type = NULL;
	double *uav_height = NULL;

	HWND hwnd_simulation_uav = NULL;

	LOGFONT LogFont;
	HFONT hFont;

	INT_PTR CALLBACK Simulation_UAV_CALLBACK(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam);
	void createSimulation_UAV2PC(HWND hWndParent)
	{
		// 字体设置
		::memset(&LogFont, 0, sizeof(LOGFONT));
		lstrcpy(LogFont.lfFaceName, L"Calibri");
		LogFont.lfWeight = FW_BOLD;
		LogFont.lfHeight = -14; // 字体大小
		LogFont.lfCharSet = 134;
		LogFont.lfOutPrecision = 3;
		LogFont.lfClipPrecision = 2;
		LogFont.lfOrientation = 45;
		LogFont.lfQuality = 1;
		LogFont.lfPitchAndFamily = 2;
		hFont = CreateFontIndirect(&LogFont);// 创建字体

		LOGFONT logfont_temp; HFONT hfont_temp;
		memcpy(&logfont_temp, &LogFont, sizeof(LOGFONT));
		// 创建新的对话框
		HWND hwnd_temp;
		hwnd_simulation_uav = CreateDialog(hinst, MAKEINTRESOURCE(IDD_DIALOG_SIMU_UAV), hWndParent, (DLGPROC)Simulation_UAV_CALLBACK);
		MoveWindow(hwnd_simulation_uav, 10, 10, 270, 540, TRUE);
		
		::memset(&LogFont, 0, sizeof(LOGFONT));
// Group1
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_GROUP1);
		MoveWindow(hwnd_temp, 10, 10, 240, 150, TRUE);
		SendMessage(hwnd_temp, WM_SETFONT, (WPARAM)hFont, 0);
		// Combo 
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_COMBO_UAV_TYPE);
		MoveWindow(hwnd_temp, 20, 30, 220, 150, TRUE);
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_WAITING"));
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_OBJECT_DETECTION"));
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_CROSSDETECTION"));
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_QUITWHILE"));
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_RESET"));
		SendMessage(hwnd_temp, CB_ADDSTRING, 0, (LPARAM)TEXT("UAV_TEST_FLELD"));
		SendMessage(hwnd_temp, CB_SELECTSTRING, 0, (LPARAM)TEXT("UAV_WAITING"));

		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_UAV_HEIGHT);
		MoveWindow(hwnd_temp, 20, 65, 60, 20, TRUE);

		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_EDIT_UAV_HEIGHT);
		MoveWindow(hwnd_temp, 90, 60, 100, 25, TRUE);

		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_BUTTON_UAV_SEND);
		MoveWindow(hwnd_temp, 60, 130, 110, 20, TRUE);
		logfont_temp.lfHeight = -10;
		hfont_temp = CreateFontIndirect(&logfont_temp);
		SendMessage(hwnd_temp, WM_SETFONT, (WPARAM)hfont_temp, 0);

//Group2
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_GROUP2);
		MoveWindow(hwnd_temp, 10, 170, 240, 170, TRUE);
		SendMessage(hwnd_temp, WM_SETFONT, (WPARAM)hFont, 0);

		// Now time.
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_NOTE_TIME);
		MoveWindow(hwnd_temp, 20, 200, 80, 30, TRUE);
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_TIME);
		MoveWindow(hwnd_temp, 110, 200, 80, 30, TRUE);

		// PC type.
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_TYPE_NOTE);
		MoveWindow(hwnd_temp, 20, 240, 80, 20, TRUE);
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_PCTYPE);
		MoveWindow(hwnd_temp, 110, 240, 80, 20, TRUE);

		// PC data;
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_DATA_NOTE);
		MoveWindow(hwnd_temp, 20, 270, 80, 30, TRUE);
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_DATA);
		MoveWindow(hwnd_temp, 110, 270, 80, 30, TRUE);

		// Pipeline time;
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_PIPELINE_NOTE);
		MoveWindow(hwnd_temp, 20, 310, 80, 20, TRUE);
		hwnd_temp = GetDlgItem(hwnd_simulation_uav, IDC_STATIC_PIPELINE);
		MoveWindow(hwnd_temp, 110, 310, 80, 20, TRUE);

		ShowWindow(hwnd_simulation_uav, SW_SHOW);
		UpdateWindow(hwnd_simulation_uav);
	}
	void UAV_SIM_OPEN()
	{
		ShowWindow(hwnd_simulation_uav, SW_SHOW);
		UpdateWindow(hwnd_simulation_uav);
	}
	void UAV_SIM_CLOSE()
	{
		ShowWindow(hwnd_simulation_uav, SW_HIDE);
		UpdateWindow(hwnd_simulation_uav);
	}

	void SetMessageSend(unsigned int *_image_type, double *_uav_height)
	{
		send_type = _image_type;
		uav_height = _uav_height;
	}


	unsigned char *isSend = 0;
	unsigned int *pc_type;
	double *pc_data;
	double *pipeline_time;
	void GetMessageReceive(unsigned char *_isSend, unsigned int *_pc_type, double _data[4], double *_pipeline_time)
	{
		isSend = _isSend;
		pc_type = _pc_type;
		pc_data = _data;
		pipeline_time = _pipeline_time;
	}

	HANDLE handle_uav_simulator;
	DWORD WINAPI SimulatorCALLBACK(LPVOID lpParam);
	UAV_EXPORTS void StartUAVSimulator()
	{
		handle_uav_simulator = CreateThread(NULL, 0, SimulatorCALLBACK, NULL, 0, NULL);
	}
	


}


INT_PTR CALLBACK SIMULATION::Simulation_UAV_CALLBACK(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
	int wmId;
	switch (msg)
	{
	case WM_INITDIALOG:
		return TRUE;
	case WM_DESTROY:
	{
		PostQuitMessage(0);
		break;
	}
	case WM_COMMAND:
	{
		wmId = LOWORD(wParam);
		switch (wmId)
		{
		case IDC_BUTTON_UAV_SEND:
		{
			WCHAR buf[256];
			GetWindowText(GetDlgItem(hDlg, IDC_EDIT_UAV_HEIGHT), buf, 256);
			double hight_value = _wtof(buf);
			int state;
			int idx = (int)SendMessage(GetDlgItem(hDlg, IDC_COMBO_UAV_TYPE), CB_GETCURSEL, 0, 0);
			switch (idx)
			{
			case 0:
				state = UAV_WAITING;
				break;
			case 1:
				state = UAV_RESET;
				break;
			case 2:
				state = UAV_C1_DETECTION;
				break;
			case 3:
				state = UAV_C2_DETECTION;
				break;
			case 4:
				state = UAV_RESET;
				break;
			case 5:
				state = UAV_WAITING;
				break;
			default:
				break;
			}
			
			if (send_type != NULL || uav_height != NULL)
			{
				*send_type = state;
				*uav_height = hight_value;
				//printf("type: %d, height: %.2f", state, hight_value);
			}
			

			break;
		}
		default:
			break;
		}
		break;
	}
	default:
		break;
	}
	return (INT_PTR)FALSE;
}


DWORD WINAPI SIMULATION::SimulatorCALLBACK(LPVOID lpParam)
{
	

	while (1)
	{
		if (*isSend)
		{
			tm t;
			time_t now;
			TCHAR tmp[256];

			time(&now);
			localtime_s(&t, &now);   //获取当地日期和时间

			// Now time.
			_stprintf_s(tmp, 256, TEXT("%d : %d : %d\n %d/%d/%d\n"), t.tm_hour, t.tm_min, t.tm_sec, t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
			SetWindowText(GetDlgItem(hwnd_simulation_uav, IDC_STATIC_TIME), tmp);
			
			// PC type
			_stprintf_s(tmp, 256, TEXT("%d"), *pc_type);
			SetWindowText(GetDlgItem(hwnd_simulation_uav, IDC_STATIC_PCTYPE), tmp);

			// PC data;
			_stprintf_s(tmp, 256, TEXT("%.2f, %.2f, %.2f, %.2f"), pc_data[0], pc_data[1], pc_data[2], pc_data[3]);
			SetWindowText(GetDlgItem(hwnd_simulation_uav, IDC_STATIC_DATA), tmp);

			// Pipeline Time.
			_stprintf_s(tmp, 256, TEXT("%.2fms"), *pipeline_time);
			SetWindowText(GetDlgItem(hwnd_simulation_uav, IDC_STATIC_PIPELINE), tmp);

			*isSend = 0;
		}
		
		Sleep(15);
	}
}