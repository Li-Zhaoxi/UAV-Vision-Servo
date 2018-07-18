#include "UAV_GUI.h"
#include <simulation.h>

namespace UAV_GUI
{
	HINSTANCE hInstance;
	void initHINSTANCE()
	{
		hInstance = GetModuleHandle(NULL);
		WNDCLASS UAV_MAIN;
		UAV_MAIN.cbClsExtra = 0;
		UAV_MAIN.cbWndExtra = 0;
		UAV_MAIN.hCursor = LoadCursor(hInstance, IDC_ARROW);;
		UAV_MAIN.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ICON_UAV_1));
		UAV_MAIN.lpszMenuName = MAKEINTRESOURCE(IDR_MENU_UAV_MAIN);
		UAV_MAIN.style = CS_HREDRAW | CS_VREDRAW;
		UAV_MAIN.hbrBackground = (HBRUSH)COLOR_WINDOW;
		UAV_MAIN.lpfnWndProc = WindowProc;
		UAV_MAIN.lpszClassName = TEXT("UAV_MAIN");
		UAV_MAIN.hInstance = hInstance;
		RegisterClass(&UAV_MAIN);
	}
	LRESULT CALLBACK WindowProc(_In_  HWND hwnd, _In_  UINT uMsg, _In_  WPARAM wParam, _In_  LPARAM lParam)
	{
		int wmID, wmEvent;
		switch (uMsg)
		{
		case WM_COMMAND:
		{
			switch (LOWORD(wParam))
			{
			case ID_UAV_OPEN:
				SIMULATION::UAV_SIM_OPEN();
				break;
			case ID_UAV_CLOSE:
				SIMULATION::UAV_SIM_CLOSE();
				break;
			default:
				break;
			}
			break;
		}
		case WM_DESTROY:
		{
			PostQuitMessage(0);
			return 0;
		}
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}


	HWND hwnd_console = NULL;

	HWND hwnd_main = NULL;
	HWND hwnd_view_1;
	std::string tilde_view_1 = "Original images.";
	HWND hwnd_view_2;
	std::string tilde_view_2 = "Final images.";
	HWND hwnd_view_3;
	std::string tilde_view_3 = "Temp images.";
	cv::Mat Img_back;


	HWND hwnd_method_height = NULL;
	HWND hwnd_resolution_height = NULL;


	void CreateMainGUI()
	{
		hwnd_main = CreateWindow(TEXT("UAV_MAIN"), L"UAV 伺服器", WS_OVERLAPPEDWINDOW&~WS_SYSMENU, 38, 20, 1280, 640, NULL, NULL, hInstance, NULL);

		hwnd_view_1 = CreateWindow(L"static", TEXT("Original images."), WS_CHILD | WS_VISIBLE | WS_BORDER, 10, 10, 480, 270, hwnd_main, (HMENU)IDC_VIEW_1, hInstance, NULL);
		hwnd_view_2 = CreateWindow(L"static", TEXT("Original images."), WS_CHILD | WS_VISIBLE | WS_BORDER, 500, 10, 480, 270, hwnd_main, (HMENU)IDC_VIEW_2, hInstance, NULL);
		hwnd_view_3 = CreateWindow(L"static", TEXT("Original images."), WS_CHILD | WS_VISIBLE | WS_BORDER, 10, 290, 480, 270, hwnd_main, (HMENU)IDC_VIEW_3, hInstance, NULL);
		// 设置窗口图片显示
		Img_back = cv::imread("Back.jpg");
		HWND hwnd_temp, hParent;

		cv::namedWindow(tilde_view_1, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
		cv::resizeWindow(tilde_view_1, 480, 270);
		hwnd_temp = (HWND)cvGetWindowHandle(tilde_view_1.c_str());
		hParent = ::GetParent(hwnd_temp);
		::SetParent(hwnd_temp, hwnd_view_1);
		::ShowWindow(hParent, SW_HIDE);
		cv::imshow(tilde_view_1, Img_back);

		cv::namedWindow(tilde_view_2, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
		cv::resizeWindow(tilde_view_2, 480, 270);
		hwnd_temp = (HWND)cvGetWindowHandle(tilde_view_2.c_str());
		hParent = ::GetParent(hwnd_temp);
		::SetParent(hwnd_temp, hwnd_view_2);
		::ShowWindow(hParent, SW_HIDE);
		cv::imshow(tilde_view_2, Img_back);
		
		cv::namedWindow(tilde_view_3, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
		cv::resizeWindow(tilde_view_3, 480, 270);
		hwnd_temp = (HWND)cvGetWindowHandle(tilde_view_3.c_str());
		hParent = ::GetParent(hwnd_temp);
		::SetParent(hwnd_temp, hwnd_view_3);
		::ShowWindow(hParent, SW_HIDE);
		cv::imshow(tilde_view_3, Img_back);

		hParent = ::GetParent(hwnd_console);
		::ShowWindow(hParent, SW_HIDE);
		::SetParent(hwnd_console, hwnd_main);
		::MoveWindow(hwnd_console, 1060, 10, 200, 540, true);
		UpdateWindow(hwnd_console);
		//std::cout << "dsff";

		// 创建数据区
		CreateWindow(L"static", TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER, 500, 290, 480, 270, hwnd_main, (HMENU)IDC_GROUP, hInstance, NULL);

		CreateWindow(L"static", TEXT("Methods Height:"), WS_CHILD | WS_VISIBLE, 510, 300, 150, 20, hwnd_main, (HMENU)IDC_STATIC_METHOD_NOTE, hInstance, NULL);
		hwnd_method_height = CreateWindow(L"static", TEXT("00.00m"), WS_CHILD | WS_VISIBLE, 660, 300, 80, 20, hwnd_main, (HMENU)IDC_STATIC_METHOD, hInstance, NULL);

		CreateWindow(L"static", TEXT("Resolution Height:"), WS_CHILD | WS_VISIBLE, 510, 330, 150, 20, hwnd_main, (HMENU)IDC_STATIC_RESOLUTION_NOTE, hInstance, NULL);
		hwnd_resolution_height = CreateWindow(L"static", TEXT("00.00m"), WS_CHILD | WS_VISIBLE, 660, 330, 80, 20, hwnd_main, (HMENU)IDC_STATIC_RESOLUTION, hInstance, NULL);



		ShowWindow(hwnd_main, SW_SHOW);
		UpdateWindow(hwnd_main);

	}
}
