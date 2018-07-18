#pragma once
#include <Windows.h>
#include "resource.h"
#include <opencv2\opencv.hpp>
#include <string>

namespace UAV_GUI
{
	extern HINSTANCE hInstance;
	void initHINSTANCE();
	LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	extern HWND hwnd_console;

	extern HWND hwnd_main;
	extern HWND hwnd_view_1; // 原始图片视角
	extern std::string tilde_view_1; // 视角1标题
	extern HWND hwnd_view_2; // 原始图片视角
	extern std::string tilde_view_2; // 视角2标题
	extern HWND hwnd_view_3; // 原始图片视角
	extern std::string tilde_view_3; // 视角2标题
	extern cv::Mat Img_back; // 背景图片
	void CreateMainGUI();



	extern HWND hwnd_method_height;
	extern HWND hwnd_resolution_height;

}