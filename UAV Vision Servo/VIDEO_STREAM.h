#pragma once
#include <opencv2\opencv.hpp>
#include <Windows.h>
#include <AVerCapAPI_Basic.h>
#include <iostream>
#include "UAV_CONFIG.h"
#include "UAV_GUI.h"
#include <definition.h>

namespace VIDEO_STREAM
{
	extern cv::VideoCapture cap; // 视频文件流


	extern cv::Mat ImgC, ImgR, ImgT;
	extern unsigned char img_gray[1080 * 1920];
	extern unsigned int imageType;
	extern bool isUpdateImage;
	extern int frameNumber;
	extern int globalID;

	extern HANDLE phCaptureObject;
	bool Open_Ave_C353(); // 开启视频采集卡
	bool Set_AVe_C353_Stream(bool isShow, HWND hwnd);
	void Delete_Ave_C353();

	bool initVideoStream();

	extern HANDLE handle_video_stream;

	DWORD WINAPI VIDEO_STREAM_CALLBACK(LPVOID lpParam);
	BOOL WINAPI Camera_CALLBACK(VIDEO_SAMPLE_INFO VideoInfo, BYTE *pbData, LONG lLength, __int64 tRefTime, LONGPTR lUserData);


	extern CRITICAL_SECTION img_lock;
}