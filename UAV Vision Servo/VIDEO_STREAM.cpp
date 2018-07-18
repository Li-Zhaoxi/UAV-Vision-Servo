#include "VIDEO_STREAM.h"

namespace VIDEO_STREAM
{
	cv::VideoCapture cap;
	cv::Mat ImgC, ImgR, ImgT;
	unsigned char img_gray[1080 * 1920];
	unsigned int imageType = COPY_RESOLUTION_960_540;
	bool isUpdateImage = false;
	int frameNumber = 0;
	int globalID = 0;

	HANDLE phCaptureObject;
	CRITICAL_SECTION img_lock;
	
	bool Open_Ave_C353()
	{
		DWORD pdwDeviceNum = 0;
		LPWSTR szDeviceName = new WCHAR[256];


		AVerGetDeviceNum(&pdwDeviceNum);
		std::cout << "数据采集卡个数为： " << pdwDeviceNum << std::endl;
		if (pdwDeviceNum == 0)
		{
			std::cout << "检测不到数据采集卡，退出程序" << std::endl;
			return false;
		}
		AVerGetDeviceName(0, szDeviceName);
		for (int i = 0; i<256 && szDeviceName[i] != 52685; i++)
			std::cout << (char)szDeviceName[i];
		std::cout << std::endl;
		return true;
	}
	bool Set_AVe_C353_Stream(bool isShow, HWND hwnd)
	{
		LONG getFunInfor;
		getFunInfor = AVerCreateCaptureObject(0, hwnd, &phCaptureObject);
		if (getFunInfor != 0)
		{
			printf("创建视频对象失败，错误编号为： %d", getFunInfor); 
			return false;
		}
		getFunInfor = AVerSetVideoSource(phCaptureObject, VIDEOSOURCE_HDMI);
		if (getFunInfor != 0)
		{
			printf("设置视频资源失败，错误编号为： %d", getFunInfor);
			return false;
		}
		VIDEO_RESOLUTION pVideoResolution;
		pVideoResolution.dwVersion = 1;
		pVideoResolution.dwVideoResolution = VIDEORESOLUTION_1920X1080;
		pVideoResolution.bCustom = FALSE;
		getFunInfor = AVerSetVideoResolutionEx(phCaptureObject, &pVideoResolution);
		if (getFunInfor != 0)
		{
			printf("设置视频分辨率失败，错误编号为： %d", getFunInfor);
			return false;
		}

		if (isShow)
		{
			RECT rectVideoWnd;
			rectVideoWnd.left = 0;
			rectVideoWnd.top = 0;
			rectVideoWnd.right = 480;
			rectVideoWnd.bottom = 270;
			getFunInfor = AVerSetVideoWindowPosition(phCaptureObject, rectVideoWnd);
			if (getFunInfor != 0)
			{
				printf("设置视频显示位置失败，错误编号为： %d", getFunInfor);
				return false;
			}
		}
		getFunInfor = AVerStartStreaming(phCaptureObject);
		if (getFunInfor != 0)
		{
			printf("开始视频流失败，错误编号为： %d", getFunInfor);
			return false;
		}
		return true;
	}
	void Delete_Ave_C353()
	{
		LONG infor;
		infor = AVerCaptureVideoSequenceStop(phCaptureObject);
		infor = AVerHwCaptureStreamStop(phCaptureObject);

		AVerStopStreaming(phCaptureObject);
		AVerDeleteCaptureObject(phCaptureObject);

	}

	HANDLE handle_video_stream;
	bool initVideoStream()
	{
		InitializeCriticalSection(&img_lock);
		ImgC = UAV_GUI::Img_back.clone();
		ImgR = UAV_GUI::Img_back.clone();
		ImgT = UAV_GUI::Img_back.clone();

		if (UAV_CONFIG::video_type == 1) //文件视频流
		{
			cap.open(UAV_CONFIG::video_path);
			if (!cap.isOpened())
			{
				printf("视频文件打开失败！\n");
				return false;
			}
		}
		else if (UAV_CONFIG::video_type == 2) //云台视频流
		{
			bool res;
			res = Open_Ave_C353();
			if (res == false)
				return false;
			res = Set_AVe_C353_Stream(true, UAV_GUI::hwnd_view_1); // 准备显示视频
			if (res == false)
				return false;

			// 设置回调函数
			LONG pInfo;
			VIDEO_CAPTURE_INFO CaptureInfo;
			CaptureInfo.dwCaptureType = CT_SEQUENCE_FRAME;//指定以帧为单位抓取
			CaptureInfo.dwSaveType = ST_CALLBACK;//传递YUY2数据给回调函数
			CaptureInfo.bOverlayMix = FALSE;//不在图像上添加文字图像等信息
			CaptureInfo.lpCallback = Camera_CALLBACK;
			pInfo = AVerCaptureVideoSequenceStart(phCaptureObject, CaptureInfo);
			if (pInfo != 0)
			{
				printf("回调函数抓取图像失败，错误编号为： %d\n", pInfo);
				return false;
			}
		}
		else
			return false;

		handle_video_stream = CreateThread(NULL, 0, VIDEO_STREAM_CALLBACK, NULL, 0, NULL);
		return true;
		
	}
	// 这个函数主要是负责显示数据图，对于文件视频文件，这个函数会更新原始图像数据，并根据需求获取需要的分辨率图像。

	
	DWORD WINAPI VIDEO_STREAM_CALLBACK(LPVOID lpParam)
	{
		double t1, t2;
		int sleep_t;
		while (1)
		{
			// 传入原始图
			t1 = cv::getTickCount();

			// 云台相机内部有自己更新图像的函数
			if (UAV_CONFIG::video_type == 1)
			{
				cap >> ImgC;
				if (ImgC.empty())
					break;
				frameNumber++;
				cv::imshow(UAV_GUI::tilde_view_1, ImgC);

				cv::Mat Img_temp;
				if (ImgC.channels() > 1)
					cv::cvtColor(ImgC, Img_temp, CV_RGB2GRAY);
				else
					Img_temp = ImgC.clone();
				if (isUpdateImage == false)
				{
					globalID = frameNumber;
					switch (imageType)
					{
					case COPY_RESOLUTION_1920_1080:
					{
						memcpy(img_gray, Img_temp.data, sizeof(unsigned char)*NUMBER_1920_1080);
						break;
					}
					case COPY_RESOLUTION_960_540:
					{
						unsigned char *_data = Img_temp.data;
						int idx_i, idx_o;
						for (int i = 0; i < 540; i++)
						{
							idx_i = i * 960;
							idx_o = i * 1920;
							for (int j = 0; j < 960; j++)
							{
								img_gray[idx_i + j] = _data[(idx_o + j) << 1];
							}
						}
					}
					default:
						break;
					}
					isUpdateImage = true;
				}
				
			}

			EnterCriticalSection(&img_lock);
			cv::imshow(UAV_GUI::tilde_view_2, ImgR);
			cv::imshow(UAV_GUI::tilde_view_3, ImgT);
			LeaveCriticalSection(&VIDEO_STREAM::img_lock);
			t2 = cv::getTickCount();
			sleep_t = 30 - (t2 - t1) * 1000 / cv::getTickFrequency();
			if (sleep_t <= 0) sleep_t = 1;
			Sleep(sleep_t);
		}
		return DWORD(0);
	}
	BOOL WINAPI Camera_CALLBACK(VIDEO_SAMPLE_INFO VideoInfo, BYTE *pbData, LONG lLength, __int64 tRefTime, LONGPTR lUserData)
	{
		frameNumber++;
		if (isUpdateImage == false)
		{
			globalID = frameNumber;
			int idx_i, idx_o;
			switch (imageType)
			{
			case COPY_RESOLUTION_1920_1080:
			{
				for (int i = 0; i < NUMBER_1920_1080; i++)
					img_gray[i] = pbData[i << 1];
				isUpdateImage = true;
				break;
			}
			case COPY_RESOLUTION_960_540:
			{
				for (int i = 0; i < 540; i++)
				{
					idx_i = i * 960;
					idx_o = i * 1920;
					for (int j = 0; j < 960; j++)
					{
						img_gray[idx_i + j] = pbData[(idx_o + j) << 2];
					}
				}
				isUpdateImage = true;
				break;
			}
			case COPY_RESOLUTION_480_270:
			{
				for (int i = 0; i < 270; i++)
				{
					idx_i = i * 480;
					idx_o = i * 1920;
					for (int j = 0; j < 480; j++)
					{
						img_gray[idx_i + j] = pbData[(idx_o + j) << 3];
					}
				}
				isUpdateImage = true;
				break;
			}
			case COPY_RESOLUTION_240_135:
			{
				for (int i = 0; i < 135; i++)
				{
					idx_i = i * 240;
					idx_o = i * 1920;
					for (int j = 0; j < 240; j++)
					{
						img_gray[idx_i + j] = pbData[(idx_o + j) << 4];
					}
				}
				isUpdateImage = true;
				break;
			}
			default:
				break;
			}
		}
		return true;
	}
}