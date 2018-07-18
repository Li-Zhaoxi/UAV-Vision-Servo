#include "UAV_STRATEGY.h"
#include <DataTransmission.h>
#include <definition.h>
#include <logfiles_io.h>

namespace UAV_STRATEGY
{
	unsigned int image_type;
	double uav_height = 20;

	unsigned char isSend = 0;
	unsigned int pc_type = 0;
	double data[4];
	double pilpline_time = 0;

	unsigned char RS = UAV_WAITING;
	unsigned char IS = UAV_WAITING;
	unsigned char SS = UAV_WAITING;

	double time_start, time_end, time_pipeline;


	double height_change_resolution = 3; //小于这个高度将转为540*270分辨率
	double height_change_methods = 10;  // 小于这个高度将转为检测10字算法

	RVIBE rvibe;
	Simplified_EDLines sedlines;
	CEllipseDetectorYaed yaed[4];
	C2_FTD c2_ftd;
	C1_FTD c1_ftd;

	HANDLE handle_strategy = NULL;

	bool running_servo = true;

	void CreateStrategy()
	{
		CFG_RVIBE cfg_rvibe("RVIBE");
		rvibe.create(540, 960, cfg_rvibe);
		sedlines.create_EDLines(540, 960);
		c2_ftd.create(540, 960);

		handle_strategy = CreateThread(NULL, 0, STRATEGY_CALLBACK, NULL, 0, NULL);

	}
	void DestoryStrategy()
	{

	}

	// 视频流中有个线程是专门用于根据需求获取对应的分辨率图像
	DWORD WINAPI STRATEGY_CALLBACK(LPVOID lpParam)
	{
		double t1, t2, use_time, sleep_time;
		while (running_servo)
		{
			t1 = cv::getTickCount();
			if (VIDEO_STREAM::isUpdateImage == true) //获得了新的图片
			{
				EnterCriticalSection(&VIDEO_STREAM::img_lock);
				time_start = t1; // 设置起始时间;
				cv::Mat ImgG = cv::Mat(540, 960, CV_8UC1, VIDEO_STREAM::img_gray);
				sedlines.runSimplified_EDLines(ImgG);
				sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
				sedlines.getLineGradients(ImgG, sedlines.LineGradients);
				c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);


				
				VIDEO_STREAM::ImgR = ImgG.clone();
				c1_ftd.drawC1_FTD(VIDEO_STREAM::ImgR);

				VIDEO_STREAM::ImgT = ImgG.clone();
				c1_ftd.drawCrossFeatures(VIDEO_STREAM::ImgT);
//				sedlines.drawLineSegments4i(VIDEO_STREAM::ImgT, false);
				LeaveCriticalSection(&VIDEO_STREAM::img_lock);

				VIDEO_STREAM::isUpdateImage = false;
				isSend = 1;
			}
			t2 = cv::getTickCount();
			
			

			use_time = (t2 - t1) * 1000 / cv::getTickFrequency();
			sleep_time = 30 - use_time;
			if (sleep_time <= 0) sleep_time = 1;

			Sleep((DWORD)sleep_time);
		}
		return 0;
	}


	

}


void UAV_STRATEGY::UAV_SOLUTION_1()
{
	unsigned char RS_1 = RS & 0xf0;
	unsigned char RS_2 = RS & 0x0f;
	float highNow = uav_height;


	// 无人机退出循环
	if (RS_1 == UAV_EXIT)
	{
		sedlines.release();
		rvibe.release();


		running_servo = false;
		return;
	}
	if (highNow < 0) return;

	// 无人机暂停一切算法
	if (RS_1 == UAV_WAITING)
	{
		return;
	}


	// C2 Detection 
	if (RS_1 == UAV_C2_DETECTION && IS == InnerState::CV_TARGET_DETECTION)
	{
		cv::Mat ImgG = cv::Mat(540, 960, CV_8UC1, VIDEO_STREAM::img_gray);
		VIDEO_STREAM::ImgR = ImgG.clone();

		switch (RS_2)
		{
		case ReceiveState::MOTiON_DETECTION:
		{
			rvibe.run_RVIBE_tbb(VIDEO_STREAM::ImgR);
			
			break;
		}
		case ReceiveState::UNIFORM_ROI:
		{
			break;
		}
		default:
			return;
			break;
		}

		// 采用OpenMP 计算ROI

		//
		

		return;
	}
	if (RS_1 == UAV_C2_DETECTION&&IS == InnerState::CV_TARGET_CONFIRM)
	{
		// Confirm Target ?


	}


	// C1 Detection 
	if (RS_1 == UAV_C1_DETECTION) // C1_FTD涉及到高度转换，即在低空时候目标很大，可以进行缩放处理
	{
		//if (highNow < height_change_resolution)
		//{

		//}
		cv::Mat ImgG = cv::Mat(540, 960, CV_8UC1, VIDEO_STREAM::img_gray);
		
		VIDEO_STREAM::ImgR = ImgG.clone();
		sedlines.runSimplified_EDLines(VIDEO_STREAM::ImgR);
		sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
		sedlines.getLineGradients(VIDEO_STREAM::ImgR, sedlines.LineGradients);
		c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
		
		UAV_IO::SaveC1_FTD(VIDEO_STREAM::globalID, c1_ftd.isFind, c1_ftd.crossCenter, c1_ftd.cross4Points);// Save log files.

		c1_ftd.drawC1_FTD(VIDEO_STREAM::ImgR);
		
		if (c1_ftd.isFind)
		{
			float p2d = DataTran::calPixcelAndDist(highNow, UAV_CLB_PIXEL2DIST);
			data[0] = (c1_ftd.crossCenter.x - ImgG.rows / 2)*p2d / 100;
			data[1] = (c1_ftd.crossCenter.y - ImgG.cols / 2)*p2d / 100;
			data[2] = data[3] = 0;
			SS = SendState::CV_C1_DETECTION_YES;
		}
		else
			SS = SendState::CV_C1_DETECTION_NO;
		
		time_end = cv::getTickCount();
		time_pipeline = (time_end - time_start) * 1000 / cv::getTickFrequency();

		UAV_IO::SaveSS(VIDEO_STREAM::globalID, SS, data, time_pipeline);// Save log files.

		if (SS == SendState::CV_C1_DETECTION_YES)
			DataTran::SendData2UAV(SS, data[0], data[1], data[2], data[3], time_pipeline);

		return;
	}





}



void UAV_STRATEGY::SetChangeHeights(HWND hwnd_methods_height, double &methods_height, HWND hwnd_resolution_height, double &resolution_height)
{
	//float p1 = DataTran::highPara_1;
	//float p2 = DataTran::highPara_2;

	float temp1, temp2;
	const float err = 1, width = 10;
	// 计算分辨率转换
//	temp1 = 

}