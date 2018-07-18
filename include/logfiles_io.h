#pragma once
#include <uavdef.h>
#include <vector>
#include <iostream>
#include <opencv2\opencv.hpp>

namespace UAV_IO
{
	// 日志文件初始化
	UAV_EXPORTS bool InitLogFiles(bool openRVIBE, bool openYAED, bool openSEDLines, bool openC1_FTD, bool openC2_FTD);
	UAV_EXPORTS void ReleaseLogFiles();

	// 保存数据存储与发送消息
	UAV_EXPORTS void SaveRS(int idNum, unsigned char RS, double uav_height);
	UAV_EXPORTS void SaveSS(int idNum, unsigned char SS, double data[4], double pipeline_time);

	// 保存算法信息
	UAV_EXPORTS void SaveRVIBE(int idNum, std::vector<cv::Rect> &findRect);
	UAV_EXPORTS void SaveYAED(int idNum, std::vector<cv::RotatedRect> &detEllipses);
	UAV_EXPORTS void SaveSEDLines(int idNum, std::vector<cv::Vec4i> &LineSegments);
	UAV_EXPORTS void SaveC1_FTD(int idNum, char isFind, cv::Point2f crossCenter, cv::Point2f cross4Points[4]);
	UAV_EXPORTS void SaveC2_FTD(int idNum, char isFind, cv::RotatedRect &landEllipse, cv::Vec4f landCross);
}


