#pragma once

#include <uavdef.h>

#include <opencv2/opencv.hpp>
#include <iostream>

#include "CFG_RVIBE.h"
#include <vector>

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::Rect;
class UAV_EXPORTS RVIBE
{
public:
	RVIBE();
	void create(int rows, int cols, CFG_RVIBE &cfg_rvibe);
	RVIBE(int rows, int cols, CFG_RVIBE &cfg_rvibe);
	void run_RVIBE(cv::Mat &Img_G);
	void run_RVIBE_tbb(cv::Mat &Img_G);
	void run_RVIBE(cv::Mat Img_C, cv::Mat Img_G);
	//void drawRVIBE(cv::Mat &Img_G);
	vector< Rect > findRects;
	vector< int > validRects;
	void init();
	void release();
protected:
	uchar **_m_sample;


	uchar *_m_mask;
	cv::Mat mask_src, mask_dst;

	uchar *_m_foregroundMatchCount;
	cv::RNG rng;

	int slipePickRows, slipePickCols;
	ushort *_slipePick;
	void set_slipePick(uchar *_mask_data);

	int slipeSmallRows, slipeSmallCols;
	ushort *_slipeSmall;
	void set_slipeSmall(ushort *_slipepick);

	cv::Mat findRect;
	uchar *_findRect;

	vector< vector<cv::Point> > contours;

	void processFirstFrame(const uchar *_image);
	void testAndUpdate(const uchar *_image);
private:
	bool isFirst;
	int imgRows, imgCols;
	void set_Paras(CFG_RVIBE &cfg_rvibe);
private://参数实例列表
	int NUM_SAMPLES, MIN_MATCHES, RADIUS, SUBSAMPLE_FACTOR;
	int slipeWindow, rectWindow;
	float minAreaRatio;
	int maxRect;
};