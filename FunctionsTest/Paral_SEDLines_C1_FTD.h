#pragma once

#include <Simplified_EDLines.h>
#include <C1_FTD.h>
#include <opencv2\opencv.hpp>

class Fast_EDLines_C1 :public cv::ParallelLoopBody
{
public:
	Fast_EDLines_C1(
		Simplified_EDLines *__multi_SEDLines,
		C1_FTD *__multi_C1,
		cv::Mat *__multi_roi,
		cv::Mat &_ImgG
	) :_multi_SEDLines(__multi_SEDLines), _multi_C1(__multi_C1), _multi_roi(__multi_roi), ImgG(_ImgG)
	{}


	void operator() (cv::Range& range) const
	{
		const int begin = range.start;
		const int end = range.end;

		for (int i = begin; i < end; ++i)
		{

		}
	}
private:
	Simplified_EDLines *_multi_SEDLines;
	C1_FTD *_multi_C1;
	cv::Mat *_multi_roi;

	cv::Mat &ImgG;
};