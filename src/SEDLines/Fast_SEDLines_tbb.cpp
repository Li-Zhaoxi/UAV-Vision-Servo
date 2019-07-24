#include "Simplified_EDLines.h"




class SEDLines_findContours :public cv::ParallelLoopBody
{
public:
	SEDLines_findContours();

	void operator()(cv::Range& range) const
	{
		const int rows = imgCanny.rows, cols = imgCanny.cols;

		const int begin = range.start;
		const int end = range.end;

		int pitch_num = end - begin;
		int pitch_height = rows / pitch_num, pitch_width = cols / pitch_num;
		

		for (int i = begin; i < end; ++i)
		{
			int roi_rows = pitch_height + 2, roi_cols = pitch_width + 2;
			unsigned char *_roi = new unsigned char[roi_rows*roi_cols];
			unsigned char *_imgCanny = (unsigned char*)imgCanny.data;
			setROIFrame0(_roi, roi_rows, roi_cols);
			copyEdge2ROI(_roi, _imgCanny, pitch_height, pitch_width);
			// 对ROI进行边缘查找

			delete[] _roi;

		}
	}
private:
	int min_edge_num;
	cv::Mat &imgCanny;
	std::vector< std::vector<cv::Point> > all_edges[8];
private:
	// 边框置为0
	void setROIFrame0(unsigned char *_data, int rows, int cols) const;
	void copyEdge2ROI(unsigned char *_data, unsigned char* _edge, int pitch_rows, int pitch_cols) const;
	void findContours(unsigned char *_edge, cv::Point &roi_start, int rows, int cols, std::vector< std::vector<cv::Point> > &edge_contours) const;
};

void SEDLines_findContours::setROIFrame0(unsigned char *_data, int rows, int cols) const
{
	int idx_first = (rows - 1)*cols;
	for (int i = 0; i < cols; i++)
	{
		_data[i] = 0;
		_data[idx_first + i] = 0;
	}
	for (int i = 1; i < rows - 1; i++)
	{
		_data[i*cols] = 0;
		_data[i*cols + cols - 1] = 0;
	}
}

void SEDLines_findContours::copyEdge2ROI(unsigned char *_data, unsigned char* _edge, int pitch_rows, int pitch_cols) const
{
	for (int i = 0; i < pitch_rows; i++)
	{
		unsigned char *_ptr_data = _data + (i + 1)*(pitch_cols + 2) + 1;
		unsigned char *_ptr_edge = _edge + i*pitch_cols;
		memcpy(_ptr_data, _ptr_edge, sizeof(unsigned char)*pitch_cols);
	}
}

void SEDLines_findContours::findContours(unsigned char *_edge, cv::Point &roi_start, int rows, int cols, std::vector< std::vector<cv::Point> > &edge_contours) const
{
}

