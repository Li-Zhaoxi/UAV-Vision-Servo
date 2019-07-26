#ifndef _SIMPLIFIED_EDLINES_H_
#define _SIMPLIFIED_EDLINES_H_

#include <uavdef.h>

#include "LineNode.h"




class UAV_EXPORTS Simplified_EDLines
{
public:
	Simplified_EDLines() {
		data = NULL; 
		//_paral_data = NULL;

		// initialize parameters
		dcols = drows = icols = irows = -1;
		lineFitErrThreshold = min_line_length = -1;
	}
	~Simplified_EDLines();
	void release();
	void create_EDLines(int drows, int dcols);
	void setParame(int min_line_length, double lineFitErrThreshold);

	Simplified_EDLines(int drows, int dcols);
	void runSimplified_EDLines(cv::Mat &ImgG);
	//void runSimplified_EDLines_tbb(cv::Mat &ImgG);
	std::vector< cv::Vec4i > LineSegments;
	
	void getLineGradients(cv::Mat &ImgG, std::vector< cv::Point2d> &linegradients) const;
	std::vector< cv::Point2d> LineGradients;

	void getFittedLineSegments(std::vector<cv::Vec<double, 11> > &fittedLineSegments) const;
	std::vector<cv::Vec<double, 11> > FittedLineSegments;

	void drawLineSegments4i(cv::Mat &ImgG, bool isShow = true);
	LineNode* const getOneNode(int i, int j) const { return data + DIDX(i, j); }

	double getMinLength() { return min_line_length; }

private:

	void findContours(unsigned char *_edge);
	void findContour(const int Wise[8][2], const int antiWise[8][2], uchar *Edge, int x, int y);
	int DIDX(int x, int y) const { return x*dcols + y; }
	int IIDX(int x, int y) const { return x*icols + y; }
	short SobelX3_3(unsigned char *_gray, int idxi) const 
	{
		return (2 * short(_gray[idxi + icols]) + short(_gray[idxi + icols - 1]) + short(_gray[idxi + icols + 1]) - 2 * short(_gray[idxi - icols]) - short(_gray[idxi - icols - 1]) - short(_gray[idxi - icols + 1]));
	}
	short SobelY3_3(unsigned char *_gray, int idxi) const
	{
		return (2 * short(_gray[idxi + 1]) + short(_gray[idxi - icols + 1]) + short(_gray[idxi + icols + 1]) - 2 * short(_gray[idxi - 1]) - short(_gray[idxi - icols - 1]) - short(_gray[idxi + icols - 1]));
	}
	cv::Point2d calGradxy(unsigned char *_gray, int idxdst, int idxded) const 
	{
		cv::Point2d res(0, 0);
		int idxiTemp, pointnum(0);
		LineNode *nodeST = data + idxdst;
		LineNode *nodeED = data + idxded;
		for (; nodeST != nodeED;)
		{
			idxiTemp = IIDX(nodeST->Location.x, nodeST->Location.y);
			res.x = res.x + SobelX3_3(_gray, idxiTemp);
			res.y = res.y + SobelY3_3(_gray, idxiTemp);
			pointnum++;
			nodeST = nodeST->nextAddress;
		}
		res.x /= pointnum;
		res.y /= pointnum;
		return res;
	}

	void getLineMat(double matLine[6], int idxdst, int idxded) const
	{
		for (int i = 0; i < 6; i++)
			matLine[i] = data[idxded].nodesMat[i] - data[idxdst].nodesMat[i];
	}

	// Vec11f is used for the fitted lines, the element is shown as follows:
	// [0] cos(theta),         [1] sin(theta),         [2] theta
	// [3] x coordinate on a point in a line,  [4] y coordinate on a point in a line
	// [5] fitting error of a line,            [6] a_3=cos(theta)*avgY-sin(theta)*avgX
	// [7] x coordinate on the start point,    [8] y coordinate on the start point
	// [9] x coordinate on the end point,      [10] y coordinate on the end point
	void fitLine2d(const double _matLine[6], cv::Vec<double, 11> &fitRes)  const
	{
		const int count_dot = int(_matLine[5] + 0.5);
		const double c[2] = { _matLine[2] / count_dot, _matLine[4] / count_dot };
		double dx2, dy2, dxy, t;
		dx2 = _matLine[0] / count_dot - c[0] * c[0];
		dy2 = _matLine[3] / count_dot - c[1] * c[1];
		dxy = _matLine[1] / count_dot - c[0] * c[1];
		t = atan2(2 * dxy, dx2 - dy2) / 2;
		fitRes[0] = cos(t);
		fitRes[1] = sin(t);
		fitRes[2] = t;
		fitRes[3] = c[0];
		fitRes[4] = c[1];
		fitRes[5] = sin(t)*sin(t)*dx2 + cos(t)*cos(t)*dy2 - sin(2 * t)*dxy;
		fitRes[6] = fitRes[0] * c[1] - fitRes[1] * c[0];
	}
	void transScale(cv::Vec<double, 11> &lineRes, double scale) const
	{
		lineRes[3] = lineRes[3] * scale;
		lineRes[4] = lineRes[4] * scale;
		lineRes[5] = lineRes[5] * scale*scale;
		lineRes[6] = lineRes[6] * scale;
	}


	static void calCannyThreshold(cv::Mat &ImgG, int &low, int &high);

private:
	int drows, dcols, irows, icols;
	LineNode *data;
	//Simplified_EDLines *_paral_data;

	cv::Mat imgCanny;
	std::vector< std::vector<cv::Point> > edgeContours, dpContours;
	std::vector<cv::Point> oneContour;
	std::vector<cv::Point> oneContourOpp;

private:
	// EDLines ����
	int min_line_length;
	double lineFitErrThreshold;
};


#endif