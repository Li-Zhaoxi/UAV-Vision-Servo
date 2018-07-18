#pragma once

#include <vector>
#include <opencv2\opencv.hpp>
#include <uavdef.h>

struct LineINEllipse
{
	int idxEllipse;   //对应椭圆的下标， 默认为-1
	std::vector<cv::Vec4i> line_data; //
	float lineScore;
	float cross4Direct[4];
};

class UAV_EXPORTS C2_FTD
{
public:
	C2_FTD();
	void create(int rows, int cols);
	C2_FTD(int rows, int cols);
	void runC2_FTD(std::vector<cv::RotatedRect> &detEllipses, std::vector<cv::Vec4i> &detLineSegments, int rows, int cols);

	void drawInsideLines(cv::Mat &ImgG);
	void drawC2_FTD(cv::Mat &ImgG);

	//计算最终结果提取区域变量
	cv::RotatedRect landEllipse; //目标椭圆
	cv::Vec4f landCross;         //目标十字信息
	bool isFind;                 //是否查找到

private:
	cv::Mat lineMap; //直线地图，用于查找在椭圆内的直线
	int drows, dcols, irows, icols;

	cv::Vec4i *_detLines_data;
	std::vector<LineINEllipse> line_in_ellipse;
	std::vector<cv::Vec6d> ellipse_parms;


	// 在irows,icols, 放置直线段的起点坐标信息
	void getLineMap(unsigned short *_lineMap, std::vector< cv::Vec4i > &cdtLines);

	//获取在每个椭圆内部的直线段 Step1 : Extract Candidate Line Segments of Cross
	void getInsideLines(const ushort *_lineMap, const std::vector<cv::RotatedRect> &cdtEllipse, std::vector<LineINEllipse> &LIE);
	 
	// 将椭圆形状参数转为一般方程6个参数
	static void ELPShape2Equation(const std::vector<cv::RotatedRect> &in_elps, std::vector<cv::Vec6d> &out_parms);


	void SlopeClustering(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detElps);
	void SelectFinalCross(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detEllipses);
	double twoL_means(std::vector<double> &input_lines, std::vector<int> &_bestLabels, double &center_1, double &center_2);
private:
	// 真实世界形状参数
	float circleDiameter;       //圆直径，标准为100cm
	float lineWidth;            //直线宽度，标准为10cm
	float circleWidth;          //圆环宽度，标准为10cm
	float dist2Pixel;           //18m距离下， 1cm对应的像素个数， 默认为0.5236px/cm
	
	// 可以自适应的参数
	float minCircleDiameter;    //内圆直径
	float maxCircleDiameter;    //外圆直径
	//需要调的参数
	float errCircleDiameter;    //直径误差


	float minratioCenter2Line, maxratioCenter2Line;

private:
	////判断椭圆是否超出尺度，是则为true，否则为false
	bool isOutOfRange(const cv::RotatedRect &elp) const
	{
		float aveRadius = (elp.size.height + elp.size.width) / 2;
		if (aveRadius > minCircleDiameter && aveRadius < maxCircleDiameter)
			return false;
		else
			return true;

	}

	bool isInEllipse(int x, int y, int elp_idx) const
	{
		const double *_parm = ellipse_parms[elp_idx].val;
		double val;

		val = _parm[0] * x*x + 2 * _parm[1] * x*y + _parm[2] * y*y + 2 * _parm[3] * x + 2 * _parm[4] * y + _parm[5];
		if (val > 0)
			return false;
		else
			return true;
	}

	bool CONSTRAINT_DIST(const cv::Vec4i &line4, const cv::RotatedRect &fitelp) const
	{
		double minDist(1), maxDist = std::min(fitelp.size.height, fitelp.size.width) / 4;
		double vecL[4], dist;

		vecL[0] = line4[2] - line4[0], vecL[1] = line4[3] - line4[1];
		vecL[2] = fitelp.center.x - line4[0]; vecL[3] = fitelp.center.y - line4[1];

		dist = abs(vecL[0] * vecL[3] - vecL[1] * vecL[2]) / sqrt(vecL[0] * vecL[0] + vecL[1] * vecL[1]);

		if (dist < maxDist && dist > minDist)
			return true;
		else
			return false;

	}
};