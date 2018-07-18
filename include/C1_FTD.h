#pragma once

#include <iostream>
#include <vector>
#include <opencv2\opencv.hpp>
#include <uavdef.h>


struct CrossFeature
{
	bool isValid;
	cv::Point2d O;//store the intersection point. O
	cv::Point2d V_l;
	cv::Point2d V_r;
	cv::Point2d G_l;
	cv::Point2d G_r;
};
class UAV_EXPORTS C1_FTD
{
public:
	// Result Information
	bool isFind; // isFind = true: the cross target is find.
	cv::Point2f crossCenter; // the cross target center
	cv::Point2f cross4Points[4]; // 

	C1_FTD();
	void runC1_FTD(std::vector< cv::Vec<double, 11> > &detLineSegments, std::vector< cv::Point2d > &LineGradients);

	void setFixedParams(double _lambda_line,double _Error_Angle) // 在函数定义式就已经定义好的
	{
		lambda_line = _lambda_line;
		Error_Angle = _Error_Angle;
	}
	void setAdaptParams(double _FeaturesDistance) // 根据高度自动变化
	{
		FeaturesDistance = _FeaturesDistance;
	}
public: // drawfunctions;
	void drawCrossFeatures(cv::Mat &ImgG, bool isShow = false);
	void drawC1_FTD(cv::Mat &ImgG, bool isShow = false);
private:
	// Step 1: Cross Feature Extraction
	void CrossFeatureExtraction(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea);
	void CrossFeatureExtraction_tbb(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea);
	// Step 2: Rectangle Search Based on Cross Features
	void RectangleSearch(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< cv::Vec4i > &out_rects) const;
	// Step 3: Final Cross Selected
	void SelectFinalCross(std::vector< cv::Vec4i > &in_rects, std::vector<CrossFeature> &in_ValidFeatures);

	void ConstructAdjacency(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< std::vector<int> > &out_adjacency) const;
	// 对直线段进行矫正 
	static void CorrectLineSegments(const std::vector< cv::Vec<double, 11> > &in_fitLines, std::vector<cv::Vec4d> &out_correctedLines);



	std::vector<CrossFeature> cross_feature;
	std::vector<cv::Vec4d> correctedLines;
	std::vector< cv::Vec4i > searchedRects;
	// 已知阈值
	double CrossAngle; //理论上的十字夹角 PI/2
	double FeaturesDistance; // 十字特征点之间的理论距离


	// 需要调试的阈值
	double lambda_line;
	double Error_Angle;// 角度测量误差 (0°,45°)
  

	double calArea(std::vector< cv::Point2f > &points4)
	{
		cv::Point2f n1, n2;
		double res = 0;
		n1.x = points4[1].x - points4[0].x; n1.y = points4[1].y - points4[0].y;
		n2.x = points4[3].x - points4[0].x; n2.y = points4[3].y - points4[0].y;
		res = abs(n1.x*n2.y - n1.y*n2.x);

		n1.x = points4[1].x - points4[2].x; n1.y = points4[1].y - points4[2].y;
		n2.x = points4[3].x - points4[2].x; n2.y = points4[3].y - points4[2].y;
		res += abs(n1.x*n2.y - n1.y*n2.x);
		res = res / 2;
		return res;
	}

};