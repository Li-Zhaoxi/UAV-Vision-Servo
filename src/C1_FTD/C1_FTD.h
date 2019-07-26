#ifndef _C1_FTD_H_
#define _C1_FTD_H_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
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

    C1_FTD();// 鍒濆鍖栫畻娉曞唴缃弬鏁	
	void initmethodParms(double _lambda_line, double error_angle)
    {
        this->lambda_line = _lambda_line, this->Error_Angle = error_angle;
    }
    void runC1_FTD(std::vector< cv::Vec<double, 11> > &detLineSegments, std::vector< cv::Point2d > &LineGradients);

    // 鍚堜綔鍙傛暟锛屽湪绫诲垵濮嬪寲鏃跺€欏氨宸茬粡瀹氫箟濂界殑
    void setFixedParams(double _lambda_line,double _Error_Angle)
    {
        lambda_line = _lambda_line;
        Error_Angle = _Error_Angle;
    }

    // 鏍规嵁娣卞害+鐩告満鍐呭弬锛岀畻濂藉搴﹀€硷紝杩涜鏇存柊
    void setAdaptParams(double _FeaturesDistance)
    {
        FeaturesDistance = _FeaturesDistance;
    }
public: // drawfunctions;
    void drawCrossFeatures(cv::Mat &ImgG, bool isShow = false);
    void drawC1_FTD(cv::Mat &ImgG, bool isShow = false);

public: // 用于非实例化调用的函数
	static void CrossFeatureExtraction(
		std::vector<cv::Vec4d> &in_correctedLines,
		std::vector< cv::Point2d > &LinesGrad,
		std::vector<CrossFeature> &out_crossfea,
		double Error_Angle,
		double lambda_line); // 无需实例化即可调用得函数,目前只考虑PI/2的情况，即不存在透视变换的情况
	static void drawCrossFeatures(std::vector<CrossFeature> &in_crossfea, cv::Mat &ImgG, bool isShow = false);
	static void CorrectLineSegments(const std::vector< cv::Vec<double, 11> > &in_fitLines, std::vector<cv::Vec4d> &out_correctedLines);

private:
    // Step 1: Cross Feature Extraction
    void CrossFeatureExtraction(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea);
    void CrossFeatureExtraction_tbb(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea);
	
    // Step 2: Rectangle Search Based on Cross Features
    void RectangleSearch(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< cv::Vec4i > &out_rects) const;
    // Step 3: Final Cross Selected
    void SelectFinalCross(std::vector< cv::Vec4i > &in_rects, std::vector<CrossFeature> &in_ValidFeatures);

    void ConstructAdjacency(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< std::vector<int> > &out_adjacency) const;

    
    std::vector<CrossFeature> cross_feature;
    std::vector<cv::Vec4d> correctedLines;
    std::vector< cv::Vec4i > searchedRects;
    // 锟斤拷知锟斤拷值
    double CrossAngle; //锟斤拷锟斤拷锟较碉拷十锟街夹斤拷 PI/2
    double FeaturesDistance; // 十锟斤拷锟斤拷锟斤拷锟斤拷之锟斤拷锟斤拷锟斤拷劬锟斤拷锟

    // 锟斤拷要锟斤拷锟皆碉拷锟斤拷值
    double lambda_line;
    double Error_Angle;// 锟角度诧拷锟斤拷锟斤拷锟(0锟斤拷,45锟斤拷)


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


#endif