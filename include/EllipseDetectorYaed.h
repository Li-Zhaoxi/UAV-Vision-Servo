/*
This code is intended for academic use only.
You are free to use and modify the code, at your own risk.

If you use this code, or find it useful, please refer to the paper:

Michele Fornaciari, Andrea Prati, Rita Cucchiara,
A fast and effective ellipse detector for embedded vision applications
Pattern Recognition, Volume 47, Issue 11, November 2014, Pages 3693-3708, ISSN 0031-3203,
http://dx.doi.org/10.1016/j.patcog.2014.05.012.
(http://www.sciencedirect.com/science/article/pii/S0031320314001976)


The comments in the code refer to the abovementioned paper.
If you need further details about the code or the algorithm, please contact me at:

michele.fornaciari@unimore.it

last update: 23/12/2014
*/

/*

This class implements a very fast ellipse detector, codename: YAED (Yet Another Ellipse Detector)

*/

#ifndef _YAED_H
#define _YAED_H


#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <vector>
#include <uavdef.h>
//#include "Ellipse.h"
#include "common.h"
#include <time.h>

//#define DISCARD_CONSTRAINT_OBOX
//#define DISCARD_CONSTRAINT_CONVEXITY
//#define DISCARD_CONSTRAINT_POSITION
//#define DISCARD_CONSTRAINT_CENTER


// Data available after selection strategy. 
// They are kept in an associative array to:
// 1) avoid recomputing data when starting from same arcs
// 2) be reused in firther proprecessing
// See Sect [] in the paper
struct EllipseData
{
	bool isValid;
	float ta;
	float tb;
	float ra;
	float rb;
	cv::Point2f Ma;
	cv::Point2f Mb;
	cv::Point2f Cab;
	std::vector<float> Sa;
	std::vector<float> Sb;
};


class UAV_EXPORTS CEllipseDetectorYaed
{
	// Parameters

	// Preprocessing - Gaussian filter. See Sect [] in the paper
	cv::Size	_szPreProcessingGaussKernelSize;	// size of the Gaussian filter in preprocessing step
	double	_dPreProcessingGaussSigma;			// sigma of the Gaussian filter in the preprocessing step



	// Selection strategy - Step 1 - Discard noisy or straight arcs. See Sect [] in the paper
	int		_iMinEdgeLength;					// minimum edge size				
	float	_fMinOrientedRectSide;				// minumum size of the oriented bounding box containing the arc
	float	_fMaxRectAxesRatio;					// maximum aspect ratio of the oriented bounding box containing the arc

	// Selection strategy - Step 2 - Remove according to mutual convexities. See Sect [] in the paper
	float _fThPosition;

	// Selection Strategy - Step 3 - Number of points considered for slope estimation when estimating the center. See Sect [] in the paper
	unsigned _uNs;									// Find at most Ns parallel chords.

	// Selection strategy - Step 3 - Discard pairs of arcs if their estimated center is not close enough. See Sect [] in the paper
	float	_fMaxCenterDistance;				// maximum distance in pixel between 2 center points
	float	_fMaxCenterDistance2;				// _fMaxCenterDistance * _fMaxCenterDistance

	// Validation - Points within a this threshold are considered to lie on the ellipse contour. See Sect [] in the paper
	float	_fDistanceToEllipseContour;			// maximum distance between a point and the contour. See equation [] in the paper

	// Validation - Assign a score. See Sect [] in the paper
	float	_fMinScore;							// minimum score to confirm a detection
	float	_fMinReliability;					// minimum auxiliary score to confirm a detection

	float _fTaoCenters;
	// auxiliary variables
	cv::Size	_szImg;			// input image size
	std::vector<double> _timesHelper;
	std::vector<double> _times;	// _times is a vector containing the execution time of each step.
							// _times[0] : time for edge detection
							// _times[1] : time for pre processing
							// _times[2] : time for grouping
							// _times[3] : time for estimation
							// _times[4] : time for validation
							// _times[5] : time for clustering

	int ACC_N_SIZE;			// size of accumulator N = B/A
	int ACC_R_SIZE;			// size of accumulator R = rho = atan(K)
	int ACC_A_SIZE;			// size of accumulator A

	int* accN;				// pointer to accumulator N
	int* accR;				// pointer to accumulator R
	int* accA;				// pointer to accumulator A






public:

	//Constructor and Destructor
	CEllipseDetectorYaed(void);
	~CEllipseDetectorYaed(void);

	void DetectAfterPreProcessing(std::vector<struct Ellipse>& ellipses, cv::Mat1b& E, cv::Mat1f& PHI = cv::Mat1f());

	//Detect the ellipses in the gray image
	void Detect(cv::Mat1b& gray, std::vector<struct Ellipse>& ellipses);
	static void YAED_Ellipse_2_CV(std::vector<struct Ellipse>& ellipses, std::vector<cv::RotatedRect>& cvellipses);
	static void drawTransEllipses(std::vector<cv::RotatedRect> &elp, cv::Mat &ImgG);


	//Draw the first iTopN ellipses on output
	void DrawDetectedEllipses(cv::Mat3b& output, std::vector<struct Ellipse>& ellipses, int iTopN = 0, int thickness = 2);

	//Set the parameters of the detector
	void SetParameters(cv::Size	szPreProcessingGaussKernelSize,
		double	dPreProcessingGaussSigma,
		float 	fThPosition,
		float	fMaxCenterDistance,
		int		iMinEdgeLength,
		float	fMinOrientedRectSide,
		float	fDistanceToEllipseContour,
		float	fMinScore,
		float	fMinReliability,
		int     iNs
	);

	// Return the execution time
	double GetExecTime() { return _times[0] + _times[1] + _times[2] + _times[3] + _times[4] + _times[5]; }
	std::vector<double> GetTimes() { return _times; }

private:

	//keys for hash table
	static const ushort PAIR_12 = 0x00;
	static const ushort PAIR_23 = 0x01;
	static const ushort PAIR_34 = 0x02;
	static const ushort PAIR_14 = 0x03;

	//generate keys from pair and indicse
	uint inline GenerateKey(uchar pair, ushort u, ushort v);

	void PrePeocessing(cv::Mat1b& I, cv::Mat1b& DP, cv::Mat1b& DN);

	void RemoveShortEdges(cv::Mat1b& edges, cv::Mat1b& clean);

	void ClusterEllipses(std::vector<struct Ellipse>& ellipses);

	int FindMaxK(const std::vector<int>& v) const;
	int FindMaxN(const std::vector<int>& v) const;
	int FindMaxA(const std::vector<int>& v) const;

	int FindMaxK(const int* v) const;
	int FindMaxN(const int* v) const;
	int FindMaxA(const int* v) const;

	float GetMedianSlope(std::vector<cv::Point2f>& med, cv::Point2f& M, std::vector<float>& slopes);
	void GetFastCenter(std::vector<cv::Point>& e1, std::vector<cv::Point>& e2, EllipseData& data);


	void DetectEdges13(cv::Mat1b& DP, VVP& points_1, VVP& points_3);
	void DetectEdges24(cv::Mat1b& DN, VVP& points_2, VVP& points_4);

	void FindEllipses(cv::Point2f& center,
		VP& edge_i,
		VP& edge_j,
		VP& edge_k,
		EllipseData& data_ij,
		EllipseData& data_ik,
		std::vector<struct Ellipse>& ellipses
	);

	cv::Point2f GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);
	cv::Point2f _GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);



	void Triplets124(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<struct Ellipse>& ellipses
	);

	void Triplets231(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<struct Ellipse>& ellipses
	);

	void Triplets342(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<struct Ellipse>& ellipses
	);

	void Triplets413(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<struct Ellipse>& ellipses
	);

	void Tic(unsigned idx) //start
	{
		_timesHelper[idx] = 0.0;
		_times[idx] = (double)cv::getTickCount();
	};

	void Tac(unsigned idx) //restart
	{
		_timesHelper[idx] = _times[idx];
		_times[idx] = (double)cv::getTickCount();
	};

	void Toc(unsigned idx) //stop
	{
		_times[idx] = ((double)cv::getTickCount() - _times[idx])*1000. / cv::getTickFrequency();
		_times[idx] += _timesHelper[idx];
	};
};

#endif