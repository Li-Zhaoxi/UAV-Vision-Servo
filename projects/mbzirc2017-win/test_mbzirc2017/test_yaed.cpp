#include "test_functions.h"
#include <YAED/EllipseDetectorYaed.h>
#include <YAED/common.h>

int main_2_0()
{
	string img_path = "..\\..\\results\\ori.jpg";
	double t1, t2;
	Mat Img_C, Img_G;
	Img_C = imread(img_path);
	cvtColor(Img_C, Img_G, CV_RGB2GRAY);



	CEllipseDetectorYaed yaed;
	int		iThLength = 16;
	float	fThObb = 3.0f;
	float	fThPos = 1.0f;
	float	fTaoCenters = 0.05f;
	int 	iNs = 16;
	float	fThScoreScore = 0.8f;
	cv::Size	szPreProcessingGaussKernelSize = cv::Size(5, 5);
	double	dPreProcessingGaussSigma = 1.0;
	float	fDistanceToEllipseContour = 0.1f;	
	float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses

	int orows, ocols, srows, scols;
	orows = Img_C.rows, ocols = Img_C.cols;
	cv::Size sz = Img_G.size();

	t1 = (double)cv::getTickCount();
	float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;
	yaed.SetParameters(szPreProcessingGaussKernelSize,
		dPreProcessingGaussSigma,
		fThPos,
		fMaxCenterDistance,
		iThLength,
		fThObb,
		fDistanceToEllipseContour,
		fThScoreScore,
		fMinReliability,
		iNs
	);
	vector<struct Ellipse> ellsYaed;
	Mat1b tmpImgg = cv::Mat1b(Img_G);
	yaed.Detect(tmpImgg, ellsYaed);
	t2 = (double)cv::getTickCount();
	cout << "YAED:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;

	cvtColor(Img_G, Img_C, COLOR_GRAY2BGR);
	Mat3b tmpImgc = cv::Mat3b(Img_C);
	yaed.DrawDetectedEllipses(tmpImgc, ellsYaed, 0, 3);

	imshow("YAED", tmpImgc);
	imwrite("YAED.jpg", tmpImgc);
	waitKey();

	system("pause");
	return 0;
}