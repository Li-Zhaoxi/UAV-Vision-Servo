#include "test_functions.h"

#include <SEDLines/Simplified_EDLines.h>
#include <YAED/EllipseDetectorYaed.h>
#include <C2_FTD/C2_FTD.h>


int main_5_0()
{
	string str_path;
	str_path = "..\\..\\..\\datasets\\MBZIRC 18m #1.mp4";
	//str_path = "E:\\v14.mp4";
	cv::VideoCapture cap(str_path);
	if (!cap.isOpened())
	{
		cout << "视频打开失败" << endl;
		system("pause");
		return -10;
	}
	else
		cout << "视频已打开" << endl;
	Mat Img_C, Img_G;

	C2_FTD c2_ftd(540, 960);
	CEllipseDetectorYaed yaed;
	Simplified_EDLines sedlines(540, 960);

	// Parameters Settings (Sect. 4.2)
	int		iThLength = 16;
	float	fThObb = 3.0f;
	float	fThPos = 1.0f;
	float	fTaoCenters = 0.05f;
	int 	iNs = 16;
	float	fThScoreScore = 0.8f;
	// Gaussian filter parameters, in pre-processing
	cv::Size	szPreProcessingGaussKernelSize = cv::Size(5, 5);
	double	dPreProcessingGaussSigma = 1.0;
	float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
	float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


	int orows, ocols, srows, scols;
	Mat imgROI;
	int loop = 0;

	double t1, t2;
	while (1)
	{
		printf("loop: %d", loop);
		cap >> Img_C;
		if (Img_C.empty())
			break;
		orows = Img_C.rows, ocols = Img_C.cols;
		resize(Img_C, Img_C, cv::Size(960, 540));
		srows = Img_C.rows, scols = Img_C.cols;
		cvtColor(Img_C, Img_G, CV_RGB2GRAY);
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
		vector<cv::RotatedRect> elps;
		Mat1b tmpImgg = cv::Mat1b(Img_G);
		yaed.Detect(tmpImgg, ellsYaed);
		CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);
		//CEllipseDetectorYaed::drawTransEllipses(elps, Img_G);

		sedlines.runSimplified_EDLines(Img_G);

		c2_ftd.runC2_FTD(elps, sedlines.LineSegments, Img_G.rows, Img_G.cols);



		t2 = (double)cv::getTickCount();
		cout << "C2-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		//sedlines.drawLineSegments4i(Img_G);
		c2_ftd.drawInsideLines(Img_G);
		//yaed.DrawDetectedEllipses(cv::Mat3b(Img_C), ellsYaed);
		c2_ftd.drawC2_FTD(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(30);

	}


	system("pause");
	return 0;
}


int main_5_1()
{
	string img_path = "..\\..\\..\\results\\ori.jpg";
	double t1, t2;
	Mat Img_C, Img_G;
	Img_C = imread(img_path);
	resize(Img_C, Img_C, cv::Size(960, 540));
	cvtColor(Img_C, Img_G, CV_RGB2GRAY);


	C2_FTD c2_ftd(540, 960);
	CEllipseDetectorYaed yaed;
	Simplified_EDLines sedlines(540, 960);


	// 检测椭圆
	t1 = (double)cv::getTickCount();
	vector<struct Ellipse> ellsYaed;
	vector<cv::RotatedRect> elps;
	Mat1b tmpImgg = cv::Mat1b(Img_G);
	yaed.Detect(tmpImgg, ellsYaed);
	CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);

	sedlines.runSimplified_EDLines(Img_G);

	c2_ftd.runC2_FTD(elps, sedlines.LineSegments, Img_G.rows, Img_G.cols);

	t2 = (double)cv::getTickCount();
	cout << "C2-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;

	c2_ftd.drawC2_FTD(Img_G);

	waitKey();

	system("pause");
	return 0;
}