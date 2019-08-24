#include "test_functions.h"
#include <YAED/EllipseDetectorYaed.h>

int main_2_0()
{
	string img_path = "..\\..\\..\\results\\ori.jpg";
	double t1, t2;
	Mat Img_C, Img_G;
	Img_C = imread(img_path);
	resize(Img_C, Img_C, cv::Size(Img_C.cols * 2, Img_C.rows * 2));
	cvtColor(Img_C, Img_G, CV_RGB2GRAY);



	CEllipseDetectorYaed yaed;

	t1 = (double)cv::getTickCount();
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


int main_2_1()
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

	CEllipseDetectorYaed yaed;
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
		Mat1b tmpImgg = cv::Mat1b(Img_G);
		yaed.Detect(tmpImgg, ellsYaed);
		t2 = (double)cv::getTickCount();
		cout << "RVIBE:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		Mat3b tmpImgc = cv::Mat3b(Img_C);
		yaed.DrawDetectedEllipses(tmpImgc, ellsYaed);

		loop++;
		imshow("RES", Img_C);
		cv::waitKey(30);

	}

	system("pause");
	return 0;
}