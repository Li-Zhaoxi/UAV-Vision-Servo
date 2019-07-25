#include "test_functions.h"

#include <RVIBE/CFG_RVIBE.h>
#include <RVIBE/RVIBE.h>

int main_3_0()
{
	string str_path;
	str_path = "..\\..\\..\\datasets\\MBZIRC 18m #1.mp4";

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
	int rvibeNumber = 0;
	CFG_RVIBE cfg("RVIBE"); // 需要配置文件
	vector< vector<Rect> > allRects;
	RVIBE rvibe(270, 480, cfg);
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
		resize(Img_C, Img_C, cv::Size(480, 270));
		srows = Img_C.rows, scols = Img_C.cols;
		cvtColor(Img_C, Img_G, CV_RGB2GRAY);
		t1 = (double)cv::getTickCount();
		//rvibe.run_RVIBE(Img_G);
		rvibe.run_RVIBE_tbb(Img_G);
		t2 = (double)cv::getTickCount();
		cout << "RVIBE:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		rvibeNumber = (int)rvibe.findRects.size();
		allRects.push_back(rvibe.findRects);
		for (int i = 0; i < rvibeNumber; i++)
		{
			imgROI = Img_C(rvibe.findRects[i]).clone();
		}
		for (int i = 0; i < rvibeNumber; i++)
		{
			cv::rectangle(Img_C, rvibe.findRects[i], cv::Scalar(0, 0, 255), 2);
		}
		loop++;
		imshow("RES", Img_C);
		cv::waitKey(30);
		cout << loop << endl;
	}


	system("pause");
	return 0;
}