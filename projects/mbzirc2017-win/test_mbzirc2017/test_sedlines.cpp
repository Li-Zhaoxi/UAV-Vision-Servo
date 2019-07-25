#include "test_functions.h"

#include <SEDLines/Simplified_EDLines.h>

int main_1_0()
{
	string img_path = "..\\..\\results\\ori.jpg";
	double t1, t2;
	Mat Img_C, Img_G;
	Img_C = imread(img_path);
	cvtColor(Img_C, Img_G, CV_RGB2GRAY);


	Simplified_EDLines sedlines(960, 540);

	t1 = (double)cv::getTickCount();
	sedlines.runSimplified_EDLines(Img_G);
	t2 = (double)cv::getTickCount();
	cout << "SEDLines:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
	sedlines.drawLineSegments4i(Img_G, false);

	imshow("Ori", Img_C);
	imshow("SEDLines", Img_G);
	imwrite("SEDLines.jpg", Img_G);
	waitKey();

	system("pause");
	return 0;

}


int main_1_1()
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

	Simplified_EDLines sedlines(960, 540);


	int orows, ocols, srows, scols;
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
		t1 = (double)cv::getTickCount();
		sedlines.runSimplified_EDLines(Img_G);
		t2 = (double)cv::getTickCount();
		cout << "SEDLines:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;

		sedlines.drawLineSegments4i(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(30);

	}
	system("pause");
	return 0;
}

