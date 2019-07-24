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