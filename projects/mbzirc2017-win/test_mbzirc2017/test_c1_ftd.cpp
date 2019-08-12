#include "test_functions.h"

#include <SEDLines/Simplified_EDLines.h>
#include <C1_FTD/C1_FTD.h>

int main_4_0()
{
	string str_path;
	str_path = "..\\..\\..\\datasets\\MBZIRC 5m #2.mp4";
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

	Simplified_EDLines sedlines(540, 960);
	C1_FTD c1_ftd;

	int orows, ocols, srows, scols;
	int loop = 0;

	double t1, t2;
	while (1)
	{
		//cout << loop << endl;

		cap >> Img_C;
		if (Img_C.empty())
			break;
		orows = Img_C.rows, ocols = Img_C.cols;
		resize(Img_C, Img_C, cv::Size(480, 270));
		srows = Img_C.rows, scols = Img_C.cols;
		cvtColor(Img_C, Img_G, CV_RGB2GRAY);

		t1 = (double)cv::getTickCount();
		sedlines.runSimplified_EDLines(Img_G);
		//sedlines.drawLineSegments4i(Img_G);
		sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
		sedlines.getLineGradients(Img_G, sedlines.LineGradients);
		c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
		t2 = (double)cv::getTickCount();
		cout << "C1-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		c1_ftd.drawCrossFeatures(Img_G, true);
		c1_ftd.drawC1_FTD(Img_G, true);
		//		sedlines.drawLineSegments4i(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(0);

	}
	system("pause");
	return 0;
}



int main_4_1()
{
	string str_path;
	str_path = ""; //图片路径，注意参数设置
	Mat ImgG = cv::imread(str_path, 0);


	Simplified_EDLines sedlines(540, 960);
	C1_FTD c1_ftd;
	c1_ftd.setFixedParams(15, CV_PI / 6);
	c1_ftd.setAdaptParams(60);
	sedlines.runSimplified_EDLines(ImgG);
	sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
	sedlines.getLineGradients(ImgG, sedlines.LineGradients);
	c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
	c1_ftd.drawCrossFeatures(ImgG, true);
	c1_ftd.drawC1_FTD(ImgG, true);
	cv::waitKey();

	return 0;
}



int main_4_2()
{
	string str_path;
	str_path = ""; //图片路径，注意参数设置
	Mat ImgG = cv::imread(str_path, 0);


	Simplified_EDLines sedlines(540, 960);
	sedlines.runSimplified_EDLines(ImgG);
	sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
	sedlines.getLineGradients(ImgG, sedlines.LineGradients);


	std::vector<cv::Vec4d> out_correctedLines;
	C1_FTD::CorrectLineSegments(sedlines.FittedLineSegments, out_correctedLines);
	
	std::vector<CrossFeature> out_crossfea;
	C1_FTD::CrossFeatureExtraction(out_correctedLines, sedlines.LineGradients, out_crossfea, CV_PI / 6, 15);
	
	std::vector< std::vector<int> > out_adjacency;
	C1_FTD::ConstructAdjacency(out_crossfea, out_adjacency, CV_PI / 6, 15);

	std::vector< cv::Vec4i > out_rects;
	C1_FTD::RectangleSearch(out_adjacency, out_rects);
}