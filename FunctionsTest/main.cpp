#include <iostream>
#include <opencv2\opencv.hpp>

#include <RVIBE.h>
#include <Simplified_EDLines.h>
#include <EllipseDetectorYaed.h>

#include <C1_FTD.h>
#include <C2_FTD.h>
#include <omp.h>

#include <fstream>


int main_1(); // 使用一个视频测试SEDLines
int main_2(); // 使用一个视频测试RVIBE
int main_3(); // 使用一个视频测试YAED

int main_4(); // 使用一个视频：直线段检测 -> C1-FTD
int main_5(); // 使用一个图片：直线段检测 -> C1-FTD

int main_6(); // 使用一个视频：椭圆检测 -> C2-FTD


int main_7(); // 使用一个视频：(并行)直线段检测 -> C1-FTD


int main_8(); // 使用OpenMP：将图像分出多个ROI，然后采用SEDLines->C1-FTD

int main_9(); // 使用数据集计算对应结果

int main_9(
	double lambda_line,              // 直线段阈值
	double angle_error,              // 
	int result_id,
	std::string dataset_type,
	std::string dataset_path,
	std::string dataset_height,
	std::string dataset_group,
	int dataset_num); // 用于参数讨论或批量做数据集
int main_10(); //针对于9进行批量计算

int main_11(int video_num = 1); // 运动目标检测+椭圆检测+直线段检测+C2-FTD

int main_12(int video_num = 1); // 测试跟踪性能



int main()
{
	//return main_9(); UAV 程序备份\UAV Dataset
	return main_12(1);
	//for (int i = 1; i <= 3; i++)
	//{
	//	cout << "正在处理第" << i << "个视频..." << endl;
	//	main_12(i);
	//}

	//return main_11();
	//return main_9(15, CV_PI / 6, 1, "Simulation", "E:\\UAV 程序备份\\UAV Dataset\\", "lower than 5m", "Group3", 31);

//	std::cout << "Test" << std::endl;
//	system("pause");
	return 0;
}


int main_1()
{
	string str_path;
	str_path = "E:\\无人机比赛论文\\视频数据\\真实-FLETD-2.mp4";
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
		cv::imwrite("Ori.png", Img_C);
		t1 = (double)cv::getTickCount();
		sedlines.runSimplified_EDLines(Img_G);
		t2 = (double)cv::getTickCount();
		cout << "SEDLines:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;

		sedlines.drawLineSegments4i(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey();

	}
}

int main_2()
{
	string str_path;
	//str_path = "E:\\无人机比赛论文\\视频数据\\模拟-FLELD运动检测-1.mp4";
	str_path = "D:\\UAV\\视频数据\\模拟-FLELD运动检测-1.mp4";

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
	int rvibeNumber = 0;
	CFG_RVIBE cfg("RVIBE");
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
		cv::waitKey(1);
		cout << loop << endl;
	}


	system("pause");
	return 0;
}

int main_3()
{
	string str_path;
	str_path = "E:\\无人机比赛论文\\视频数据\\真实-FLETD-2.mp4";
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
		vector<Ellipse> ellsYaed;
		yaed.Detect(cv::Mat1b(Img_G), ellsYaed);
		t2 = (double)cv::getTickCount();
		cout << "RVIBE:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;

		yaed.DrawDetectedEllipses(cv::Mat3b(Img_C), ellsYaed);

		loop++;
		imshow("RES", Img_C);
		cv::waitKey(1);

	}


	system("pause");
	return 0;
}


int main_4()
{
	string str_path;
	str_path = "E:\\无人机比赛论文\\视频数据\\真实-FCTD-3.mp4";
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
		//		if (loop == 52)
		//		{
		////			cv::imwrite("Ori.png", Img_G);
		//			cout << "Error" << endl;
		//			return 0;
		//		}
				//
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

int main_5()
{
	string str_path;
	str_path = "Ori.png";
	Mat ImgG = cv::imread(str_path, 0);


	Simplified_EDLines sedlines(540, 960);
	C1_FTD c1_ftd;

	sedlines.runSimplified_EDLines(ImgG);
	sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
	sedlines.getLineGradients(ImgG, sedlines.LineGradients);
	c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
	c1_ftd.drawCrossFeatures(ImgG, true);
	c1_ftd.drawC1_FTD(ImgG, true);
	cv::waitKey();

	return 0;
}



int main_6()
{
	string str_path;
	str_path = "E:\\无人机比赛论文\\视频数据\\真实-FLETD-3.mp4";
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
		vector<Ellipse> ellsYaed;
		vector<cv::RotatedRect> elps;
		yaed.Detect(cv::Mat1b(Img_G), ellsYaed);
		CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);
		//CEllipseDetectorYaed::drawTransEllipses(elps, Img_G);
		//if (loop == 62)
		//{
		//	std::cout << "test" << std::endl;
		//}
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
		cv::waitKey(1);

	}


	system("pause");
	return 0;
}

int main_7()
{
	string str_path;
	str_path = "E:\\无人机比赛论文\\视频数据\\真实-FCTD-3.mp4";
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
		resize(Img_C, Img_C, cv::Size(960, 540));
		srows = Img_C.rows, scols = Img_C.cols;
		cvtColor(Img_C, Img_G, CV_RGB2GRAY);
		//		if (loop == 52)
		//		{
		////			cv::imwrite("Ori.png", Img_G);
		//			cout << "Error" << endl;
		//			return 0;
		//		}
		//
		t1 = (double)cv::getTickCount();
		sedlines.runSimplified_EDLines(Img_G);
		//sedlines.drawLineSegments4i(Img_G);
		sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
		sedlines.getLineGradients(Img_G, sedlines.LineGradients);
		c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
		t2 = (double)cv::getTickCount();
		cout << "C1-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		c1_ftd.drawCrossFeatures(Img_G);
		c1_ftd.drawC1_FTD(Img_G);
		//		sedlines.drawLineSegments4i(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(1);

	}
	system("pause");
	return 0;
}


int main_8()
{
	Simplified_EDLines *_multi_sedlines = NULL;
	C1_FTD *_multi_c1_ftd = NULL;
	int core_num = omp_get_max_threads();


	_multi_sedlines = new Simplified_EDLines[core_num];
	_multi_c1_ftd = new C1_FTD[core_num];


	for (int i = 0; i < core_num; i++)
	{
		_multi_sedlines[i].create_EDLines(540, 960);
	}

	int roi_cols = 240, roi_rows = 135, h_step = 15;
	vector < cv::Rect> rois;

	cv::Mat ImgG = cv::imread("pic_112858.bmp", 0);
	int rows = ImgG.rows, cols = ImgG.cols;
	for (int i = 0; i < rows;)
	{
		for (int j = 0; j < cols;)
		{

		}
	}




	delete[] _multi_sedlines;
	delete[] _multi_c1_ftd;

	system("pause");
	return 0;
}



int main_9()
{
	string datapath = "E:\\UAV Dataset\\Simulation\\lower than 5m\\Group1\\";
	std::ofstream outfile(datapath + "C1-FTD-Res.txt");
	int startNum = 1;
	int endNum = 32;
	char a[128];
	Simplified_EDLines sedlines(540, 960);
	C1_FTD c1_ftd;
	c1_ftd.setFixedParams(15, CV_PI / 6);
	c1_ftd.setAdaptParams(30);

	Mat ImgC, ImgG;
	double t1, t2, t3;
	for (int i = startNum; i <= endNum; i++)
	{
		_itoa_s(i, a, 10);
		ImgC = cv::imread(datapath + string(a) + ".png");
		resize(ImgC, ImgC, cv::Size(480, 270));
		imshow("Ori", ImgC);
		cvtColor(ImgC, ImgG, CV_RGB2GRAY);
		t1 = cv::getTickCount();
		sedlines.runSimplified_EDLines(ImgG);
		t2 = cv::getTickCount();
		sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
		sedlines.getLineGradients(ImgG, sedlines.LineGradients);
		c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
		t3 = cv::getTickCount();

		c1_ftd.drawCrossFeatures(ImgG, true);
		c1_ftd.drawC1_FTD(ImgG, true);
		if (c1_ftd.isFind)
		{
			outfile << c1_ftd.cross4Points[0].x << '\t' << c1_ftd.cross4Points[0].y << '\t'
				<< c1_ftd.cross4Points[1].x << '\t' << c1_ftd.cross4Points[1].y << '\t'
				<< c1_ftd.cross4Points[2].x << '\t' << c1_ftd.cross4Points[2].y << '\t'
				<< c1_ftd.cross4Points[3].x << '\t' << c1_ftd.cross4Points[3].y << '\t'
				<< c1_ftd.crossCenter.x << '\t' << c1_ftd.crossCenter.y << '\t';
		}
		else
		{
			outfile << 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t';
		}
		outfile << (t2 - t1) * 1000 / cv::getTickFrequency() << '\t';
		outfile << (t3 - t2) * 1000 / cv::getTickFrequency() << endl;
		cv::waitKey(1);
	}

	outfile.close();
	system("pause");
	return 0;
}

int main_9(
	double lambda_line,              // 直线段阈值
	double angle_error,              // 
	int result_id,
	std::string dataset_type,
	std::string dataset_path,
	std::string dataset_height,
	std::string dataset_group,
	int dataset_num) // 用于参数讨论或批量做数据集
{
	string datapath = dataset_path + dataset_type + "\\" + dataset_height + "\\" + dataset_group + "\\";
	char a[128];
	_itoa_s(result_id, a, 10);
	std::ofstream outfile(datapath + "Results\\C1-FTD-Res-" + string(a) + ".txt");
	int startNum = 1;
	int endNum = dataset_num;

	Simplified_EDLines sedlines(540, 960);
	C1_FTD c1_ftd;
	c1_ftd.setFixedParams(lambda_line, angle_error);
	c1_ftd.setAdaptParams(30);

	Mat ImgC, ImgG;
	double t1, t2, t3;
	for (int i = startNum; i <= endNum; i++)
	{
		_itoa_s(i, a, 10);
		ImgC = cv::imread(datapath + string(a) + ".png");
		resize(ImgC, ImgC, cv::Size(480, 270));
		//imshow("Ori", ImgC);
		cvtColor(ImgC, ImgG, CV_RGB2GRAY);
		t1 = cv::getTickCount();
		sedlines.runSimplified_EDLines(ImgG);
		t2 = cv::getTickCount();
		sedlines.getFittedLineSegments(sedlines.FittedLineSegments);
		sedlines.getLineGradients(ImgG, sedlines.LineGradients);
		c1_ftd.runC1_FTD(sedlines.FittedLineSegments, sedlines.LineGradients);
		t3 = cv::getTickCount();

		//c1_ftd.drawCrossFeatures(ImgG, true);
		//c1_ftd.drawC1_FTD(ImgG, true);
		if (c1_ftd.isFind)
		{
			outfile << c1_ftd.cross4Points[0].x << '\t' << c1_ftd.cross4Points[0].y << '\t'
				<< c1_ftd.cross4Points[1].x << '\t' << c1_ftd.cross4Points[1].y << '\t'
				<< c1_ftd.cross4Points[2].x << '\t' << c1_ftd.cross4Points[2].y << '\t'
				<< c1_ftd.cross4Points[3].x << '\t' << c1_ftd.cross4Points[3].y << '\t'
				<< c1_ftd.crossCenter.x << '\t' << c1_ftd.crossCenter.y << '\t';
		}
		else
		{
			outfile << 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t'
				<< 0 << '\t' << 0 << '\t';
		}
		outfile << (t2 - t1) * 1000 / cv::getTickFrequency() << '\t';
		outfile << (t3 - t2) * 1000 / cv::getTickFrequency() << endl;
		cv::waitKey(1);
	}

	outfile.close();
	//system("pause");
	return 0;
}

int main_10()
{
	double data_lambda_line[15];
	double data_angle_error[9];
	for (int i = 0; i < 15; i++)
		data_lambda_line[i] = i + 1;
	for (int i = 0; i < 9; i++)
		data_angle_error[i] = (i + 1) * 5;

	int group_num = 5;
	//int groups_imagenum[20] = { 32,26,31,47,204,32,38,26,26,14,6,12,17,21,12,12,15,36,22,23 };
	int groups_imagenum[5] = { 9,36,31,19,36 };
	std::string dataset_path = "D:\\UAV Dataset\\";
	std::string dataset_type = "MBZIRC";
	std::string dataset_height = "lower than 5m";

	char str2nums[256];


	for (int group_id = 1; group_id <= group_num; group_id++)
	{
		std::cout << "正在处理第" << group_id << "组..." << std::endl;
		_itoa_s(group_id, str2nums, 10);
		int file_id = 1;
		for (int angle_idx = 0; angle_idx < 9; angle_idx++)
		{
			main_9(15, data_angle_error[angle_idx] * 1.00 / 180 * CV_PI, file_id++, dataset_type, dataset_path, dataset_height, "Group" + string(str2nums), groups_imagenum[group_id - 1]);
		}
		for (int lambda_idx = 0; lambda_idx < 15; lambda_idx++)
		{
			main_9(data_lambda_line[lambda_idx], CV_PI / 6, file_id++, dataset_type, dataset_path, dataset_height, "Group" + string(str2nums), groups_imagenum[group_id - 1]);
		}

	}



	main_9(15, CV_PI / 6, 1, "Simulation", "E:\\UAV 程序备份\\UAV Dataset\\", "lower than 5m", "Group3", 31);




	return 0;






}




int main_11(int video_num)
{

	char temp[256];
	_itoa_s(video_num, temp, 10);
	string str_path;
	string video_name = "模拟-FLELD运动检测-" + string(temp);
	string vidio_path = "D:\\UAV\\视频数据\\";
	str_path = vidio_path + video_name + ".mp4";

	std::ofstream outfile(vidio_path + video_name + "-res.txt");

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
	Mat Img_C, Img_G, Img_T;

	CFG_RVIBE cfg("RVIBE");
	RVIBE rvibe(270, 480, cfg);
	CEllipseDetectorYaed yaed;
	Simplified_EDLines sedlines(540, 960);
	C2_FTD c2_ftd(540, 960);



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

	double pipeline_st, pipeline_ed;
	double t1, t2;


	cv::RotatedRect landEllipse; //目标椭圆
	cv::Vec4f landCross;         //目标十字信息
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

		double time_rvibe(0), time_yaed(0), time_sedlines(0), time_c2_ftd(0);
		// 运动目标检测
		pipeline_st = cv::getTickCount();
		t1 = cv::getTickCount();
		resize(Img_G, Img_T, cv::Size(480, 270));
		rvibe.run_RVIBE_tbb(Img_T);
		t2 = cv::getTickCount();
		time_rvibe = (t2 - t1) * 1000 / cv::getTickFrequency();
		// 椭圆+直线段检测，并采用C2-FTD检测
		const int rect_num = rvibe.findRects.size();
		for (int j = 0; j < rect_num; j++)
		{
			// 获取ROI
			t1 = cv::getTickCount();
			cv::Rect imgROI = rvibe.findRects[j];
			imgROI.x = imgROI.x * 2, imgROI.y = imgROI.y * 2;
			imgROI.width = imgROI.width * 2, imgROI.height = imgROI.height * 2;

			Img_T = Img_G(imgROI).clone();
			cv::Size sz = Img_T.size();
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
			vector<Ellipse> ellsYaed;
			vector<cv::RotatedRect> elps;
			yaed.Detect(cv::Mat1b(Img_T), ellsYaed);
			t2 = cv::getTickCount();
			time_yaed += (t2 - t1) * 1000 / cv::getTickFrequency();
			//if (ellsYaed.size() == 0)
			//	continue;

			t1 = cv::getTickCount();
			CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);
			sedlines.runSimplified_EDLines(Img_T);
			t2 = cv::getTickCount();
			time_sedlines += (t2 - t1) * 1000 / cv::getTickFrequency();

			t1 = cv::getTickCount();
			c2_ftd.runC2_FTD(elps, sedlines.LineSegments, Img_T.rows, Img_T.cols);
			t2 = cv::getTickCount();
			time_c2_ftd += (t2 - t1) * 1000 / cv::getTickFrequency();
			if (c2_ftd.isFind)
			{
				//landEllipse = c2_ftd.landEllipse;
				//landEllipse.center.y += imgROI.x;
				//landEllipse.center.x += imgROI.y;
				//landCross = c2_ftd.landCross;
				c2_ftd.landEllipse.center.x += imgROI.y;
				c2_ftd.landEllipse.center.y += imgROI.x;
				//c2_ftd.drawC2_FTD(Img_G);
				break;
			}
			else
			{
				//cv::imshow("C2-FTD",Img_G);
			}

		}
		pipeline_ed = cv::getTickCount();

		//cout << "C2-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		//sedlines.drawLineSegments4i(Img_G);
		//c2_ftd.drawInsideLines(Img_G);
		//yaed.DrawDetectedEllipses(cv::Mat3b(Img_C), ellsYaed);
		//c2_ftd.drawC2_FTD(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(1);
		//cout << "RVBIE:" << time_rvibe << ", YAED:" << time_yaed << ", SEDLines:" << time_sedlines << ", C2-FTD:" << time_c2_ftd << ", Pipeline:" << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;

		if (c2_ftd.isFind)
		{
			outfile << c2_ftd.landEllipse.center.x << '\t' << c2_ftd.landEllipse.center.y << '\t' << time_rvibe << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
		}
		else
		{
			outfile << 0 << '\t' << 0 << '\t' << time_rvibe << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
		}

	}

	outfile.close();

	//system("pause");
	return 0;
}


int main_12(int video_num)
{
	char temp[256];
	_itoa_s(video_num, temp, 10);
	string str_path;
	string video_name = "真实-FLETD-" + string(temp);
	string vidio_path = "D:\\UAV\\视频数据\\";
	str_path = vidio_path + video_name + ".mp4";

	std::ofstream outfile(vidio_path + video_name + "-res-tracking.txt");

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
	Mat Img_C, Img_G, Img_T;

	CFG_RVIBE cfg("RVIBE");
	RVIBE rvibe(270, 480, cfg);
	CEllipseDetectorYaed yaed;
	Simplified_EDLines sedlines(540, 960);
	C2_FTD c2_ftd(540, 960);



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

	double pipeline_st, pipeline_ed;
	double t1, t2;


	cv::RotatedRect landEllipse; //目标椭圆
	cv::Vec4f landCross;         //目标十字信息

	bool isTracking = false;
	cv::Rect trackROI;

	while (1)
	{
		//printf("loop: %d", loop);
		cap >> Img_C;
		if (Img_C.empty())
			break;
		orows = Img_C.rows, ocols = Img_C.cols;
		resize(Img_C, Img_C, cv::Size(960, 540));
		srows = Img_C.rows, scols = Img_C.cols;
		cvtColor(Img_C, Img_G, CV_RGB2GRAY);

		double time_yaed(0), time_sedlines(0), time_c2_ftd(0);

		pipeline_st = cv::getTickCount();
		if (isTracking) //如果当前处在跟踪状态
		{
			// 获取当前需要的ROI
			cv::Rect newROI;
			cv::Point leftUp, rightDown;
			leftUp.x = trackROI.x - 0.25*trackROI.width;
			leftUp.y = trackROI.y - 0.25*trackROI.height;
			rightDown.x = leftUp.x + 1.5*trackROI.width;
			rightDown.y = leftUp.y + 1.5*trackROI.height;

			if (leftUp.x < 0) leftUp.x = 0;
			if (leftUp.x >= Img_G.cols) leftUp.x = Img_G.cols - 1;
			if (leftUp.y < 0) leftUp.y = 0;
			if (leftUp.y >= Img_G.rows) leftUp.y = Img_G.rows - 1;

			if (rightDown.x < 0) rightDown.x = 0;
			if (rightDown.x >= Img_G.cols) rightDown.x = Img_G.cols - 1;
			if (rightDown.y < 0) rightDown.y = 0;
			if (rightDown.y >= Img_G.rows) rightDown.y = Img_G.rows - 1;

			newROI.x = leftUp.x, newROI.y = leftUp.y;
			newROI.width = rightDown.x - leftUp.x;
			newROI.height = rightDown.y - leftUp.y;
			cv::rectangle(Img_C, newROI, cv::Scalar(255, 0, 0), 2);
			imshow("ROI", Img_C);
			t1 = cv::getTickCount();
			Img_T = Img_G(newROI).clone();
			cv::Size sz = Img_T.size();
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
			vector<Ellipse> ellsYaed;
			vector<cv::RotatedRect> elps;
			yaed.Detect(cv::Mat1b(Img_T), ellsYaed);
			t2 = cv::getTickCount();
			time_yaed += (t2 - t1) * 1000 / cv::getTickFrequency();
			//if (ellsYaed.size() == 0)
			//	continue;

			t1 = cv::getTickCount();
			CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);
			sedlines.runSimplified_EDLines(Img_T);
			t2 = cv::getTickCount();
			time_sedlines += (t2 - t1) * 1000 / cv::getTickFrequency();

			t1 = cv::getTickCount();
			c2_ftd.runC2_FTD(elps, sedlines.LineSegments, Img_T.rows, Img_T.cols);
			t2 = cv::getTickCount();
			time_c2_ftd += (t2 - t1) * 1000 / cv::getTickFrequency();

			// 获取新的ROi
			if (c2_ftd.isFind)
			{

				trackROI.x = c2_ftd.landEllipse.center.y - c2_ftd.landEllipse.size.width / 2 + newROI.x;
				trackROI.y = c2_ftd.landEllipse.center.x - c2_ftd.landEllipse.size.width / 2 + newROI.y;
				trackROI.width = c2_ftd.landEllipse.size.width;
				trackROI.height = c2_ftd.landEllipse.size.width;

				c2_ftd.landEllipse.center.x += newROI.y;
				c2_ftd.landEllipse.center.y += newROI.x;

			}
			else
			{
				trackROI = newROI;
			}
			
			pipeline_ed = cv::getTickCount();

			c2_ftd.drawC2_FTD(Img_G);

			if (c2_ftd.isFind)
			{
				outfile << loop << '\t' << c2_ftd.landEllipse.center.x << '\t' << c2_ftd.landEllipse.center.y << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
			}
			else
			{
				outfile << loop << '\t' << 0 << '\t' << 0 << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
			}

		}
		else // 处在检测ROI状态
		{
			resize(Img_G, Img_T, cv::Size(480, 270));
			rvibe.run_RVIBE_tbb(Img_T);
			const int rect_num = rvibe.findRects.size();
			for (int j = 0; j < rect_num; j++)
			{
				// 获取ROI
				cv::Rect imgROI = rvibe.findRects[j];
				imgROI.x = imgROI.x * 2, imgROI.y = imgROI.y * 2;
				imgROI.width = imgROI.width * 2, imgROI.height = imgROI.height * 2;

				Img_T = Img_G(imgROI).clone();
				cv::Size sz = Img_T.size();
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
				vector<Ellipse> ellsYaed;
				vector<cv::RotatedRect> elps;
				yaed.Detect(cv::Mat1b(Img_T), ellsYaed);
				//if (ellsYaed.size() == 0)
				//	continue;

				CEllipseDetectorYaed::YAED_Ellipse_2_CV(ellsYaed, elps);
				sedlines.runSimplified_EDLines(Img_T);

				c2_ftd.runC2_FTD(elps, sedlines.LineSegments, Img_T.rows, Img_T.cols);
				if (c2_ftd.isFind)
				{
					c2_ftd.landEllipse.center.x += imgROI.y;
					c2_ftd.landEllipse.center.y += imgROI.x;
					c2_ftd.drawC2_FTD(Img_G);

					trackROI.x = c2_ftd.landEllipse.center.y - c2_ftd.landEllipse.size.width / 2;
					trackROI.y = c2_ftd.landEllipse.center.x - c2_ftd.landEllipse.size.width / 2;
					trackROI.width = c2_ftd.landEllipse.size.width;
					trackROI.height = c2_ftd.landEllipse.size.width;
					isTracking = true;
					c2_ftd.drawC2_FTD(Img_G);
					break;
				}
				else
				{
					cv::imshow("C2-FTD",Img_G);
				}

			}
		}

		// 椭圆+直线段检测，并采用C2-FTD检测


		//cout << "C2-FTD:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		//sedlines.drawLineSegments4i(Img_G);
		//c2_ftd.drawInsideLines(Img_G);
		//yaed.DrawDetectedEllipses(cv::Mat3b(Img_C), ellsYaed);
		//c2_ftd.drawC2_FTD(Img_G);
		loop++;
		//imshow("RES", Img_C);
		cv::waitKey(1);
		//cout << "RVBIE:" << time_rvibe << ", YAED:" << time_yaed << ", SEDLines:" << time_sedlines << ", C2-FTD:" << time_c2_ftd << ", Pipeline:" << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;

		//if (c2_ftd.isFind)
		//{
		//	outfile << c2_ftd.landEllipse.center.x << '\t' << c2_ftd.landEllipse.center.y << '\t' << time_rvibe << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
		//}
		//else
		//{
		//	outfile << 0 << '\t' << 0 << '\t' << time_rvibe << '\t' << time_yaed << '\t' << time_sedlines << '\t' << time_c2_ftd << '\t' << (pipeline_ed - pipeline_st) * 1000 / cv::getTickFrequency() << endl;
		//}

	}

	outfile.close();

	//system("pause");
	return 0;
}