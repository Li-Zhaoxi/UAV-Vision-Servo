#include "C2_FTD.h"

C2_FTD::C2_FTD()
{
	isFind = false;
	_detLines_data = NULL;

}

void C2_FTD::create(int rows, int cols)
{
	drows = rows, dcols = cols;
	lineMap = cv::Mat::zeros(rows, cols, CV_16UC1);
	isFind = false;
	_detLines_data = NULL;
}
C2_FTD::C2_FTD(int rows, int cols)
{
	create(rows, cols);
}

void C2_FTD::runC2_FTD(std::vector<cv::RotatedRect> &detEllipses, std::vector<cv::Vec4i> &detLineSegments, int rows, int cols)
{
//  结果初始化
	isFind = false;
	landEllipse.center.x = landEllipse.center.y = 0;
	landEllipse.size.width = landEllipse.size.height = 1;

	line_in_ellipse.clear();
	if (detEllipses.size() == 0)
		return;

	irows = rows, icols = cols;
	unsigned short *_lineMap = (unsigned short*)lineMap.data;
	_detLines_data = detLineSegments.data();

	getLineMap(_lineMap, detLineSegments);    // 设置直线地图

	ELPShape2Equation(detEllipses, ellipse_parms);
	getInsideLines(_lineMap, detEllipses, line_in_ellipse);

	SlopeClustering(line_in_ellipse, detEllipses);
	SelectFinalCross(line_in_ellipse, detEllipses);

}


void C2_FTD::getLineMap(unsigned short *_lineMap, std::vector< cv::Vec4i > &cdtLines)
{
	memset((void*)_lineMap, 0, sizeof(unsigned short)*drows*dcols);

	const int lineNum = (int)cdtLines.size();
	for (int i = 0; i < lineNum; i++)
		_lineMap[cdtLines[i][0] * icols + cdtLines[i][1]] = i + 1;
}

void C2_FTD::getInsideLines(const ushort *_lineMap, const std::vector<cv::RotatedRect> &cdtEllipse, std::vector<LineINEllipse> &LIE)
{
	cv::Point loc[4];
	unsigned short val;

	const int elpNum = (int)cdtEllipse.size();

	LIE.resize(elpNum);
	for (int i = 0; i < elpNum; i++)
	{
		// Get search range.
		loc[0].x = int(cdtEllipse[i].center.x - cdtEllipse[i].size.width / 2 + 0.5), loc[0].y = int(cdtEllipse[i].center.y - cdtEllipse[i].size.width / 2 + 0.5);
		loc[1].x = int(cdtEllipse[i].center.x - cdtEllipse[i].size.width / 2 + 0.5), loc[1].y = int(cdtEllipse[i].center.y + cdtEllipse[i].size.width / 2 + 0.5);
		loc[2].x = int(cdtEllipse[i].center.x + cdtEllipse[i].size.width / 2 + 0.5), loc[2].y = int(cdtEllipse[i].center.y + cdtEllipse[i].size.width / 2 + 0.5);
		loc[3].x = int(cdtEllipse[i].center.x + cdtEllipse[i].size.width / 2 + 0.5), loc[3].y = int(cdtEllipse[i].center.y - cdtEllipse[i].size.width / 2 + 0.5);
		for (int i = 0; i < 4; i++)
		{
			if (loc[i].x < 0) loc[i].x = 0;
			if (loc[i].x >= irows) loc[i].x = irows - 1;
			if (loc[i].y < 0) loc[i].y = 0;
			if (loc[i].y >= icols) loc[i].y = icols - 1;
		}

		// Find Lines;
		LIE[i].idxEllipse = i;//设置椭圆角标
		LIE[i].line_data.clear(); // 清空直线段数据

		for (int iMap = loc[0].x; iMap <= loc[3].x; iMap++)
		{
			for (int jMap = loc[0].y; jMap <= loc[1].y; jMap++)
			{
				val = _lineMap[iMap*icols + jMap]; //获得直线段对应的index
				if (val == 0) //此处没有直线
					continue;
				if (!isInEllipse(iMap, jMap, i))//直线的一个端点不在椭圆内部
					continue;
				val--; //获得直线段idx
				if (!isInEllipse(_detLines_data[val][2], _detLines_data[val][3], i)) // 直线的另一个端点不在椭圆内部
					continue;
				if (!CONSTRAINT_DIST(_detLines_data[val], cdtEllipse[i])) //如果这个直线满足距离约束
					continue;
				LIE[i].line_data.push_back(_detLines_data[val]);
			}
		}

	}
}


void C2_FTD::SlopeClustering(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detElps)
{
	const int all_num = (int)LIE.size(); //存储满足条件的椭圆个数

	double cluster_err, center_err, angleCluse[2], dx, dy;
	std::vector<double> slopes;
	std::vector<int> best_labels;
	
	double fitMat[2][6];

	for (int i = 0; i < all_num; i++)
	{
		const int line_num = (int)LIE[i].line_data.size();
		if (line_num <= 4) //小于4条直线缺失严重，扔掉
		{
			LIE[i].lineScore = 100;
			continue;
		}

		// 直线段 斜率 聚类
		slopes.resize(line_num);
		cv::Vec4i *_line_data = LIE[i].line_data.data();
		for (int j = 0; j < line_num; j++)
		{
			dx = _line_data[j][2] - _line_data[j][0];
			dy = _line_data[j][3] - _line_data[j][1];
			if (dx < 0) { dx = -dx, dy = -dy; }
			slopes[j] = atan2(dy, dx);
		}

		cluster_err = twoL_means(slopes, best_labels, angleCluse[0], angleCluse[1]) / CV_PI * 180;
		// 计算聚类后两类直线段的交点
		for (int j = 0; j < 6; j++)
			fitMat[0][j] = 0, fitMat[1][j] = 0;
		LIE[i].cross4Direct[0] = cos(angleCluse[0]), LIE[i].cross4Direct[1] = sin(angleCluse[0]);
		LIE[i].cross4Direct[2] = cos(angleCluse[1]), LIE[i].cross4Direct[3] = sin(angleCluse[1]);

		LIE[i].lineScore = cluster_err;
		
	}

}


void C2_FTD::ELPShape2Equation(const std::vector<cv::RotatedRect> &in_elps, std::vector<cv::Vec6d> &out_parms)
{
	const int elp_num = (int)in_elps.size();

	double xc, yc, a, b, theta, sqr_a, sqr_b;
	double cos_theta, sin_theta ,sqr_cos_theta, sqr_sin_theta;
	double *parm_temp(NULL);

	out_parms.resize(elp_num);
	for (int i = 0; i < elp_num; i++)
	{
		xc = in_elps[i].center.x;
		yc = in_elps[i].center.y;
		a = in_elps[i].size.width;
		b = in_elps[i].size.height;
		theta = in_elps[i].angle / 180 * CV_PI;

		cos_theta = cos(theta), sin_theta = sin(theta);
		sqr_cos_theta = cos_theta*cos_theta, sqr_sin_theta = sin_theta*sin_theta;

		sqr_a = a*a, sqr_b = b*b;

		parm_temp = out_parms[i].val;
		parm_temp[0] = sqr_cos_theta / sqr_a + sqr_sin_theta / sqr_b;
		parm_temp[1] = -(sin(2 * theta)*(sqr_a - sqr_b)) / (2 * sqr_a*sqr_b);
		parm_temp[2] = sqr_cos_theta / sqr_b + sqr_sin_theta / sqr_a;
		parm_temp[3] = (-sqr_a*xc*sqr_sin_theta + (sqr_a*yc*sin(2 * theta)) / 2) / (sqr_a*sqr_b) - (xc*sqr_cos_theta + (yc*sin(2 * theta)) / 2) / sqr_a;
		parm_temp[4] = (-sqr_a*yc*sqr_cos_theta + (sqr_a*xc*sin(2 * theta)) / 2) / (sqr_a*sqr_b) - (yc*sqr_sin_theta + (xc*sin(2 * theta)) / 2) / sqr_a;
		parm_temp[5] = (xc*cos_theta + yc*sin_theta)*(xc*cos_theta + yc*sin_theta) / sqr_a + (yc*cos_theta - xc*sin_theta)*(yc*cos_theta - xc*sin_theta) / sqr_b - 1;

	}
}


void C2_FTD::SelectFinalCross(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detEllipses)
{
	const int LIE_num = (int)LIE.size();
	int idx_min = 0;
	float val_min = LIE[0].lineScore;
	for (int i = 1; i < LIE_num; i++)
	{
		if (LIE[i].lineScore < val_min)
			val_min = LIE[i].lineScore, idx_min = i;
	}

	if (val_min < 50)
	{
		isFind = true;
		landEllipse = detEllipses[idx_min];
		landCross[0] = LIE[idx_min].cross4Direct[0];
		landCross[1] = LIE[idx_min].cross4Direct[1];
		landCross[2] = LIE[idx_min].cross4Direct[2];
		landCross[3] = LIE[idx_min].cross4Direct[3];
	}
}



void C2_FTD::drawC2_FTD(cv::Mat &ImgG)
{

	cv::Mat ImgT;
	cvtColor(ImgG, ImgT, CV_GRAY2BGR);
	if (isFind)
	{
		cv::RotatedRect temp;
		temp.center.x = landEllipse.center.y;
		temp.center.y = landEllipse.center.x;
		temp.size.height = landEllipse.size.width;
		temp.size.width = landEllipse.size.height;
		temp.angle = -landEllipse.angle;
		ellipse(ImgT, temp, cv::Scalar(255, 0, 255), 2);

		cv::Point st, ed;
		int l = landEllipse.size.height / 4 * 3;
		st.y = -l*landCross[0] + temp.center.y; st.x = -l*landCross[1] + temp.center.x;
		ed.y = l*landCross[0] + temp.center.y; ed.x = l*landCross[1] + temp.center.x;
		line(ImgT, st, ed, cv::Scalar(0, 0, 255), 2);

		st.y = -l*landCross[2] + temp.center.y; st.x = -l*landCross[3] + temp.center.x;
		ed.y = l*landCross[2] + temp.center.y; ed.x = l*landCross[3] + temp.center.x;
		line(ImgT, st, ed, cv::Scalar(0, 0, 255), 2);
	}
	

	cv::imshow("C2-FTD", ImgT);
	cv::waitKey(1);

}