#include "Simplified_EDLines.h"

void Simplified_EDLines::create_EDLines(int drows, int dcols)
{
	data = new LineNode[drows*dcols];
	this->dcols = dcols, this->drows = drows;

	for (int i = 0; i < drows; i++)
	{
		for (int j = 0; j < dcols; j++)
		{
			data[i*dcols + j] = LineNode(i, j, 100);
		}
	}

	// EDLines Params
	min_line_length = 8;
	lineFitErrThreshold = 1.4f;
}
Simplified_EDLines::~Simplified_EDLines()
{
	if (data != NULL)
	{
		delete[] data;
		data = NULL;
	}
}

void Simplified_EDLines::release()
{
	if (data != NULL)
	{
		delete[] data;
		data = NULL;
	}
}
Simplified_EDLines::Simplified_EDLines(int drows, int dcols)
{
	create_EDLines(drows, dcols);
}


void Simplified_EDLines::runSimplified_EDLines(cv::Mat &ImgG)
{
	irows = ImgG.rows, icols = ImgG.cols;
	LineSegments.clear();

	int canny_high, canny_low;
	calCannyThreshold(ImgG, canny_low, canny_high);
	cv::Canny(ImgG, imgCanny, canny_low, canny_high);

	unsigned char *_edge = (unsigned char*)imgCanny.data;
	findContours(_edge);
//	LineNode *temp = data + DIDX(edgeContours[0][0].x, edgeContours[0][1].y);

	const int edgeNum = (int)edgeContours.size();
	int dpNum, sqr_length = min_line_length*min_line_length;
	cv::Vec4i linetemp;
	cv::Point2i vecLine;
	for (int i = 0; i < edgeNum; i++)
	{
		approxPolyDP(edgeContours[i], oneContour, lineFitErrThreshold, false);//��ȡһ���ƽ���
		dpNum = (int)oneContour.size();
		
		for (int j = 0; j < dpNum - 1; j++)
		{
			linetemp[0] = oneContour[j].x, linetemp[1] = oneContour[j].y;
			linetemp[2] = oneContour[j + 1].x, linetemp[3] = oneContour[j + 1].y;
			vecLine.x = linetemp[2] - linetemp[0], vecLine.y = linetemp[3] - linetemp[1];
			if (vecLine.x*vecLine.x + vecLine.y*vecLine.y < sqr_length)
				continue;
			LineSegments.push_back(linetemp);
		}
	}

}





void Simplified_EDLines::calCannyThreshold(cv::Mat &ImgG, int &low, int &high)
{
	cv::Mat ImgT, dx, dy, grad;
	cv::resize(ImgG, ImgT, cv::Size(ImgG.cols / 10, ImgG.rows / 10));
	cv::Sobel(ImgT, dx, CV_16SC1, 1, 0);
	cv::Sobel(ImgT, dy, CV_16SC1, 0, 1);
	short *_dx = (short*)dx.data, *_dy = (short*)dy.data;

	int subpixel_num = dx.rows*dx.cols;
	grad.create(1, subpixel_num, CV_32SC1);
	int* _grad = (int*)grad.data;
	int maxGrad(0);
	for (int i = 0; i < subpixel_num; i++)
	{
		_grad[i] = std::abs(_dx[i]) + std::abs(_dy[i]);
		if (maxGrad < _grad[i])
			maxGrad = _grad[i];
	}

	//set magic numbers
	const int NUM_BINS = 64;
	const double percent_of_pixels_not_edges = 0.7;
	const double threshold_ratio = 0.4;
	int bins[NUM_BINS] = { 0 };


	//compute histogram
	int bin_size = int(round(maxGrad / float(NUM_BINS)) + 1);
	if (bin_size < 1) bin_size = 1;
	for (int i = 0; i < subpixel_num; i++)
	{
		bins[_grad[i] / bin_size]++;
	}

	//% Select the thresholds
	float total(0.f);
	float target = float(subpixel_num * percent_of_pixels_not_edges);

	high = 0;
	while (total < target)
	{
		total += bins[high];
		high++;
	}
	high *= bin_size;
	low = int(std::floor(threshold_ratio * float(high)));

}

void Simplified_EDLines::findContours(unsigned char *_edge)
{
	const int clockWise[8][2] = { { 0,1 },{ 1,0 },{ 0,-1 },{ -1,0 },{ -1,1 },{ 1,1 },{ 1,-1 },{ -1,-1 } };
	const int anticlockWise[8][2] = { { 0,-1 },{ 1,0 },{ 0,1 },{ -1,0 },{ -1,-1 },{ 1,-1 },{ 1,1 },{ -1,1 } };
	edgeContours.clear();
	int idx_first = (irows - 1)*icols;


	for (int i = 0; i < icols; i++)
	{
		_edge[i] = 0;
		_edge[idx_first + i] = 0;
	}
	for (int i = 1; i < irows - 1; i++)
	{
		_edge[i*icols] = 0;
		_edge[i*icols + icols - 1] = 0;
	}
	for (int i = 1; i < irows; i++)
	{
		idx_first = i*icols;
		for (int j = 1; j < icols; j++)
		{
			if (_edge[idx_first + j])
			{
				_edge[idx_first + j] = 0;
				if (_edge[idx_first + icols + j - 1] && _edge[idx_first + icols + j] && _edge[idx_first + icols + j + 1])
					continue;
				else
				{
					findContour(clockWise, anticlockWise, _edge, i, j);
				}
			}
		}
	}



}

void Simplified_EDLines::findContour(const int Wise[8][2], const int antiWise[8][2], uchar *Edge, int x, int y)
{
	bool isEnd;
	int find_x, find_y;
	int move_x = x, move_y = y;
	oneContour.clear(); oneContourOpp.clear();
	oneContour.push_back(cv::Point(x, y));
	int idxdMove = DIDX(x, y), idxiMove = IIDX(x, y), idxdFind, idxiFind;
	data[idxdMove].setFirst();
	while (1)
	{
		isEnd = true;
		idxiMove = IIDX(move_x, move_y);
		for (int i = 0; i < 8; i++)
		{
			find_x = move_x + Wise[i][0];
			find_y = move_y + Wise[i][1];
			idxiFind = IIDX(find_x, find_y);
			if (Edge[idxiFind])
			{
				Edge[idxiFind] = 0;
				isEnd = false;
				idxdMove = DIDX(move_x, move_y);
				idxdFind = DIDX(find_x, find_y);
				(data + idxdMove)->nextIs(data + idxdFind);
				move_x = find_x; move_y = find_y;
				oneContour.push_back(cv::Point(move_x, move_y));
				break;
			}
		}
		if (isEnd)
		{
			idxdMove = DIDX(move_x, move_y);
			(data + idxdMove)->nextIs(NULL);
			break;
		}
	}


	move_x = oneContour[0].x; move_y = oneContour[0].y;
	while (1)
	{
		isEnd = true;
		idxiMove = IIDX(move_x, move_y);
		for (int i = 0; i < 8; i++)
		{
			find_x = move_x + antiWise[i][0];
			find_y = move_y + antiWise[i][1];
			idxiFind = IIDX(find_x, find_y);
			if (Edge[idxiFind])
			{
				Edge[idxiFind] = 0;
				isEnd = false;
				idxdMove = DIDX(move_x, move_y);
				idxdFind = DIDX(find_x, find_y);
				(data + idxdMove)->lastIs(data + idxdFind);
				move_x = find_x; move_y = find_y;
				oneContourOpp.push_back(cv::Point(move_x, move_y));
				break;
			}
		}
		if (isEnd)
		{
			idxdMove = DIDX(move_x, move_y);
			(data + idxdMove)->lastIs(NULL);
			break;
		}
	}
	if (oneContour.size() + oneContourOpp.size() > min_line_length)
	{
		if (oneContourOpp.size() > 0)
		{
			cv::Point temp;
			for (int i = 0; i < (oneContourOpp.size() + 1) / 2; i++)
			{
				temp = oneContourOpp[i];
				oneContourOpp[i] = oneContourOpp[oneContourOpp.size() - 1 - i];
				oneContourOpp[oneContourOpp.size() - 1 - i] = temp;
			}
			oneContourOpp.insert(oneContourOpp.end(), oneContour.begin(), oneContour.end());
			edgeContours.push_back(oneContourOpp);
		}
		else
			edgeContours.push_back(oneContour);
	}
	//LineNode *temp = data + DIDX(edgeContours[0][0].x, edgeContours[0][1].y);
	//temp = temp->nextAddress->nextAddress;
	//if (temp->Location.x != 3 && temp->Location.y != 165)
	//	std::cout << "Error" << std::endl;
}


void Simplified_EDLines::drawLineSegments4i(cv::Mat &ImgG, bool isShow)
{
	if (!isShow)
	{
		cvtColor(ImgG, ImgG, CV_GRAY2BGR);
		for (int i = 0; i < LineSegments.size(); i++)
		{
			cv::Point st, ed;
			st.y = LineSegments[i][0], st.x = LineSegments[i][1];
			ed.y = LineSegments[i][2], ed.x = LineSegments[i][3];
			cv::line(ImgG, st, ed, cv::Scalar(100, 255, 100), 2);

		}
	}
	else
	{
		cv::Mat ImgT= 255- (255-ImgG.clone())/2;
		cvtColor(ImgT, ImgT, CV_GRAY2BGR);
		for (int i = 0; i < LineSegments.size(); i++)
		{
			cv::Point st, ed;
			st.y = LineSegments[i][0], st.x = LineSegments[i][1];
			ed.y = LineSegments[i][2], ed.x = LineSegments[i][3];
			cv::line(ImgT, st, ed, cv::Scalar(255, 0, 0), 2);

		}
		cv::imshow("LineSegments", ImgT);
		cv::imwrite("LineSegments.png", ImgT);
		cv::waitKey(1);
	}
}



void Simplified_EDLines::getLineGradients(cv::Mat &ImgG, std::vector< cv::Point2d> &linegradients) const
{
	const int line_num = (int)LineSegments.size();
	int idxdst, idxded;
	unsigned char *_gray = (unsigned char*)ImgG.data;

	linegradients.resize(line_num);

	for (int i = 0; i < line_num; i++)
	{
		idxdst = DIDX(LineSegments[i][0], LineSegments[i][1]);
		idxded = DIDX(LineSegments[i][2], LineSegments[i][3]);
		linegradients[i] = calGradxy(_gray, idxdst, idxded);
	}
}

void Simplified_EDLines::getFittedLineSegments(std::vector<cv::Vec<double, 11> > &fittedLineSegments) const
{
	const int line_num = (int)LineSegments.size();
	cv::Vec<double, 11> tempLine;
	double matLine[6];
	int idxdst, idxded;

	fittedLineSegments.resize(line_num);

	for (int i = 0; i < line_num; i++)
	{
		idxdst = DIDX(LineSegments[i][0], LineSegments[i][1]);
		idxded = DIDX(LineSegments[i][2], LineSegments[i][3]);
		getLineMat(matLine, idxdst, idxded);
		tempLine[7] = LineSegments[i][0], tempLine[8] = LineSegments[i][1];
		tempLine[9] = LineSegments[i][2], tempLine[10] = LineSegments[i][3];
		fitLine2d(matLine, tempLine);
		transScale(tempLine, 100);
		fittedLineSegments[i] = tempLine;
	}

}
