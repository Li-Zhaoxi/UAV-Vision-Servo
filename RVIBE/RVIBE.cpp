#include "RVIBE.h"
RVIBE::RVIBE()
{
	_m_sample = NULL;
	_m_mask = NULL;
	_m_foregroundMatchCount = NULL;
	_slipePick = NULL;
	_slipeSmall = NULL;
	_findRect = NULL;
}
void RVIBE::create(int rows, int cols, CFG_RVIBE &cfg_rvibe)
{
	imgRows = rows, imgCols = cols;
	isFirst = true;
	set_Paras(cfg_rvibe);

	_m_sample = new uchar *[NUM_SAMPLES];
	for (int i = 0; i < NUM_SAMPLES; i++)
	{
		_m_sample[i] = new uchar[rows * cols];
	}

	_m_mask = new uchar[rows * cols];
	_m_foregroundMatchCount = new uchar[rows * cols];


	slipePickRows = imgRows / slipeWindow, slipePickCols = imgCols / slipeWindow;
	_slipePick = new ushort[slipePickRows * slipePickCols];

	slipeSmallRows = (imgRows - rectWindow) / slipeWindow + 1;
	slipeSmallCols = (imgCols - rectWindow) / slipeWindow + 1;
	_slipeSmall = new ushort[slipeSmallRows * slipeSmallCols];

	findRect.create(slipeSmallRows, slipeSmallCols, CV_8UC1);
	_findRect = (uchar*)findRect.data;
}

RVIBE::RVIBE(int rows, int cols, CFG_RVIBE &cfg_rvibe)
{
	create(rows, cols, cfg_rvibe);
}

void RVIBE::run_RVIBE(Mat &Img_G)
{
	findRects.clear(); validRects.clear();
	uchar *_gray=(uchar*)Img_G.data;
	if(isFirst)
		processFirstFrame(_gray);
	else
	{
		//t1 = cv::getTickCount();
		testAndUpdate(_gray);

		//t2 = cv::getTickCount();
		//cout << "RVIBE:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		mask_src = Mat(imgRows, imgCols, CV_8UC1, _m_mask);
		morphologyEx(mask_src, mask_dst, cv::MORPH_OPEN, cv::Mat());
		morphologyEx(mask_dst, mask_dst, cv::MORPH_CLOSE, cv::Mat());

		uchar *_mask=(uchar*)mask_dst.data;
		set_slipePick(_mask);
		set_slipeSmall(_slipePick);

		int idx_i, idx;
		for(int i = 0; i < slipeSmallRows; i++)
		{
			idx_i = i*slipeSmallCols;
			for(int j = 0; j < slipeSmallCols; j++)
			{
				idx = idx_i + j;
				if(_slipeSmall[idx] > minAreaRatio *rectWindow *rectWindow)
					_findRect[idx] = 255;
				else
					_findRect[idx] = 0;
			}
		}
		cv::findContours(findRect, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		Rect tempArea;

		for(int i = 0; i < contours.size(); i++)
		{
			tempArea = cv::boundingRect(contours[i]);
			tempArea.x = tempArea.x * slipeWindow;
			tempArea.y = tempArea.y * slipeWindow;
			tempArea.width = tempArea.width * slipeWindow + rectWindow;
			tempArea.height = tempArea.height *slipeWindow + rectWindow;
			findRects.push_back(tempArea);
			if( tempArea.width > maxRect || tempArea.height > maxRect)
				validRects.push_back(0);
			else
				validRects.push_back(1);
		}
	}
}
void RVIBE::run_RVIBE(Mat Img_C, Mat Img_G)
{
	findRects.clear(); validRects.clear();
	uchar *_gray=(uchar*)Img_G.data;
	if(isFirst)
		processFirstFrame(_gray);
	else
	{
		testAndUpdate(_gray);
		mask_src = Mat(imgRows, imgCols, CV_8UC1, _m_mask);
		morphologyEx(mask_src, mask_dst, cv::MORPH_OPEN, cv::Mat());
		morphologyEx(mask_dst, mask_dst, cv::MORPH_CLOSE, cv::Mat());

		uchar *_mask=(uchar*)mask_dst.data;
		set_slipePick(_mask);
		set_slipeSmall(_slipePick);
		
		int idx_i, idx;
		for(int i = 0; i < slipeSmallRows; i++)
		{
			idx_i = i*slipeSmallCols;
			for(int j = 0; j < slipeSmallCols; j++)
			{
				idx = idx_i + j;
				if(_slipeSmall[idx] > minAreaRatio *rectWindow *rectWindow)
					_findRect[idx] = 255;
				else
					_findRect[idx] = 0;
			}
		}
		cv::findContours(findRect, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		Rect tempArea;

		for(int i = 0; i < contours.size(); i++)
		{
			tempArea = cv::boundingRect(contours[i]);
			tempArea.x = tempArea.x * slipeWindow;
			tempArea.y = tempArea.y * slipeWindow;
			tempArea.width = tempArea.width * slipeWindow + rectWindow;
			tempArea.height = tempArea.height *slipeWindow + rectWindow;
			findRects.push_back(tempArea);
			if( tempArea.width > maxRect || tempArea.height > maxRect)
				validRects.push_back(0);
			else
				validRects.push_back(1);
		}
	}
}


void RVIBE::init()
{
	isFirst = true;
}

void RVIBE::release()
{
	if (_m_sample != NULL)
	{
		for (int i = 0; i < NUM_SAMPLES; i++)
		{
			if(_m_sample[i]!=NULL)
				delete[] _m_sample[i];
		}
		delete[] _m_sample;
	}

	if(_m_mask!=NULL)
		delete[] _m_mask;
	
	if(_m_foregroundMatchCount!=NULL)
		delete[] _m_foregroundMatchCount;

	if(_slipePick!=NULL) 
		delete[] _slipePick;

	if(_slipeSmall!=NULL)
		delete[] _slipeSmall;
}

void RVIBE::set_Paras(CFG_RVIBE &cfg_rvibe)
{
	NUM_SAMPLES = cfg_rvibe.get_NUM_SAMPLES();
	MIN_MATCHES = cfg_rvibe.get_MIN_MATCHES();
	RADIUS = cfg_rvibe.get_RADIUS();
	SUBSAMPLE_FACTOR = cfg_rvibe.get_SUBSAMPLE_FACTOR();
	slipeWindow = cfg_rvibe.get_slipeWindow();
	rectWindow = cfg_rvibe.get_rectWindow();
	minAreaRatio = cfg_rvibe.get_minAreaRatio();
	maxRect = cfg_rvibe.get_maxRect();
}

void RVIBE::processFirstFrame(const uchar *_image)
{
	int idx, idx_i;
	const int c_off[9] = {-imgCols-1, -imgCols, -imgCols+1, -1, 0, 1, imgCols-1, imgCols, imgCols+1};
	for (int i = 1; i < imgRows -1; i++)
	{
		idx_i = i*imgCols;
		for (int j = 1; j < imgCols -1; j++)
		{
			idx=idx_i+j;
			for (int k = 0; k < NUM_SAMPLES; k++)
				_m_sample[k][idx] = _image[idx + c_off[ rng.next() % 9 ] ];
		}
	}
	isFirst = false;
}

void RVIBE::testAndUpdate(const uchar *_image)
{
	int idx, idx_i, idx_neibor, i, j;
	int matches = 0, count = 0, dist;
	const int c_off[9] = {-imgCols-1, -imgCols, -imgCols+1, -1, 0, 1, imgCols-1, imgCols, imgCols+1};
	bool isMatches = false;

//	bool haveSSE2 = cv::checkHardwareSupport(CV_CPU_SSE2);

	for (i = 1; i < imgRows -1; i++)
	{
		idx_i = i * imgCols;
		for (j = 1; j < imgCols -1; j++)
		{
			idx = idx_i + j;
			isMatches = false;
			matches = count = 0;
			for(matches = 0, count = 0; count < NUM_SAMPLES; count++)
			{
				dist = abs( _m_sample[count][idx] - _image[idx] );
				if(dist < RADIUS)
				{
					matches++;
					if(matches >= MIN_MATCHES)
					{
						isMatches = true;
						break;
					}
				}
			}
			if(isMatches)
			{
				_m_foregroundMatchCount[idx]=0;
				_m_mask[idx]=0;
				if ( (rng.next() % SUBSAMPLE_FACTOR) == 0)
				{
					_m_sample[ rng.next() % NUM_SAMPLES ][idx] = _image[idx];
				}
				if ( (rng.next() % SUBSAMPLE_FACTOR) == 0)
				{
					idx_neibor = idx + c_off[ rng.next() % 9 ];
					_m_sample[ rng.next() % NUM_SAMPLES ][idx_neibor] = _image[idx];
				}
			}
			else
			{
				_m_foregroundMatchCount[idx]++;
				_m_mask[idx]=255;
				if (_m_foregroundMatchCount[idx] > 50)
				{
					if ( (rng.next() % SUBSAMPLE_FACTOR) == 0)
					{
						_m_sample[ rng.next() % NUM_SAMPLES ][idx] = _image[idx];
					}
				}
			}

		}
	}
}

void RVIBE::set_slipePick(uchar *_mask_data)
{
	int idx_i, idx, st_x, ed_x, st_y, ed_y, idx_k;
	for(int i = 0; i < slipePickRows; i++)
	{
		idx_i = i * slipePickCols;
		for(int j = 0; j < slipePickCols; j++)
		{
			idx = idx_i + j;
			_slipePick[idx] = 0;
			st_x = i * slipeWindow; ed_x = st_x + slipeWindow;
			st_y = j * slipeWindow; ed_y = st_y + slipeWindow;
			for(int k = st_x; k < ed_x; k++)
			{
				idx_k = k * imgCols;
				for(int p = st_y; p < ed_y; p++)
				{
					if( _mask_data[idx_k+p])
						_slipePick[idx]++;
				}
			}
		}
	}
}

void RVIBE::set_slipeSmall(ushort *_slipepick)
{
	int idx_i, idx, idx_k, ed_k, ed_p;
	int slipeNum = rectWindow / slipeWindow;
	for(int i = 0; i < slipeSmallRows; i++)
	{
		idx_i = i * slipeSmallCols;
		for(int j = 0; j < slipeSmallCols; j++)
		{
			idx = idx_i + j;
			_slipeSmall[idx] = 0;
			ed_k = i + slipeNum; ed_p = j + slipeNum;
			for(int k = i; k < ed_k; k++)
			{
				idx_k = k * slipePickCols;
				for(int p = j; p < ed_p; p++)
				{
					if(_slipepick[idx_k+p])
						_slipeSmall[idx] += _slipepick[idx_k +p];
				}
			}
		}
	}
}