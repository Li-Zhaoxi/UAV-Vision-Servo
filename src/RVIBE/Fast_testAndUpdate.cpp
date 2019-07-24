#include "RVIBE.h"


class VIBE_testAndUpdate : public cv::ParallelLoopBody
{
public:
	VIBE_testAndUpdate(
		unsigned char *_gray_,
		int _rows_,
		int _cols_,
		int _NUM_SAMPLES,
		int _RADIUS,
		int _MIN_MATCHES,
		unsigned char **__m_sample,
		unsigned char *__m_foregroundMatchCount,
		unsigned char *__m_mask
	) :_gray(_gray_), rows(_rows_), cols(_cols_), NUM_SAMPLES(_NUM_SAMPLES), RADIUS(_RADIUS), MIN_MATCHES(_MIN_MATCHES), _m_sample(__m_sample), _m_foregroundMatchCount(__m_foregroundMatchCount), _m_mask(__m_mask)
	{}
	
	void operator()(const cv::Range& range) const
	{
		const int begin = range.start;
		const int end = range.end;
		const int c_off[9] = { -cols - 1, -cols, -cols + 1, -1, 0, 1, cols - 1, cols, cols + 1 };

		for (int i = begin; i < end; ++i)
		{
			int idx_i = i%cols;
			int idx_j = i - idx_i*cols;
			if (idx_i == 0 || idx_i == rows-1 || idx_j == 0 || idx_j == cols-1)
				continue;

			bool isMatches = false;
			int matches(0), count(0), dist, idx(i);

			for (matches = 0, count = 0; count < NUM_SAMPLES; count++)
			{
				dist = abs(_m_sample[count][idx] - _gray[idx]);
				if (dist < RADIUS)
				{
					matches++;
					if (matches >= MIN_MATCHES)
					{
						isMatches = true;
						break;
					}
				}
			}

			cv::RNG rng;
			int idx_neibor;
			if (isMatches)
			{
				_m_foregroundMatchCount[idx] = 0;
				_m_mask[idx] = 0;
				if ((rng.next() % SUBSAMPLE_FACTOR) == 0)
				{
					_m_sample[rng.next() % NUM_SAMPLES][idx] = _gray[idx];
				}
				if ((rng.next() % SUBSAMPLE_FACTOR) == 0)
				{
					idx_neibor = idx + c_off[rng.next() % 9];
					_m_sample[rng.next() % NUM_SAMPLES][idx_neibor] = _gray[idx];
				}
			}
			else
			{
				_m_foregroundMatchCount[idx]++;
				_m_mask[idx] = 255;
				if (_m_foregroundMatchCount[idx] > 50)
				{
					if ((rng.next() % SUBSAMPLE_FACTOR) == 0)
					{
						_m_sample[rng.next() % NUM_SAMPLES][idx] = _gray[idx];
					}
				}
			}

		}
	}

private:
	unsigned char *_gray;
	int rows;
	int cols;

	int NUM_SAMPLES;
	int RADIUS;
	int MIN_MATCHES;
	unsigned char **_m_sample;

	int SUBSAMPLE_FACTOR;
	unsigned char *_m_foregroundMatchCount;
	unsigned char *_m_mask;
};


void RVIBE::run_RVIBE_tbb(Mat &Img_G)
{
	findRects.clear(); validRects.clear();
	uchar *_gray = (uchar*)Img_G.data;
	if (isFirst)
		processFirstFrame(_gray);
	else
	{
		//t1 = cv::getTickCount();
		cv::parallel_for_(cv::Range(0, imgRows*imgCols), VIBE_testAndUpdate(_gray, imgRows, imgCols, NUM_SAMPLES, RADIUS, MIN_MATCHES, _m_sample, _m_foregroundMatchCount, _m_mask));
		//t2 = cv::getTickCount();
		//cout << "RVIBE:" << (t2 - t1) * 1000 / cv::getTickFrequency() << endl;
		mask_src = Mat(imgRows, imgCols, CV_8UC1, _m_mask);
		morphologyEx(mask_src, mask_dst, cv::MORPH_OPEN, cv::Mat());
		morphologyEx(mask_dst, mask_dst, cv::MORPH_CLOSE, cv::Mat());

		uchar *_mask = (uchar*)mask_dst.data;
		set_slipePick(_mask);
		set_slipeSmall(_slipePick);

		int idx_i, idx;
		for (int i = 0; i < slipeSmallRows; i++)
		{
			idx_i = i*slipeSmallCols;
			for (int j = 0; j < slipeSmallCols; j++)
			{
				idx = idx_i + j;
				if (_slipeSmall[idx] > minAreaRatio *rectWindow *rectWindow)
					_findRect[idx] = 255;
				else
					_findRect[idx] = 0;
			}
		}
		cv::findContours(findRect, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		Rect tempArea;

		for (int i = 0; i < contours.size(); i++)
		{
			tempArea = cv::boundingRect(contours[i]);
			tempArea.x = tempArea.x * slipeWindow;
			tempArea.y = tempArea.y * slipeWindow;
			tempArea.width = tempArea.width * slipeWindow + rectWindow;
			tempArea.height = tempArea.height *slipeWindow + rectWindow;
			if (tempArea.width > maxRect || tempArea.height > maxRect)
				validRects.push_back(0);
			else
			{
				validRects.push_back(1);
				findRects.push_back(tempArea);
			}
		}
	}
}