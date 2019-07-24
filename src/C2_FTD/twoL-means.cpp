#include "C2_FTD.h"
// d(\theta_i, \theta_j) = arccos[cos(2\theta_i - 2\theta_j)]
static inline double dist_thetai_thetaj(double theta_i, double theta_j)
{
	return std::acos(std::cos(2 * (theta_i - theta_j)));
}

// 
static inline double n_thetai_thetaj(double theta_i, double theta_j)
{
	return std::asin(std::sin(2 * (theta_i - theta_j)));
}

/*
k-means center initialization using the following algorithm:
Arthur & Vassilvitskii (2007) k-means++: The Advantages of Careful Seeding
*/
static void generate_2L_means_Centers(const std::vector<double> &_data, double &center_1, double &center_2, cv::RNG &rng) // 生成两个聚类中心点
{
	const int SPP_TRIALS = 3;
	int N = (int)_data.size(), i, j, centers[2];
	double sum0 = 0;

	const double *data = _data.data();
	std::vector<double> _dist(N * 3);
	double *dist = _dist.data(), *tdist = dist + N, *tdist2 = tdist + N;

	// Step 1: 从输入的数据点集合中随机选择一个点作为第一个聚类中心
	centers[0] = (unsigned)rng % N; // 随机选择第i个点的index
	// Step 2: 对于数据集中的每一个点x，计算它与最近聚类中心(指已选择的聚类中心)的距离D(x)
	for (i = 0; i < N; i++) // 计算每个数据点距离第一个中心点的距离
	{
		dist[i] = dist_thetai_thetaj(data[i], data[centers[0]]);
		sum0 += dist[i];
	}
	// Step 3: 选择一个新的数据点作为新的聚类中心，选择的原则是：D(x)较大的点，被选取作为聚类中心的概率较大

	double bestSum = DBL_MAX;
	int bestCenter = -1;
	for (j = 0; j < SPP_TRIALS; j++)
	{
		double p = (double)rng*sum0, s = 0;
		for (i = 0; i < N - 1; i++)
			if ((p -= dist[i]) <= 0)
				break;
		int ci = i;

		for (i = 0; i < N; i++)
		{
			tdist2[i] = std::min(dist_thetai_thetaj(data[i], data[ci]), dist[i]);
			s += tdist2[i];
		}

		if (s < bestSum)
		{
			bestSum = s;
			bestCenter = ci;
			std::swap(tdist, tdist2);
		}

	}
	centers[1] = bestCenter;

	center_1 = data[centers[0]];
	center_2 = data[centers[1]];
}




double C2_FTD::twoL_means(std::vector<double> &input_lines, std::vector<int> &_bestLabels, double &center_1, double &center_2)
{
	const int SPP_TRIALS = 3;
	const int attempts = 3;
	
	int N = (int)input_lines.size();
	double *_data = input_lines.data();
	_bestLabels.resize(N);
	int* labels = _bestLabels.data();

	int a, iter, i;

	std::vector<int> counters(2);
	cv::RNG& rng = cv::theRNG();
	double _tcenters[2], _tcenters_new[2], compactness;

	for (a = 0; a < attempts; a++)
	{
		double max_center_shift = DBL_MAX, dist1, dist2, sum1, sum2, num_sum1, num_sum2, convergence;

		generate_2L_means_Centers(input_lines, _tcenters[0], _tcenters[1], rng);

		for (iter = 0;;)
		{
			// Redistribute each data point to the nearest cluster average
			compactness = 0;
			for (i = 0; i < N; i++)
			{
				dist1 = dist_thetai_thetaj(_data[i], _tcenters[0]);
				dist2 = dist_thetai_thetaj(_data[i], _tcenters[1]);
				labels[i] = dist1 < dist2 ? 0 : 1; // Update C1, C2
				compactness += std::min(dist1, dist2);
			}

			num_sum1 = sum1 = num_sum2 = sum2 = 0;
			for (i = 0; i < N; i++)
			{
				if (labels[i] == 0)
				{
					sum1 += n_thetai_thetaj(_data[i], _tcenters[0]);
					num_sum1++;
				}
				else
				{
					sum2 += n_thetai_thetaj(_data[i], _tcenters[1]);
					num_sum2++;
				}
			}
			_tcenters_new[0] = _tcenters[0] + (num_sum1 > 0 ? sum1 / (2 * num_sum1) : 0);
			_tcenters_new[1] = _tcenters[1] + (num_sum2 > 0 ? sum2 / (2 * num_sum2) : 0);

			
			convergence = std::abs(_tcenters_new[0] - _tcenters[0]) + std::abs(_tcenters_new[1] - _tcenters[1]);
			std::swap(_tcenters, _tcenters_new);
			if (++iter == 100 || convergence <= FLT_EPSILON)
				break;
		}
	}

	if (_tcenters[0] <= -CV_PI / 2) _tcenters[0] += CV_PI;
	if (_tcenters[0] > CV_PI / 2) _tcenters[0] -= CV_PI;

	if (_tcenters[1] <= -CV_PI / 2) _tcenters[1] += CV_PI;
	if (_tcenters[1] > CV_PI / 2) _tcenters[1] -= CV_PI;

	center_1 = _tcenters[0];
	center_2 = _tcenters[1];
	return compactness;
}
