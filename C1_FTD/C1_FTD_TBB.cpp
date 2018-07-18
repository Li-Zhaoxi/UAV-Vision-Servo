#include <C1_FTD.h>

class C1_FTD_CrossFeatureExtraction : public cv::ParallelLoopBody
{
public:
	C1_FTD_CrossFeatureExtraction(
		double _angleAcuteDown,
		double _angleAcuteUp,
		double _angleObtuseDown,
		double _angleObtuseUp,
		std::vector<cv::Vec4d> &_correctedLines
	) :angleAcuteDown(_angleAcuteDown), angleAcuteUp(_angleAcuteUp), angleObtuseDown(_angleObtuseDown), angleObtuseUp(_angleObtuseUp), correctedLines(_correctedLines)
	{}

	void operator()(const cv::Range& range) const
	{

		const int begin = range.start;
		const int end = range.end;

		const int line_num = (int)correctedLines.size();
		const int CF_Num = (line_num - 1)*line_num / 2;
		for (int i = begin; i < end; ++i)
		{
			int idx_i, idx_j;

		}


	}

private:
	double angleAcuteDown, angleAcuteUp, angleObtuseDown, angleObtuseUp;
	std::vector<cv::Vec4d> &correctedLines;
};



void C1_FTD::CrossFeatureExtraction_tbb(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea)
{
	// Cross Èñ½ÇÈ¡Öµ·¶Î§
	double angleAcuteDown = cos(cv::min(CV_PI / 2, CrossAngle + Error_Angle)),
		angleAcuteUp = cos(cv::max(CrossAngle - Error_Angle, 0.0));
	// Cross ¶Û½ÇÈ¡Öµ·¶Î§
	double angleObtuseDown = cos(cv::min(CV_PI - CrossAngle + Error_Angle, CV_PI)),
		angleObtuseUp = cos(cv::max(CV_PI - CrossAngle - Error_Angle, CV_PI / 2));


	const int line_num = (int)in_correctedLines.size();

	out_crossfea.resize((line_num - 1 + 0)*line_num / 2);
	int idx_out_crossfea = 0;

	cv::Vec4d line1, line2;
	cv::Point2d n_i, n_j, lsti_lstj;
	cv::Point2d O, V_l, V_r, G_l, G_r;
	double ln_i, ln_j, cosTheta, ni_mul_nj, t, p, dir_t, dir_p, v_t, err_t, err_p;
	bool isValid = false;

	for (int i = 0; i < line_num; i++)
	{
		line1 = in_correctedLines[i];
		n_i.x = line1[2] - line1[0], n_i.y = line1[3] - line1[1];
		ln_i = sqrt(n_i.x*n_i.x + n_i.y*n_i.y);
		err_t = lambda_line / ln_i;

		for (int j = i + 1; j < line_num; j++)
		{
			isValid = true;

			line2 = in_correctedLines[j];
			n_j.x = line2[2] - line2[0], n_j.y = line2[3] - line2[1];
			ln_j = sqrt(n_j.x*n_j.x + n_j.y*n_j.y);
			err_p = lambda_line / ln_j;

			lsti_lstj.x = line2[0] - line1[0], lsti_lstj.y = line2[1] - line1[1]; //(l^st_j - l^st_i)
			ni_mul_nj = n_i.x*n_j.y - n_i.y*n_j.x; // n_i x n_j

			t = (lsti_lstj.x*n_j.y - lsti_lstj.y*n_j.x) / ni_mul_nj;
			p = (lsti_lstj.x*n_i.y - lsti_lstj.y*n_i.x) / ni_mul_nj;


			if (err_t >= 0.5 || err_p >= 0.5)
				isValid = false;
			if (((t > (0 - err_t) && t <= (0 + err_t)) || (t >= (1 - err_t) && t < 1 + err_t)) &&
				((p >(0 - err_p) && p <= (0 + err_p)) || (p >= (1 - err_p) && p < 1 + err_p)))
			{
				O.x = line1[0] + t*n_i.x, O.y = line1[1] + t*n_i.y;
				dir_t = t > 0.5 ? -1 : 1, dir_p = p > 0.5 ? -1 : 1;
				v_t = dir_t*dir_p*ni_mul_nj;
				if (v_t < 0)
				{
					V_l = dir_t*n_i / ln_i;
					V_r = dir_p*n_j / ln_j;
					G_l = LinesGrad[i];
					G_r = LinesGrad[j];
				}
				else
				{
					V_l = dir_p*n_j / ln_j;
					V_r = dir_t*n_i / ln_i;
					G_l = LinesGrad[j];
					G_r = LinesGrad[i];
				}

				cosTheta = V_l.x*V_r.x + V_l.y*V_r.y;
				if (!((cosTheta > angleAcuteDown&&cosTheta < angleAcuteUp) ||
					(cosTheta > angleObtuseDown&&cosTheta < angleObtuseUp)))
					isValid = false;
			}
			else
				isValid = false;

			if (!(G_l.x*V_l.y - G_l.y*V_l.x > 0))
				isValid = false;
			if (!(G_r.x*V_r.y - G_r.y*V_r.x < 0))
				isValid = false;


			if (isValid)
			{
				out_crossfea[idx_out_crossfea].O = O;
				out_crossfea[idx_out_crossfea].V_l = V_l;
				out_crossfea[idx_out_crossfea].V_r = V_r;
				out_crossfea[idx_out_crossfea].G_l = G_l;
				out_crossfea[idx_out_crossfea].G_r = G_r;
				out_crossfea[idx_out_crossfea].isValid = true;
			}
			else
				out_crossfea[idx_out_crossfea].isValid = false;

			idx_out_crossfea++;
			continue;


		}
	}
}