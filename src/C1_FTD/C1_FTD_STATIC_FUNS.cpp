#include "C1_FTD.h"


void C1_FTD::CrossFeatureExtraction(
	std::vector<cv::Vec4d> &in_correctedLines,
	std::vector< cv::Point2d > &LinesGrad,
	std::vector<CrossFeature> &out_crossfea,
	double Error_Angle,
	double lambda_line)
{

	const double CrossAngle = CV_PI / 2;


	// 设置夹角范围
	double angleAcuteDown = cos(cv::min(CV_PI / 2, CrossAngle + Error_Angle)),
		angleAcuteUp = cos(cv::max(CrossAngle - Error_Angle, 0.0));
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


			if (((t > -err_t && t <= err_t / 2) || (t >= (1 - err_t / 2) && t < 1 + err_t)) &&
				((p > -err_p && p <= err_p / 2) || (p >= (1 - err_p / 2) && p < 1 + err_p)))
			{
				O.x = line1[0] + t * n_i.x, O.y = line1[1] + t * n_i.y;
				dir_t = t > 0.5 ? -1 : 1, dir_p = p > 0.5 ? -1 : 1;
				v_t = dir_t * dir_p*ni_mul_nj;
				if (v_t < 0)
				{
					V_l.x = dir_t * n_i.x / ln_i, V_l.y = dir_t * n_i.y / ln_i;
					V_r.x = dir_p * n_j.x / ln_j, V_r.y = dir_p * n_j.y / ln_j;
					G_l = LinesGrad[i];
					G_r = LinesGrad[j];
				}
				else
				{
					V_l.x = dir_p * n_j.x / ln_j, V_l.y = dir_p * n_j.y / ln_j;
					V_r.x = dir_t * n_i.x / ln_i, V_r.y = dir_t * n_i.y / ln_i;
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



void C1_FTD::drawCrossFeatures(std::vector<CrossFeature> &in_crossfea, cv::Mat &ImgG, bool isShow)
{
	if (isShow)
	{
		cv::Mat ImgT = ImgG.clone();
		cv::cvtColor(ImgT, ImgT, CV_GRAY2BGR);
		int l = 10;
		for (int i = 0; i < in_crossfea.size(); i++)
		{
			if (in_crossfea[i].isValid == false)
				continue;
			circle(ImgT, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)), 2, cv::Scalar(0, 0, 255), 2);
			cv::line(ImgT, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)), 
				cv::Point((int)(in_crossfea[i].O.y + l * in_crossfea[i].V_l.y + 0.5), 
				(int)(in_crossfea[i].O.x + l * in_crossfea[i].V_l.x + 0.5)), cv::Scalar(255, 0, 0), 2);
			cv::line(ImgT, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)), 
				cv::Point((int)(in_crossfea[i].O.y + l * in_crossfea[i].V_r.y + 0.5), (int)(in_crossfea[i].O.x + l * in_crossfea[i].V_r.x + 0.5)), 
				cv::Scalar(255, 0, 255), 2);

		}
		cv::imshow("Cross Features", ImgT);
		//cv::imwrite("CrossFeatures.png", ImgT);
		cv::waitKey(1);
	}
	else
	{
		if (ImgG.channels() == 1)
			cv::cvtColor(ImgG, ImgG, CV_GRAY2BGR);
		int l = 10;
		for (int i = 0; i < in_crossfea.size(); i++)
		{
			if (in_crossfea[i].isValid == false)
				continue;
			circle(ImgG, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)), 2, cv::Scalar(0, 0, 255), 2);
			cv::line(ImgG, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)),
				cv::Point((int)(in_crossfea[i].O.y + l * in_crossfea[i].V_l.y + 0.5),
				(int)(in_crossfea[i].O.x + l * in_crossfea[i].V_l.x + 0.5)), cv::Scalar(255, 0, 0), 2);
			cv::line(ImgG, cv::Point((int)(in_crossfea[i].O.y + 0.5), (int)(in_crossfea[i].O.x + 0.5)),
				cv::Point((int)(in_crossfea[i].O.y + l * in_crossfea[i].V_r.y + 0.5), (int)(in_crossfea[i].O.x + l * in_crossfea[i].V_r.x + 0.5)),
				cv::Scalar(255, 0, 255), 2);
		}
	}
}