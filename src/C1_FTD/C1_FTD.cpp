#include "C1_FTD.h"

C1_FTD::C1_FTD()
{
	lambda_line = 15;
	Error_Angle = CV_PI / 6;


	CrossAngle = CV_PI / 2; //十字夹角
	FeaturesDistance = 60; // 十字宽度

	// The parameters initialized -1 need to be set by users.
	isFind = 0;
	//lineWidth = maxCircleDiameter = maxratioCenter2Line = minCircleDiameter = -1;
}
void C1_FTD::runC1_FTD(std::vector< cv::Vec<double, 11> > &detLineSegments, std::vector< cv::Point2d > &LineGradients)
{
	// Init
	isFind = false;
	crossCenter.x = crossCenter.y = 0;
	cross4Points[0].x = cross4Points[0].y = 0;
	cross4Points[1].x = cross4Points[1].y = 0;
	cross4Points[2].x = cross4Points[2].y = 0;
	cross4Points[3].x = cross4Points[3].y = 0;

//	cross_feature.clear(); correctedLines.clear(); searchedRects.clear();

	std::vector<CrossFeature> ValidFeatures;

	// 矫正直线段的起始点
	CorrectLineSegments(detLineSegments, correctedLines); 
	// 特征点检测
	CrossFeatureExtraction(correctedLines, LineGradients, cross_feature);

	// 根据特征点组合出目标区域
	RectangleSearch(cross_feature, ValidFeatures, searchedRects);
	// 选择最终的十字区域
	SelectFinalCross(searchedRects, ValidFeatures);
}

void C1_FTD::CrossFeatureExtraction(std::vector<cv::Vec4d> &in_correctedLines, std::vector< cv::Point2d > &LinesGrad, std::vector<CrossFeature> &out_crossfea)
{
	// Cross ���ȡֵ��Χ
	double angleAcuteDown = cos(cv::min(CV_PI / 2, CrossAngle + Error_Angle)),
		angleAcuteUp = cos(cv::max(CrossAngle - Error_Angle, 0.0));
	// Cross �۽�ȡֵ��Χ
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
				O.x = line1[0] + t*n_i.x, O.y = line1[1] + t*n_i.y;
				dir_t = t > 0.5 ? -1 : 1, dir_p = p > 0.5 ? -1 : 1;
				v_t = dir_t*dir_p*ni_mul_nj;
				if (v_t < 0)
				{
					V_l.x = dir_t* n_i.x / ln_i, V_l.y = dir_t* n_i.y / ln_i;
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


void C1_FTD::CorrectLineSegments(const std::vector< cv::Vec<double, 11> > &in_fitLines, std::vector<cv::Vec4d> &out_correctedLines)
{
	const int line_num = (int)in_fitLines.size();

	double distP2Line;
	cv::Vec4d crtLines;

	out_correctedLines.resize(line_num);

	for (int i = 0; i < line_num; i++)
	{
		//corrected line1_st.
		distP2Line = in_fitLines[i][1] * in_fitLines[i][7] - in_fitLines[i][0] * in_fitLines[i][8] + in_fitLines[i][6];

		crtLines[0] = distP2Line*in_fitLines[i][1] + in_fitLines[i][7];
		crtLines[1] = -distP2Line*in_fitLines[i][0] + in_fitLines[i][8];

		//corrected line1_ed
		distP2Line = in_fitLines[i][1] * in_fitLines[i][9] - in_fitLines[i][0] * in_fitLines[i][10] + in_fitLines[i][6];
		crtLines[2] = distP2Line*in_fitLines[i][1] + in_fitLines[i][9];
		crtLines[3] = -distP2Line*in_fitLines[i][0] + in_fitLines[i][10];

		out_correctedLines[i] = crtLines;
	}
}


void C1_FTD::RectangleSearch(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< cv::Vec4i > &out_rects) const
{
	out_rects.clear();

	std::vector< std::vector<int> > features_adjacency;
	ConstructAdjacency(in_crossfea, ValidFeatures, features_adjacency);

	const int valid_num = (int)ValidFeatures.size();

	cv::Vec4i findTemp;
	int idx_CrossFeature[4][2];
	int nodesFocus = -1;
	for (int i = 0; i < valid_num; i++)
	{
		findTemp[0] = i;
		nodesFocus = 0; idx_CrossFeature[0][0] = i; idx_CrossFeature[0][1] = -1;
		while (nodesFocus != -1)
		{
			if (idx_CrossFeature[nodesFocus][1] + 1 == features_adjacency[idx_CrossFeature[nodesFocus][0]].size())//�����ǰfocus
			{
				nodesFocus -= 1;
				continue;
			}
			idx_CrossFeature[nodesFocus][1] += 1;
			int newIdx = features_adjacency[idx_CrossFeature[nodesFocus][0]][idx_CrossFeature[nodesFocus][1]];
			nodesFocus += 1;
			if (nodesFocus == 4)
			{
				if (features_adjacency[idx_CrossFeature[3][0]][idx_CrossFeature[3][1]] == idx_CrossFeature[0][0])
				{
					out_rects.push_back(findTemp);
					break;
				}
				else
				{
					nodesFocus -= 1;
					continue;
				}
			}
			idx_CrossFeature[nodesFocus][0] = newIdx; idx_CrossFeature[nodesFocus][1] = -1;
			findTemp[nodesFocus] = newIdx;

		}
	}

}


void C1_FTD::ConstructAdjacency(std::vector<CrossFeature> &in_crossfea, std::vector<CrossFeature> &ValidFeatures, std::vector< std::vector<int> > &out_adjacency) const
{
	const double T1 = std::cos(Error_Angle), T2 = -T1;
	const int all_features = (int)in_crossfea.size();
	int minWidth = std::min(int(FeaturesDistance - FeaturesDistance / 2), 3), maxWidth = (int)(FeaturesDistance + FeaturesDistance);

	ValidFeatures.clear();
	for (int i = 0; i < all_features; i++)
	{
		if (in_crossfea[i].isValid)
			ValidFeatures.push_back(in_crossfea[i]);
	}
	const int valid_num = (int)ValidFeatures.size();

	// ��ʼ�ж�
	double judgeValue;
	CrossFeature *feature_1(NULL), *feature_2(NULL), *valid_features = ValidFeatures.data();
	cv::Point2d vecOiOj;
	std::vector<int> linkTemp;

	out_adjacency.clear();

	for (int i = 0; i < valid_num; i++)
	{
		feature_1 = valid_features + i;
		linkTemp.clear();
		for (int j = 0; j < valid_num; j++)
		{
			if (i == j)
				continue;
			feature_2 = valid_features + j;

			judgeValue = feature_1->V_r.x*feature_2->V_l.x + feature_1->V_r.y*feature_2->V_l.y;
			if (judgeValue <= T1)
				continue;

			judgeValue = feature_1->V_l.x*feature_2->V_r.x + feature_1->V_l.y*feature_2->V_r.y;
			if (judgeValue >= T2)
				continue;

			vecOiOj.x = feature_2->O.x - feature_1->O.x, vecOiOj.y = feature_2->O.y - feature_1->O.y;
			judgeValue = sqrt(vecOiOj.x*vecOiOj.x + vecOiOj.y*vecOiOj.y);
			if (judgeValue<minWidth || judgeValue>maxWidth)
				continue;

			judgeValue = (feature_1->V_l.x*vecOiOj.x + feature_1->V_l.y*vecOiOj.y) / judgeValue;
			if (judgeValue >= T2)
				continue;

			judgeValue = feature_1->V_r.x*vecOiOj.y - feature_1->V_r.y*vecOiOj.x;
			if (judgeValue >= 0)
				continue;


			linkTemp.push_back(j);
		}
		out_adjacency.push_back(linkTemp);
	}
}


void C1_FTD::SelectFinalCross(std::vector< cv::Vec4i > &in_rects, std::vector<CrossFeature> &in_ValidFeatures)
{
	const int findNumber = (int)in_rects.size();
	std::vector <cv::Point2f> rectPoints(4);
	cv::RotatedRect tempRect;
	double ratioRect = 0, maxRatio = 0, realArea; 
	int idx = -1;

	for (int i = 0; i < findNumber; i++)
	{
		rectPoints[0] = in_ValidFeatures[in_rects[i][0]].O;
		rectPoints[1] = in_ValidFeatures[in_rects[i][1]].O;
		rectPoints[2] = in_ValidFeatures[in_rects[i][2]].O;
		rectPoints[3] = in_ValidFeatures[in_rects[i][3]].O;
		tempRect = cv::minAreaRect(rectPoints);
		//realArea = calArea(rectPoints) / (tempRect.size.width*tempRect.size.height);
		realArea = 100 / (tempRect.size.width*tempRect.size.height);
		if ((tempRect.size.width*tempRect.size.height) < 64)
			continue;
		if (realArea > maxRatio)
		{
			maxRatio = realArea;
			idx = i;
		}
	}

	if (idx == -1)
	{
		isFind = false;
	}
	else
	{
		isFind = true;
		cross4Points[0] = in_ValidFeatures[in_rects[idx][0]].O;
		cross4Points[1] = in_ValidFeatures[in_rects[idx][1]].O;
		cross4Points[2] = in_ValidFeatures[in_rects[idx][2]].O;
		cross4Points[3] = in_ValidFeatures[in_rects[idx][3]].O;

		crossCenter.x = cross4Points[0].x + cross4Points[1].x + cross4Points[2].x + cross4Points[3].x;
		crossCenter.y = cross4Points[0].y + cross4Points[1].y + cross4Points[2].y + cross4Points[3].y;
		crossCenter.x = crossCenter.x / 4; crossCenter.y = crossCenter.y / 4;
	}



}
