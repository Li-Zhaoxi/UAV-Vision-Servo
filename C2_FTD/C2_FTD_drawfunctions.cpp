#include "C2_FTD.h"

void C2_FTD::drawInsideLines(cv::Mat &ImgG)
{
	cv::Mat Img_T;
	cv::cvtColor(ImgG, Img_T, CV_GRAY2BGR);

	for (int i = 0; i < line_in_ellipse.size(); i++)
	{
		for (int j = 0; j < line_in_ellipse[i].line_data.size(); j++)
		{
			cv::Point st, ed;
			st.y = line_in_ellipse[i].line_data[j][0];
			st.x = line_in_ellipse[i].line_data[j][1];
			ed.y = line_in_ellipse[i].line_data[j][2];
			ed.x = line_in_ellipse[i].line_data[j][3];
			cv::line(Img_T, st, ed, cv::Scalar(255, 0, 255), 2);
		}
	}

	cv::imshow("In Lines", Img_T);
	cv::waitKey(1);
}