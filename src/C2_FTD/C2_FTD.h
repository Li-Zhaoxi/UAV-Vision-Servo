#ifndef _C2_FTD_H_
#define _C2_FTD_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <uavdef.h>

struct LineINEllipse
{
	int idxEllipse;   //��Ӧ��Բ���±꣬ Ĭ��Ϊ-1
	std::vector<cv::Vec4i> line_data; //
	float lineScore;
	float cross4Direct[4];
};

class UAV_EXPORTS C2_FTD
{
public:
	C2_FTD();
	void create(int rows, int cols);
	C2_FTD(int rows, int cols);
	void runC2_FTD(std::vector<cv::RotatedRect> &detEllipses, std::vector<cv::Vec4i> &detLineSegments, int rows, int cols);

	void drawInsideLines(cv::Mat &ImgG);
	void drawC2_FTD(cv::Mat &ImgG);

	//�������ս����ȡ�������
	cv::RotatedRect landEllipse; //Ŀ����Բ
	cv::Vec4f landCross;         //Ŀ��ʮ����Ϣ
	bool isFind;                 //�Ƿ���ҵ�

private:
	cv::Mat lineMap; //ֱ�ߵ�ͼ�����ڲ�������Բ�ڵ�ֱ��
	int drows, dcols, irows, icols;

	cv::Vec4i *_detLines_data;
	std::vector<LineINEllipse> line_in_ellipse;
	std::vector<cv::Vec6d> ellipse_parms;


	// ��irows,icols, ����ֱ�߶ε����������Ϣ
	void getLineMap(unsigned short *_lineMap, std::vector< cv::Vec4i > &cdtLines);

	//��ȡ��ÿ����Բ�ڲ���ֱ�߶� Step1 : Extract Candidate Line Segments of Cross
	void getInsideLines(const ushort *_lineMap, const std::vector<cv::RotatedRect> &cdtEllipse, std::vector<LineINEllipse> &LIE);
	 
	// ����Բ��״����תΪһ�㷽��6������
	static void ELPShape2Equation(const std::vector<cv::RotatedRect> &in_elps, std::vector<cv::Vec6d> &out_parms);


	void SlopeClustering(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detElps);
	void SelectFinalCross(std::vector<LineINEllipse> &LIE, std::vector<cv::RotatedRect> &detEllipses);
	double twoL_means(std::vector<double> &input_lines, std::vector<int> &_bestLabels, double &center_1, double &center_2);
private:
	// ��ʵ������״����
	float circleDiameter;       //Բֱ������׼Ϊ100cm
	float lineWidth;            //ֱ�߿�ȣ���׼Ϊ10cm
	float circleWidth;          //Բ����ȣ���׼Ϊ10cm
	float dist2Pixel;           //18m�����£� 1cm��Ӧ�����ظ����� Ĭ��Ϊ0.5236px/cm
	
	// ��������Ӧ�Ĳ���
	float minCircleDiameter;    //��Բֱ��
	float maxCircleDiameter;    //��Բֱ��
	//��Ҫ���Ĳ���
	float errCircleDiameter;    //ֱ�����


	float minratioCenter2Line, maxratioCenter2Line;
	float fitMat[2][6];
private:
	////�ж���Բ�Ƿ񳬳��߶ȣ�����Ϊtrue������Ϊfalse
	bool isOutOfRange(const cv::RotatedRect &elp) const
	{
		float aveRadius = (elp.size.height + elp.size.width) / 2;
		if (aveRadius > minCircleDiameter && aveRadius < maxCircleDiameter)
			return false;
		else
			return true;

	}

	bool isInEllipse(int x, int y, int elp_idx) const
	{
		const double *_parm = ellipse_parms[elp_idx].val;
		double val;

		val = _parm[0] * x*x + 2 * _parm[1] * x*y + _parm[2] * y*y + 2 * _parm[3] * x + 2 * _parm[4] * y + _parm[5];
		if (val > 0)
			return false;
		else
			return true;
	}

	bool CONSTRAINT_DIST(const cv::Vec4i &line4, const cv::RotatedRect &fitelp) const
	{
		double minDist(1), maxDist;
		double vecL[4], dist;

		maxDist = std::fminf(fitelp.size.height, fitelp.size.width) / 4;

		vecL[0] = line4[2] - line4[0], vecL[1] = line4[3] - line4[1];
		vecL[2] = fitelp.center.x - line4[0]; vecL[3] = fitelp.center.y - line4[1];

		dist = abs(vecL[0] * vecL[3] - vecL[1] * vecL[2]) / sqrt(vecL[0] * vecL[0] + vecL[1] * vecL[1]);

		if (dist < maxDist && dist > minDist)
			return true;
		else
			return false;

	}
};

#endif