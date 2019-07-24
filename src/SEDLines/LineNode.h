#ifndef _LINENODE_H_
#define _LINENODE_H_

#include <opencv2/opencv.hpp>

class LineNode
{
public:
	LineNode();
	LineNode(int x, int y, double scale);

	cv::Point Location;
	LineNode *nextAddress;
	LineNode *lastAddress;
	int edgeID;
	double nodeMat[6];
	double nodesMat[6];

public:
	void nextIs(LineNode *next_dot);// Link to the next point
	void lastIs(LineNode *last_dot);// Link to the last point
	void setFirst();


};

#endif
