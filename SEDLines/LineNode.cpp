#include "LineNode.h"


LineNode::LineNode()
{
	nextAddress = NULL;
	lastAddress = NULL;
}

LineNode::LineNode(int x, int y, double scale)
{
	Location.x = x; Location.y = y;
	edgeID = 1;
	nextAddress = NULL;
	lastAddress = NULL;

	nodeMat[0] = double(x)*double(x) / (scale*scale);
	nodeMat[1] = double(x)*double(y) / (scale*scale);
	nodeMat[2] = double(x) / scale;
	nodeMat[3] = double(y)*double(y) / (scale*scale);
	nodeMat[4] = double(y) / scale;
	nodeMat[5] = 1;
}


void LineNode::nextIs(LineNode *next_dot)
{
	if (next_dot != NULL)
	{
		nextAddress = next_dot;
		next_dot->lastAddress = this;
		next_dot->edgeID = this->edgeID + 1;
		for (int k = 0; k < 6; k++)
			next_dot->nodesMat[k] = nodesMat[k] + next_dot->nodeMat[k];
	}
	else
		nextAddress = NULL;
}

void LineNode::lastIs(LineNode *last_dot)
{
	if (last_dot != NULL)
	{
		lastAddress = last_dot;
		last_dot->nextAddress = this;
		last_dot->edgeID = this->edgeID - 1;
		for (int k = 0; k < 6; k++)
			last_dot->nodesMat[k] = this->nodesMat[k] - last_dot->nodeMat[k];
	}
	else
		lastAddress = NULL;
}

void LineNode::setFirst()
{
	lastAddress = NULL;
	nextAddress = NULL;
	edgeID = 1;
	for (int k = 0; k < 6; k++)
		nodesMat[k] = nodeMat[k];
}