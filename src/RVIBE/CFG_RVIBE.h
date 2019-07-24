#ifndef _CFG_RVIBE_H_
#define _CFG_RVIBE_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <uavdef.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;

class UAV_EXPORTS CFG_RVIBE
{
public:
	CFG_RVIBE()
	{
		minAreaRatio = -1;
		maxRect = MIN_MATCHES = NUM_SAMPLES = RADIUS = -1;
		rectWindow = slipeWindow = SUBSAMPLE_FACTOR = -1;
	}
	CFG_RVIBE(string filename){Read_RVIBECONFIG(filename);}
	void Write_RVIBECONFIG(string filename);
	void Read_RVIBECONFIG(string filename);

	int get_NUM_SAMPLES(){return NUM_SAMPLES;}
	int get_MIN_MATCHES(){return MIN_MATCHES;}
	int get_RADIUS(){return RADIUS;}
	int get_SUBSAMPLE_FACTOR(){return SUBSAMPLE_FACTOR;}
	int get_slipeWindow(){return slipeWindow;}
	int get_rectWindow(){return rectWindow;}
	float get_minAreaRatio(){return minAreaRatio;}

	int get_maxRect(){return maxRect;}

private:
	int NUM_SAMPLES;		//ÿ�����ص����������
	int MIN_MATCHES;		//#minָ��
	int RADIUS;		        //Sqthere�뾶
	int SUBSAMPLE_FACTOR;   //�Ӳ�������

	int slipeWindow;
	int rectWindow; 
	float minAreaRatio;
	 
	int maxRect;
};


#endif