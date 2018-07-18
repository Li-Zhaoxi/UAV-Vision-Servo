#pragma once
#include <uavdef.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using std::string;
using std::cout;
using std::endl;
using std::vector;

class UAV_EXPORTS CFG_RVIBE
{
public:
	CFG_RVIBE(){}
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
	int NUM_SAMPLES;		//每个像素点的样本个数
	int MIN_MATCHES;		//#min指数
	int RADIUS;		        //Sqthere半径
	int SUBSAMPLE_FACTOR;   //子采样概率

	int slipeWindow;
	int rectWindow; 
	float minAreaRatio;
	 
	int maxRect;
};