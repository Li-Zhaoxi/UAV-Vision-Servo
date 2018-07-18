#include "DataTransmission.h"
#include <MainCommunication.h>
#include <definition.h>
#include <iostream>


CRITICAL_SECTION lock;
unsigned char *_RS = NULL;
double *_uav_height = NULL;
void(getStateFunction)(unsigned char state, float altitude
	/*  , float attitudeX, float attitudeY, float attitudeZ*/
	)
{
	EnterCriticalSection(&lock);
	if (_RS != NULL && _uav_height != NULL)
	{
		*_RS = state;
		*_uav_height = altitude;
	}
	LeaveCriticalSection(&lock);
}
MainCommunication *pMainCommunication = NULL;

bool DataTran::OpenSerialPort(unsigned char *_RS, double *_height)
{
	::_RS = _RS;
	::_uav_height = _height;

	InitializeCriticalSection(&lock);//初始化结构CRITICAL_SECTION
	pMainCommunication = new MainCommunication();
	if (pMainCommunication != NULL)
	{
		bool isOpen = pMainCommunication->initializePipe();
		if (isOpen)
		{
			pMainCommunication->setAssignStateFunction(getStateFunction);
			pMainCommunication->startReceiveAndAssignThread();
			return true;
		}
	}
	if (pMainCommunication != NULL)
	{
		
		pMainCommunication->release();
		
		free(pMainCommunication);
		//delete[] pMainCommunication;
		pMainCommunication = NULL;
	}
	return false;
}


void DataTran::SendData2UAV(unsigned char SS, float pX, float pY, float vX, float vY, float useTime)
{
	if (pMainCommunication != NULL)
	{
		pMainCommunication->sendObjectInformation(SS, pX, pY, vX, vY, useTime);
	}
}



float DataTran::pixelError = 8;
float DataTran::highPara_1 = -0.99245958f;
float DataTran::highPara_2 = 2.22146648f;

float DataTran::carHighValue = 1.7f;
float DataTran::cross_Length = 10;

int DataTran::confirmNum = 0;
int DataTran::confirmedNum = 0;


float DataTran::calPixcelAndDist(float inValue, float Para_1, float Para_2, unsigned char calType)
{
	if(calType == UAV_CLB_HIGH2PIXEL)
	{
		return pow(inValue, Para_1) * exp(Para_2);
	}
	else if(calType == UAV_CLB_PIXEL2DIST)
	{
		return pow(inValue, -Para_1) * exp(-Para_2);
	}
	return -1;
}

float DataTran::calPixcelAndDist(float inValue, unsigned char calType)
{
	if (calType == UAV_CLB_HIGH2PIXEL)
	{
		return pow(inValue, DataTran::highPara_1) * exp(DataTran::highPara_2);
	}
	else if (calType == UAV_CLB_PIXEL2DIST)
	{
		return pow(inValue, -DataTran::highPara_1) * exp(-DataTran::highPara_2);
	}
	return -1;
}