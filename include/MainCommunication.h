#ifndef MAINCOMMUNICATION___H

#define MAINCOMMUNICATION___H

#include "SerialPort.h"
#include "uavdef.h"

#define SEND_PERIOD_MS 200

#define RECEIVE_BYTE_PERIOD_MS 0 //15 //20 //30 //200

#define RECEIVE_SEGMENT_PERIOD_MS 10 // 0 //15

#define RECEIVE_TRY_MS 10 //20// 1000 // 10

#define PACKET_PREAMBLE_BYTE 0xFA

#define PACKET_BID_BYTE 0xFF

#define MID_FLIGHT_MODE 0x70

#define MID_FLIGHT_STATE 0x80

#define MID_OBJECT_INFORMATION 0x64 //100

#define MID_TRACK_INFORMATION 0x65 //101

#define MID_MODE_ACKNOWLEDGE 0x71

//typedef void(*QueryObjectFunction)(float* pPositionX, float* pPositionY, float* pVelocityX, float* pVelocityY, unsigned char* pState);
typedef void(*QueryObjectFunction)(unsigned char* pState,float* pPositionX, float* pPositionY, float* pVelocityX, float* pVelocityY, float* pUseTime /* unsigned char* pState*/);
typedef void(*QueryTrackFunction)(float* pCenterX, float* pCenterY, float* pOrientationX, float* pOrientationY, float* pOrientationZ, unsigned char* pState);
typedef void(*AssignModeFunction)(unsigned char mode);
typedef void(*AssignStateFunction)(unsigned char state,float altitude
								 /*  , float attitudeX, float attitudeY, float attitudeZ*/);

class MainCommunication
{
    private:
		bool mIsQueryAndSendThreadRunning;
		bool mIsReceiveAndAssignThreadRunning;
		SerialPort mSerialPort;
		CRITICAL_SECTION mCriticalSectionWrite;
		QueryObjectFunction mQueryObjectFunction;
		QueryTrackFunction mQueryTrackFunction;
		AssignModeFunction mAssignModeFunction;
		AssignStateFunction mAssignStateFunction;
		unsigned long writeBuffer(const unsigned char* pData, unsigned long dataLength);
		unsigned long sendPackage(unsigned char* pBuffer, unsigned int size);
	//	unsigned char computeCheckSum(unsigned char* inBuffer, int start, int end);
		unsigned long readBuffer(unsigned char* pData, unsigned long dataLength);



   
    public:
		MainCommunication();

		void release();

		~MainCommunication();
		bool initializePipe();
		bool isPipeOpen();
		void finishPipe();
		/*bool*/ // unsigned long sendObjectInformation(float positionX, float positionY, float velocityX, float velocityY, unsigned char state);
        unsigned long sendObjectInformation( unsigned char state,float positionX, float positionY, float velocityX, float velocityY,float useTime);
		unsigned long sendTrackInformation(float centerX, float centerY, float orientationX, float orientationY, float orientationZ, unsigned char state);
		unsigned long sendModeAcknowledge();
		void setQueryObjectFunction(QueryObjectFunction pQueryObjectFunction);
		QueryObjectFunction getQueryObjectFunction();
		void setQueryTrackFunction(QueryTrackFunction pQueryTrackFunction);
		QueryTrackFunction getQueryTrackFunction();
		void startQueryAndSendThread();
		void stopQueryAndSendThread();
		bool isQueryAndSendThreadRunning();
		void setAssignModeFunction(AssignModeFunction pAssignModeFunction);
		AssignModeFunction getAssignModeFunction();
		void setAssignStateFunction(AssignStateFunction pAssignStateFunction);
		AssignStateFunction getAssignStateFunction();
		void startReceiveAndAssignThread();
		void stopReceiveAndAssignThread();
		bool isReceiveAndAssignThreadRunning();
		unsigned long readOneByteWithRetry(unsigned char * pByte);
	//	unsigned long writeBuffer(const unsigned char* pData, unsigned long dataLength);


};


#endif
