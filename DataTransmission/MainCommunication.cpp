#include "MainCommunication.h"
//#include <iostream>
//#include <iomanip>
//#include <iostream>
#include <stdio.h>
struct ObjectMessage {
//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char state : 8;
	float positionX;
	float positionY;
	float velocityX;
	float velocityY;
//	unsigned char state : 8;
//	unsigned char reserved[7];
	float useTime;
	unsigned char reserved[3];
	unsigned char checkSum;
};

struct TrackMessage {
	//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	float centerX;
	float centerY;
	float orientationX;
	float orientationY;
	float orientationZ;
	unsigned char state : 8;
	unsigned char reserved[3];
	unsigned char checkSum;
};

struct ModeAcknowledgeMessage {
	//	ObjectMessage() : preamble(0xFA), bid(0xFF), mid(100), len(24), reserved({ 0,0,0,0,0,0 }) {}
	unsigned char preamble : 8;
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char checkSum;
};

struct ModeMessage {
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char mode : 8;
};

struct StateMessage {
	unsigned char bid : 8;
	unsigned char mid : 8;
	unsigned char len : 8;
	unsigned char state;
	float altitude;
//	float attitudeX;
//	float attitudeY;
//	float attitudeZ;
};

/*
float convertFloatEndian(float in) {
	unsigned char* byte = reinterpret_cast<unsigned char*>(&in);
	unsigned char swap = (*byte);
	(*byte) = *(byte+3);
	*(byte+3)=swap;
	swap = *(byte+1);
	*(byte+1) = *(byte+2);
	*(byte+2) = swap;
    return in;
}
*/

template<typename T> T convertEndian(T in) {
	unsigned char* buffer = reinterpret_cast<unsigned char*>(&in);
	unsigned char length = sizeof(T);
	unsigned char store;
	for (int index = 0;index < (length / 2);index++) {
		store = *(buffer + index);
		*(buffer + index) = *(buffer + length - 1 - index);
		*(buffer + length - 1 - index) = store;
	}
	return in;
}


unsigned char /*MainCommunication::*/computeCheckSum(unsigned char * inBuffer, int start, int end)
{
	unsigned int sum = 0;
	unsigned char result;
	for (int index = start; index <= end; index++) {
		sum += *(inBuffer + index);
		sum = sum & (0xFF);
	}
	result = (unsigned char)sum;
	result = (~result) + 1;
	return result;
}

DWORD CALLBACK ReceiveThreadProc(PVOID pvoid)
{
	unsigned char byte;
	int phase = 0;
	int stage = 0;
	int count = 0;
	unsigned char checkSum;
	unsigned char mid;
	unsigned char length;
	unsigned char* pPacket = NULL;
	AssignModeFunction pAssignMode = NULL;
	AssignStateFunction pAssignState = NULL;
	struct ModeMessage* pMode = NULL;
	struct StateMessage* pState = NULL;
	MainCommunication* pThis = (MainCommunication*)pvoid;
	while (pThis->isReceiveAndAssignThreadRunning()) {
		if (pThis->isPipeOpen()) {
			if (pThis->readOneByteWithRetry(&byte)>0) {
		//		std::cout <<"hex : " << hex <<byte <<endl ;  
	     		//printf("%x\n",byte);
				switch (phase) {
				    case 0:
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						break;
					case 1:
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						else if (byte == PACKET_BID_BYTE) {
							phase = 2;
						}
						else {
							phase = 0;
						}
						break;
					case 2:
						if (byte == PACKET_PREAMBLE_BYTE) {
							phase = 1;
						}
						else if ((byte == MID_FLIGHT_MODE) || (byte == MID_FLIGHT_STATE)) {
							phase = 3;
							mid = byte;
							stage = 0;
						}
						else {
							phase = 0;
						}
						break;
					case 3:
						switch (stage) {
						    case 0:
								length = byte;
								pPacket = new unsigned char[length+3];
								*pPacket = PACKET_BID_BYTE;
								*(pPacket + 1) = mid;
								*(pPacket + 2) = length;
								count = 0;
								stage = 1;
								break;

							case 1:
								*(pPacket + 3 + (count++)) = byte;
								if (count == length) {
									stage = 2;
									checkSum = computeCheckSum(pPacket, 0, length + 2);
								}
								break;

							case 2:
								if (checkSum == byte) {
									//package is correct
									//to do callback
									switch (mid) {
									    case MID_FLIGHT_MODE:
											pAssignMode = pThis->getAssignModeFunction();
											if (pAssignMode != NULL) {
												pMode = reinterpret_cast<struct ModeMessage*>(pPacket);
												pAssignMode(pMode->mode);
											}
											pThis->sendModeAcknowledge();
										    break;

										case MID_FLIGHT_STATE:
											pAssignState = pThis->getAssignStateFunction();
											if (pAssignState != NULL) {
												pState = reinterpret_cast<struct StateMessage*>(pPacket);
												pAssignState(pState->state,
												//	convertEndian<float>(
													pState->altitude
												//	)
												//	,pState->attitudeX,pState->attitudeY,pState->attitudeZ
													);
											}
											break;
									}
									Sleep(RECEIVE_SEGMENT_PERIOD_MS);

								}
								if (pPacket != NULL) {
									delete pPacket;
									pPacket = NULL;
								}
								phase = 0;
								stage = 0;
								break;
						}
						break;
				}

			}
			else {
				break;
			}
		}
		else {
			break;
		}
		Sleep(RECEIVE_BYTE_PERIOD_MS);
	}
	
	pThis->stopReceiveAndAssignThread();
	return 0;
}

DWORD CALLBACK SendThreadProc(PVOID pvoid)
{
	MainCommunication* pThis = (MainCommunication*) pvoid;
	QueryObjectFunction fQueryObject=NULL;
	QueryTrackFunction fQueryTrack = NULL;
	while (pThis->isQueryAndSendThreadRunning()) {
		fQueryObject = pThis->getQueryObjectFunction();
		if (fQueryObject != NULL) {
			float positionX, positionY, velocityX, velocityY,useTime;
			unsigned char state;
			fQueryObject(&state,&positionX, &positionY, &velocityX, &velocityY, &useTime /*&state*/);
//			printf("thread fqueryobject positionX=%f, positionY=%f \n", positionX,positionY);
			pThis->sendObjectInformation(state,positionX, positionY, velocityX, velocityY,useTime/* state*/);
		}
		fQueryTrack = pThis->getQueryTrackFunction();
		if (fQueryTrack != NULL) {
			float centerX, centerY, orientationX, orientationY, orientationZ;
			unsigned char state;
			fQueryTrack(&centerX, &centerY, &orientationX, &orientationY, &orientationZ, &state);
//			printf("thread fqueryobject centerX=%f, centerY=%f \n", centerX, centerY);
			pThis->sendTrackInformation(centerX, centerY, orientationX, orientationY, orientationZ, state);
		}
		Sleep(SEND_PERIOD_MS);
	}
	return 0;
}



unsigned long MainCommunication::writeBuffer(const unsigned char * pData, unsigned long dataLength)
{
	unsigned long sentSize = 0;
//	EnterCriticalSection(&mCriticalSectionWrite);
	sentSize = mSerialPort.writeBuffer(pData, dataLength);
//	LeaveCriticalSection(&mCriticalSectionWrite);
	//	sentSize = sizeof(struct ObjectMessage);
	return sentSize;
}

unsigned long MainCommunication::sendPackage(unsigned char *pBuffer,unsigned int size) {
	unsigned long sentSize = 0;
	unsigned char checkSum = computeCheckSum(pBuffer, 1, size - 2);
	*(pBuffer+size-1) = checkSum;
	sentSize = writeBuffer(pBuffer, sizeof(struct ObjectMessage));
	return sentSize;
}



unsigned long MainCommunication::readBuffer(unsigned char * pData, unsigned long dataLength)
{
	unsigned long readSize = 0;
//	EnterCriticalSection(&mCriticalSectionWrite);
	readSize = mSerialPort.readBuffer(pData, dataLength);
//	LeaveCriticalSection(&mCriticalSectionWrite);
	//	sentSize = sizeof(struct ObjectMessage);
	return readSize;
	
}

MainCommunication::MainCommunication()
	: mIsQueryAndSendThreadRunning(false),mIsReceiveAndAssignThreadRunning(false),
	mQueryObjectFunction(NULL),mQueryTrackFunction(NULL),
	mAssignModeFunction(NULL), mAssignStateFunction(NULL)
{
//	mQueryObjectFunction = NULL;
	InitializeCriticalSection(&mCriticalSectionWrite);

}

MainCommunication::~MainCommunication()
{
	
}

bool MainCommunication::initializePipe()
{
	return mSerialPort.openPort();
//	return false;
}

bool MainCommunication::isPipeOpen()
{
	return mSerialPort.isPortOpen();
}

void MainCommunication::finishPipe()
{
	EnterCriticalSection(&mCriticalSectionWrite); //prevents port closing if WriteBuffer is not complete
	mSerialPort.closePort();
	LeaveCriticalSection(&mCriticalSectionWrite);
}


/*bool*/// unsigned long MainCommunication::sendObjectInformation(float positionX, float positionY, float velocityX, float velocityY, unsigned char state)
 unsigned long MainCommunication::sendObjectInformation(unsigned char state,float positionX, float positionY, float velocityX, float velocityY,float useTime )
{
	bool flag = true;
	unsigned long sentSize = 0;
	struct ObjectMessage objectMessage = {
//		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_OBJECT_INFORMATION, 24, convertFloatEndian(positionX),convertFloatEndian(positionY),convertFloatEndian(velocityX),convertFloatEndian(velocityY),state,{ 0,0,0,0,0,0,0 },0
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_OBJECT_INFORMATION, 24, state,convertEndian<float>(positionX),convertEndian<float>(positionY),convertEndian<float>(velocityX),convertEndian<float>(velocityY),convertEndian<float>(useTime),{0,0,0}/*state,{ 0,0,0,0,0,0,0 }*/,0
	};

	//	objectMessage.positionX = positionX;
	//	objectMessage.positionY = positionY;
	//	objectMessage.velocityX = velocityX;
	//	objectMessage.velocityY = velocityY;
	//	objectMessage.state = state;

	unsigned char *p = reinterpret_cast<unsigned char *>(&objectMessage);
	sentSize = sendPackage(p, sizeof(struct ObjectMessage));

	//for(int i=0;i<sizeof(struct ObjectMessage);i++) {
	//	printf("%x ",*(p+i));
	//}
	//printf("\n");
	//	unsigned char checkSum = computeCheckSum(p, 1, sizeof(struct ObjectMessage)-2);
	//	objectMessage.checkSum = checkSum;
	//	sentSize = writeBuffer(p,sizeof(struct ObjectMessage));
	if (sentSize == 0) {
		flag = false;
	}
	return sentSize;
	//return flag;
}

/*bool*/ unsigned long MainCommunication::sendTrackInformation(float centerX, float centerY, float orientationX, float orientationY, float orientationZ, unsigned char state)
{
	bool flag = true;
	unsigned long sentSize = 0;
	struct TrackMessage trackMessage = {
	//	PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_TRACK_INFORMATION, 24, convertFloatEndian(centerX),convertFloatEndian(centerY),convertFloatEndian(orientationX),convertFloatEndian(orientationY),convertFloatEndian(orientationZ),state,{ 0,0,0 },0
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_TRACK_INFORMATION, 24, convertEndian<float>(centerX),convertEndian<float>(centerY),convertEndian<float>(orientationX),convertEndian<float>(orientationY),convertEndian<float>(orientationZ),state,{ 0,0,0 },0
	};


	unsigned char *p = reinterpret_cast<unsigned char *>(&trackMessage);
	sentSize = sendPackage(p, sizeof(struct TrackMessage));

	
	if (sentSize == 0) {
		flag = false;
	}
	return sentSize;
	//return flag;
}

unsigned long MainCommunication::sendModeAcknowledge()
{
	struct ModeAcknowledgeMessage modeAcknowledgeMessage = {
		PACKET_PREAMBLE_BYTE, PACKET_BID_BYTE, MID_MODE_ACKNOWLEDGE, 0,0
	};
	unsigned char *p = reinterpret_cast<unsigned char *>(&modeAcknowledgeMessage);
	unsigned long sentSize = sendPackage(p, sizeof(struct ModeAcknowledgeMessage));

	return sentSize;
}

void MainCommunication::setQueryObjectFunction(QueryObjectFunction pQueryObjectFunction)
{
	mQueryObjectFunction = pQueryObjectFunction;
}

QueryObjectFunction MainCommunication::getQueryObjectFunction()
{
	return mQueryObjectFunction;
}

void MainCommunication::setQueryTrackFunction(QueryTrackFunction pQueryTrackFunction)
{
	mQueryTrackFunction = pQueryTrackFunction;
}

QueryTrackFunction MainCommunication::getQueryTrackFunction()
{
	return mQueryTrackFunction;
}

void MainCommunication::startQueryAndSendThread()
{
	mIsQueryAndSendThreadRunning = true;
	DWORD dwThreadId;
	//	HANDLE hThread = CreateThread(NULL, 0, ThreadProc,  /* cannot far call TimerFunction TimerFunction */ TimeProc, 0, &dwThreadId);
	//	MMRESULT nIDTimerEvent = timeSetEvent(1000, 0, MMTimeProc, 0, (UINT)TIME_PERIODIC);
	HANDLE hThread = CreateThread(NULL, 0, SendThreadProc, this, 0, &dwThreadId);
	CloseHandle(hThread);
}

void MainCommunication::stopQueryAndSendThread()
{
	mIsQueryAndSendThreadRunning = false;
}

bool MainCommunication::isQueryAndSendThreadRunning()
{
	return mIsQueryAndSendThreadRunning;
}

void MainCommunication::setAssignModeFunction(AssignModeFunction pAssignModeFunction)
{
	mAssignModeFunction = pAssignModeFunction;
}

AssignModeFunction MainCommunication::getAssignModeFunction()
{
	return mAssignModeFunction;
}

void MainCommunication::setAssignStateFunction(AssignStateFunction pAssignStateFunction)
{
	mAssignStateFunction = pAssignStateFunction;
}

AssignStateFunction MainCommunication::getAssignStateFunction()
{
	return mAssignStateFunction;
}

void MainCommunication::startReceiveAndAssignThread()
{
	mIsReceiveAndAssignThreadRunning = true;
	DWORD dwThreadId;
	HANDLE hThread = CreateThread(NULL, 0, ReceiveThreadProc, this, 0, &dwThreadId);
	CloseHandle(hThread);
}

void MainCommunication::stopReceiveAndAssignThread()
{
	mIsReceiveAndAssignThreadRunning = false;
}

bool MainCommunication::isReceiveAndAssignThreadRunning()
{
	return mIsReceiveAndAssignThreadRunning;
}

unsigned long MainCommunication::readOneByteWithRetry(unsigned char* pByte) {
	unsigned long readCount=0;
	//	EnterCriticalSection(&mCriticalSectionWrite);
	while ( isReceiveAndAssignThreadRunning()) {
		readCount=readBuffer(pByte,1);
		if (readCount == 0) {
			Sleep(RECEIVE_TRY_MS);
		}
		else {
			break;
		}
	}
	return readCount;
	// LeaveCriticalSection(&mCriticalSectionWrite);
}


void MainCommunication::release()
{
	if (mSerialPort.isPortOpen()) {
		finishPipe();
	}
	mIsQueryAndSendThreadRunning = false;
	mIsReceiveAndAssignThreadRunning = false;
	mQueryObjectFunction = NULL;
	mQueryTrackFunction = NULL;
	mAssignModeFunction = NULL;
	mAssignStateFunction = NULL;
	DeleteCriticalSection(&mCriticalSectionWrite);


	mSerialPort.release();
}