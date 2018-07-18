#include "SerialPort.h"
#include <stdio.h>
//#include <stdlib.h>



//#define SERIALPORT_INTERNAL__TIMEOUT 1



SerialPort::SerialPort()
{
	mSerialPortHandle = INVALID_HANDLE_VALUE;
	//	InitializeCriticalSection(&mCriticalSectionRead);
	//	InitializeCriticalSection(&mCriticalSectionWrite);
}


void SerialPort::release()
{
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		closePort();
	}
}
SerialPort::~SerialPort()
{
	//if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	//{
	//	closePort();
	//}

//	DeleteCriticalSection(&mCriticalSectionRead);
//	DeleteCriticalSection(&mCriticalSectionWrite);
}

char* SerialPort::parseString(const char* start, char* output) {
	int wordLength;
	const char* end = NULL;
	char* lap = NULL;
	//com port
	end = strchr(start, '\n');
	if (end == NULL) {
		end = start + strlen(start);
	}
	wordLength = int(end - start);
	if (wordLength > 0) {
		lap=(char*)strchr(start, '=');
		if ((lap == NULL) || (lap >= end)) {
			lap = (char *)start;
		}
		else {
			lap = lap + 1;
			
		}
		//delete space
		while ((*lap) == ' ') {
			lap++;
		}
		wordLength = int(end - lap);
		//delete '\r' in window style
		if (*(end - 1) == '\r') {
			wordLength--;
		}
		//delete space
		while (*(lap + wordLength - 1) == ' ') {
			wordLength--;
		}
	//	memcpy(output, start, sizeof(char)*wordLength);
		memcpy(output, lap, sizeof(char)*wordLength);
		*(output + wordLength) = 0;
	}
	return (char*)end;
}

void SerialPort::configureSerialPort(char** arrayParameter,int arraySize) {
	HANDLE hConfig;
	DWORD bytesRead = 0;
	char lpBuffer[TEXT_LENGTH] = "235";//文件读取的内容 
	hConfig = CreateFileA("Config\\serial.txt",//"\\\\.\\COM3",            //port name 

		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 

		0,            // No Sharing                               

		NULL,         // No Security                              
					  //	CREATE_ALWAYS, //
		OPEN_EXISTING,// Open existing port only                     

		0,            // Non Overlapped I/O                           

		NULL);        // Null for Comm Devices

	if (hConfig == INVALID_HANDLE_VALUE)
	{
	//	printf("Error in opening configuration");
	}
	else {
		int fileOpen = ReadFile// WriteFile//
		(
			hConfig,
			lpBuffer,
			TEXT_LENGTH,//读取文件中多少内容 
			&bytesRead,
			NULL
		);

		if (fileOpen == 0)
		{
		//	printf("Error in reading configuration");
		}
		else {
		//	printf("bytesRead=%d\n", bytesRead);
			char* start = lpBuffer;
			char* tail;
			for (int index = 0;index <arraySize;index++) {
				tail = parseString((const char*)start,arrayParameter[index]);
				if (tail != (start + strlen(start))) {
					start = tail + 1;
				}
				else {
					break;
				}
			}

		}
	};
	CloseHandle(hConfig);

}

HANDLE SerialPort::openSerialPort() {
	HANDLE hComm = NULL;

	char comPort[6] = "COM3";
	char sizeInputBuffer[8] = "1024";//"64";//"1024";
	char sizeOutputBuffer[8] = "1024";//"64";//"1024";
	char baudRate[7] = "9600";
	char bitValid[3] = "8";
	char checkOption[11] = "NOPARITY";
	char stopOption[13] = "TWOSTOPBITS";

	char readIntervalTimeout[8] = "100";
	char readTotalTimeoutMultiplier[8] = "50";
	char readTotalTimeoutConstant[8] = "5000";
	char writeTotalTimeoutMultiplier[8] = "0";
	char writeTotalTimeoutConstant[8] = "0";

	char* arrayParameter[] = { comPort,sizeInputBuffer,sizeOutputBuffer,baudRate,bitValid,checkOption,stopOption,
		readIntervalTimeout,readTotalTimeoutMultiplier,readTotalTimeoutConstant,writeTotalTimeoutMultiplier,writeTotalTimeoutConstant
	};

	configureSerialPort(arrayParameter,sizeof(arrayParameter)/sizeof(char*));

	hComm = CreateFileA(comPort,//"\\\\.\\COM3",            //port name 

		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 

		0,            // No Sharing                               

		NULL,         // No Security                              

		OPEN_EXISTING,// Open existing port only                     

		0,            // Non Overlapped I/O                           

		NULL);        // Null for Comm Devices




	//configure
	SetupComm(hComm, atoi(sizeInputBuffer), atoi(sizeOutputBuffer)); //输入缓冲区和输出缓冲区的大小都是1024

	COMMTIMEOUTS TimeOuts;
	//设定读超时
	TimeOuts.ReadIntervalTimeout = atoi(readIntervalTimeout);// 90; // 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = atoi(readTotalTimeoutMultiplier);//50; // 500;
	TimeOuts.ReadTotalTimeoutConstant = atoi(readTotalTimeoutConstant);//5000;
	//设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = atoi(writeTotalTimeoutMultiplier);//0;// -1; // 500;
	TimeOuts.WriteTotalTimeoutConstant = atoi(writeTotalTimeoutConstant);//0;//-1; //2000;
	SetCommTimeouts(hComm, &TimeOuts); //设置超时

	DCB dcb;
	GetCommState(hComm, &dcb);
	dcb.BaudRate = atoi(baudRate); //波特率为9600
	dcb.ByteSize = atoi(bitValid); //每个字节有8位
	if (!strcmp(checkOption, "NOPARITY")) {
		dcb.Parity = NOPARITY; //无奇偶校验位
	}
	else if (!strcmp(checkOption, "ODDPARITY")) {
		dcb.Parity = ODDPARITY; //奇校验位
	}
	else if (!strcmp(checkOption, "EVENPARITY")) {
		dcb.Parity = EVENPARITY; //偶校验位
	}
	else if (!strcmp(checkOption, "MARKPARITY")) {
		dcb.Parity = MARKPARITY; //奇校验位
	}

	if (!strcmp(stopOption, "ONESTOPBIT")) {
		dcb.StopBits = ONESTOPBIT; //一个停止位
	}
	else if (!strcmp(stopOption, "TWOSTOPBITS")) {
		dcb.StopBits = TWOSTOPBITS; //两个停止位
	}
	else if (!strcmp(stopOption, "ONE5STOPBITS")) {
		dcb.StopBits = ONE5STOPBITS; //一个半停止位
	}
	SetCommState(hComm, &dcb);

	PurgeComm(hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);

	return hComm;
}

bool SerialPort::openPort()
{
	bool isOpen = false;
	if (mSerialPortHandle == INVALID_HANDLE_VALUE) {
		mSerialPortHandle = openSerialPort();
	}
	if (mSerialPortHandle != INVALID_HANDLE_VALUE) {
		isOpen = true;
	}
	return isOpen;
}

void SerialPort::closePort()
{
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		CloseHandle(mSerialPortHandle);
		mSerialPortHandle = INVALID_HANDLE_VALUE;
	}
}

bool SerialPort::isPortOpen() {
	bool isOpen = false;
	if (mSerialPortHandle != INVALID_HANDLE_VALUE) {
		isOpen = true;
	}
	return isOpen;
}

unsigned long SerialPort::writeBuffer(const unsigned char* pData, unsigned long dataLength)
{
	
//	unsigned long result = 0;
	DWORD bytesWritten = 0;
//	long tickcount1, tickcount2 ;
	
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
	//	WriteComm();
	//	OpenComm(
		//tickcount1 = GetTickCount();
		if (!WriteFile(mSerialPortHandle, pData, dataLength, &bytesWritten, NULL))
		{
			bytesWritten = 0;
		}
		//tickcount2 = GetTickCount();
		//printf("serial port write begin time:%ld\n", tickcount1);
	 //   printf("serial port write end time:%ld\n", tickcount2);
		//printf("serial port write elapse time:%ld\n",tickcount2-tickcount1);
	}
	return bytesWritten;

}

unsigned long SerialPort::readBuffer(unsigned char* pData, unsigned long dataLength)
{
	//	unsigned long result = 0;
	DWORD bytesRead = 0;
	if (mSerialPortHandle != INVALID_HANDLE_VALUE)
	{
		//	WriteComm();
		//	OpenComm(
	//	printf("serial port read begin time:%ld\n", GetTickCount());
		if (!ReadFile(mSerialPortHandle, pData, dataLength, &bytesRead, NULL))
		{
			bytesRead = 0;
		}
   //		printf("serial port read end time:%ld\n", GetTickCount());
	}
	return bytesRead;

}

