#pragma once

#ifndef SERIALPORT___H

#define SERIALPORT___H

#define TEXT_LENGTH 768

#include <windows.h>



class SerialPort
{
	private:
		HANDLE mSerialPortHandle;
	//	CRITICAL_SECTION mCriticalSectionRead;
	//	CRITICAL_SECTION mCriticalSectionWrite;
	//	int __readBuffer(unsigned char* pData, int dataLength /*, int timeOutMS = -1*/);
	//	int __writeBuffer(const unsigned char* pData, int dataLength);
		char* SerialPort::parseString(const char* start, char* output);
		void configureSerialPort(char** arrayParameter, int arraySize);
		HANDLE SerialPort::openSerialPort();

	public:
		SerialPort();
		void release();
		~SerialPort();
		bool openPort();
		void closePort();
		bool isPortOpen();
		unsigned long writeBuffer(const unsigned char* pData, unsigned long dataLength);
		unsigned long readBuffer(unsigned char * pData, unsigned long dataLength);
	//	int readBuffer(unsigned char* pData, int dataLength /*, int timeOutMS = -1*/);
	//	int writeBuffer(const unsigned char* pData, int dataLength);
	//	int writeBufferNonBlock(const unsigned char* pData, int dataLength);

};
#endif