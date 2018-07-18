#include <logfiles_io.h>
#include <Windows.h>
#include <string>
#include <direct.h>
#include <fstream>

using std::string;

std::ofstream rvibe, yaed, sedlines, c1_ftd, c2_ftd, uav_RS, uav_SS;


bool UAV_IO::InitLogFiles(bool openRVIBE, bool openYAED, bool openSEDLines, bool openC1_FTD, bool openC2_FTD)
{
	std::string res;
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char temp[100];
	_itoa_s(sys.wYear, temp, 10);//Year
	res = res + string(temp) + "-";
	_itoa_s(sys.wMonth, temp, 10);//Month
	res = res + string(temp) + "-";
	_itoa_s(sys.wDay, temp, 10);//Day
	res = res + string(temp) + "-";
	_itoa_s(sys.wHour, temp, 10);//Hour
	res = res + string(temp) + "-";
	_itoa_s(sys.wMinute, temp, 10);//wMinute
	res = res + string(temp) + "-";
	_itoa_s(sys.wSecond, temp, 10);//Second
	res = res + string(temp);

	string pathDataFile = "Data", pathImageFile = "Data\\Image", pathstateTypeFile = "Data\\stateValue";

	pathDataFile = "Data\\" + res;
	int pInfor1 = _mkdir(pathDataFile.c_str());
	pathImageFile = "Data\\" + res + "\\Image";
	int pInfor2 = _mkdir(pathImageFile.c_str());
	pathstateTypeFile = "Data\\" + res + "\\stateValue";
	int pInfor3 = _mkdir(pathstateTypeFile.c_str());
	if (pInfor1 || pInfor2 || pInfor3)
		return false;


	uav_RS.open(pathstateTypeFile + "\\ReceiveState.uav", std::ios::out | std::ios::app | std::ios::binary);
	uav_SS.open(pathstateTypeFile + "\\SendState.uav", std::ios::out | std::ios::app | std::ios::binary);

	if (openRVIBE)
	{
		rvibe.open(pathstateTypeFile + "\\RVIBE.uav", std::ios::out | std::ios::app | std::ios::binary);
	}
	if (openYAED)
	{
		yaed.open(pathstateTypeFile + "\\YAED.uav", std::ios::out | std::ios::app | std::ios::binary);
	}
	if (openSEDLines)
	{
		sedlines.open(pathstateTypeFile + "\\SEDLines.uav", std::ios::out | std::ios::app | std::ios::binary);
	}
	if (openC1_FTD)
	{
		c1_ftd.open(pathstateTypeFile + "\\C1-FTD.uav", std::ios::out | std::ios::app | std::ios::binary);
	}
	if (openC2_FTD)
	{
		c2_ftd.open(pathstateTypeFile + "\\C2-FTD.uav", std::ios::out | std::ios::app | std::ios::binary);
	}
	return true;
}

void UAV_IO::ReleaseLogFiles()
{
	rvibe.close();
	yaed.close();
	sedlines.close();
	c1_ftd.close();
	c2_ftd.close();

	uav_RS.close();
	uav_SS.close();
}

void UAV_IO::SaveRVIBE(int idNum, std::vector<cv::Rect> &findRect)
{
	rvibe.write((char*)&idNum, sizeof(int));
	int resNum = findRect.size();
	rvibe.write((char*)&resNum, sizeof(int));
	rvibe.write((char*)findRect.data(), sizeof(cv::Rect)*resNum);
}

void UAV_IO::SaveYAED(int idNum, std::vector<cv::RotatedRect> &detEllipses)
{
	yaed.write((char*)&idNum, sizeof(int));
	int resNum = detEllipses.size();
	yaed.write((char*)&resNum, sizeof(int));
	yaed.write((char*)detEllipses.data(), sizeof(cv::RotatedRect)*resNum);
}

void UAV_IO::SaveSEDLines(int idNum, std::vector<cv::Vec4i> &LineSegments)
{
	sedlines.write((char*)&idNum, sizeof(int));
	int resNum = LineSegments.size();
	sedlines.write((char*)&resNum, sizeof(int));
	sedlines.write((char*)LineSegments.data(), sizeof(cv::Vec4i)*resNum);
}

void UAV_IO::SaveC1_FTD(int idNum, char isFind, cv::Point2f crossCenter, cv::Point2f cross4Points[4])
{
	c1_ftd.write((char*)&idNum, sizeof(int));
	c1_ftd.write((char*)&isFind, sizeof(char));
	c1_ftd.write((char*)&crossCenter, sizeof(cv::Point2f));
	c1_ftd.write((char*)cross4Points, sizeof(cv::Point2f) * 4);
}

void UAV_IO::SaveC2_FTD(int idNum, char isFind, cv::RotatedRect &landEllipse, cv::Vec4f landCross)
{
	c2_ftd.write((char*)&idNum, sizeof(int));
	c2_ftd.write((char*)&isFind, sizeof(char));
	c2_ftd.write((char*)&landEllipse, sizeof(cv::RotatedRect));
	c2_ftd.write((char*)&landCross, sizeof(cv::Vec4f));
}


void UAV_IO::SaveRS(int idNum, unsigned char RS, double uav_height)
{
	uav_RS.write((char*)&idNum, sizeof(int));
	uav_RS.write((char*)&RS, sizeof(char));
	uav_RS.write((char*)&uav_height, sizeof(double));
}

void UAV_IO::SaveSS(int idNum, unsigned char SS, double data[4], double pipeline_time)
{
	uav_RS.write((char*)&idNum, sizeof(int));
	uav_RS.write((char*)&SS, sizeof(char));
	uav_RS.write((char*)data, sizeof(double) * 4);
	uav_RS.write((char*)&pipeline_time, sizeof(double));
}