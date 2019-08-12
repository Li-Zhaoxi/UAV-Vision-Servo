#ifndef _TEST_FUNCTIONS_H_
#define _TEST_FUNCTIONS_H_


#include <string>
#include <opencv2/opencv.hpp>
#include <Windows.h>

using namespace std;
using namespace cv;


int main_1_0(); // 测试SEDLines在一张图片下的检测结果
int main_1_1(); // 使用一个视频测试SEDLines


int main_2_0(); // 测试YAED在一张图片下的检测结果
int main_2_1(); // 使用一个视频测试YAED算法


int main_3_0(); // 使用一个视频测试RVIBE


int main_4_0(); // 使用一个视频: SEDLines -> C1_FTD
int main_4_1(); // 使用一个图片：直线段检测 -> C1-FTD
int main_4_2(); // 使用一个图片：直线段检测 -> 静态C1-FTD


int main_5_0(); // 使用一个视频：椭圆检测 -> C2-FTD
#endif
