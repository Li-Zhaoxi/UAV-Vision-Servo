#pragma  once

#define UAV_WAITING_NOTHING          0x02    //输出参数，无任何修改
#define UAV_CHANGING_RESOLUTION      0x03    //修改分辨率，此帧无效


    // RS: Cross and Circle Detection
#define TARGET_CONFIRM               0x32    // IS: 目标确认
#define UAV_C2_DETECTION_YES         0x31    // SS: Cross has beed detected

#define UAV_C1_DETECTION             0x40    // RS: Cross Detection


#define UAV_EXIT                     0xf0    // RS: Exit Vision Servo


enum ReceiveState
{
	// 主体状态，占用前4位(0000)0000
	UAV_WAITING = 0x10, // RS, IS, SS: 无人机检测程序等待状态
	UAV_RESET = 0x20,    // RS: 无人机参数重置状态
	UAV_C2_DETECTION = 0x30,

	// 附加状态, 占用后4位0000(0000)
	MOTiON_DETECTION = 0x01, // 无人机静止，搜索目标
	UNIFORM_ROI = 0x02,    // 均匀采取搜索目标
};

enum InnerState
{
	CV_TARGET_DETECTION = 0x31,
	CV_TARGET_CONFIRM = 0x32,
};

enum SendState
{
	CV_WAITING = 0x10,
	CV_C1_DETECTION_YES = 0x41,
	CV_C1_DETECTION_NO = 0x42,
};




#define COPY_RESOLUTION_1920_1080  0x10   //原始图片大小
#define NUMBER_1920_1080           1920*1080

#define COPY_RESOLUTION_960_540    0x11   //长宽缩小2倍
#define NUMBER_960_540             960*540

#define COPY_RESOLUTION_480_270    0x12   //长宽缩小4倍
#define NUMBER_480_270             480*270

#define COPY_RESOLUTION_240_135    0x13   //长宽缩小8倍
#define NUMBER_240_135             240*135




#define UAV_CLB_HIGH2PIXEL     1          //标定状态：高度估计像素(px/cm)
#define UAV_CLB_PIXEL2DIST     2          //标定状态：高度估计距离(cm/px)