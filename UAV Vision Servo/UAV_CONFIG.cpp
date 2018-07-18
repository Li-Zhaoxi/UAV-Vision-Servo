#include "UAV_CONFIG.h"

namespace UAV_CONFIG
{
	int video_type = 1; //默认从文件读
	std::string note_video_type = "video_type: 视频流类型，1从文件读，2选择云台";

	std::string video_path = "E:\\无人机比赛论文\\视频数据\\真实-FLETD-3.mp4";
	std::string note_video_path = "video_path: 视频文件路径";

	bool SaveConfig(std::string filepath)
	{
		cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
		if (!fs.isOpened())
			return false;

		cvWriteComment(fs.fs, note_video_type.c_str(), 0);
		fs << "video_type" << (int)video_type;

		cvWriteComment(fs.fs, note_video_path.c_str(), 0);
		fs << "video_path" << (std::string)video_path;

		fs.release();

		return true;
	}

	bool LoadConfig(std::string filepath)
	{
		cv::FileStorage fs(filepath, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;

		video_type = (int)fs["video_type"];
		video_path = (std::string)fs["video_path"];

		fs.release();
		return true;
	}
}