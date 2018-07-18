#pragma once
#include <opencv2\opencv.hpp>
#include <iostream>

namespace UAV_CONFIG
{
	// Public:
	extern int video_type;
	extern std::string note_video_type;

	extern std::string video_path;
	extern std::string note_video_path;


	bool SaveConfig(std::string filepath);
	bool LoadConfig(std::string filepath);

	// Private:
	


}