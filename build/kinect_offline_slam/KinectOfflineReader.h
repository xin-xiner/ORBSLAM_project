#include "fileOperate.h"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

class KinectOfflineReader
{
	std::vector<std::string> rgb_file_list_;
	std::vector<std::string> depth_file_list_;
	std::string path_;
	int frame_id_;
	int list_size_;
public:
	KinectOfflineReader(std::string path)
		:path_(path), frame_id_(0)
	{
		getImagesList(path_ + "\\rgb\\", rgb_file_list_);
		getImagesList(path_ + "\\depth\\", depth_file_list_);
		list_size_ = rgb_file_list_.size();
		std::cout << "file size: " << list_size_ << std::endl;
	}

	bool getNextFrame(cv::Mat& depth, cv::Mat& rgb)
	{
		if (frame_id_ >= list_size_)
			return false;
		depth = cv::imread(path_ + "\\depth\\"+depth_file_list_[frame_id_], -1);
		rgb = cv::imread(path_ + "\\rgb\\"+rgb_file_list_[frame_id_], -1);
		if (depth.empty() || rgb.empty())
			return false;
		frame_id_++;
		return true;
	}

	int getFrameID()
	{
		std::stringstream sst;
		sst << rgb_file_list_[frame_id_ - 1].substr(0,7);
		int  id;
		sst >> id;
		return frame_id_-1;
	}

};