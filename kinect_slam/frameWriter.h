#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <deque>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <Windows.h>
class FrameQueue
{
	class Frame
	{
	public:
		cv::Mat rgb_image_;
		cv::Mat depth_image_;
		double time_stamp_;
		Frame(cv::Mat rgb_image, cv::Mat depth_image, double time_stamp)
			:rgb_image_(rgb_image), depth_image_(depth_image), time_stamp_(time_stamp)
		{}
	};

	std::deque<Frame> frame_queue_;
	std::mutex frame_queue_mutex_;

	std::string record_path_;

	bool isQuit = false;

public:

	FrameQueue(std::string record_path)
		:record_path_(record_path)
	{
	}
	FrameQueue(FrameQueue& frame_queue)
	{
		frame_queue_ = frame_queue.frame_queue_;
		record_path_ = frame_queue.record_path_;
		frame_queue_mutex_.unlock();
	}
	~FrameQueue()
	{
		frame_queue_mutex_.unlock();
		close();
	}


	void pushBack(cv::Mat rgb_image, cv::Mat depth_image, double time_stamp)
	{
		frame_queue_mutex_.lock();
		frame_queue_.push_back(Frame(rgb_image, depth_image, time_stamp));
		frame_queue_mutex_.unlock();
	}



	void writeThread()
	{
		std::stringstream sst;
		while (isQuit != true)
		{
			if (!frame_queue_.empty())
			{
				Frame current_frame = frame_queue_.front();
				frame_queue_mutex_.lock();
				frame_queue_.pop_front();
				frame_queue_mutex_.unlock();
				sst.str(""); sst.clear();
				sst << record_path_<<"\\"<<"rgb\\" << std::setw(7) << std::setfill('0') << (long long)current_frame.time_stamp_<<".png";
				cv::imwrite(sst.str(), current_frame.rgb_image_);
				sst.str(""); sst.clear();
				sst << record_path_ << "\\" << "depth\\" << std::setw(7) << std::setfill('0') << (long long)current_frame.time_stamp_ << ".png";
				cv::imwrite(sst.str(), current_frame.depth_image_);
				//std::cout << sst.str() << endl;
			}
			else
			{
				Sleep(10);
			}
		}
	}

	void close()
	{
		while (!frame_queue_.empty())
		{
			Sleep(10);
		}
		isQuit = true;
	}

};