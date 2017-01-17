#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <fisheye_corrector\fisheye_corrector.h>
using namespace std;

#define usleep(x) Sleep((float)x/1000.0f)

int main(int argc, char **argv)
{
	if (argc != 6)
	{
		cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}

	// Retrieve paths to images
	std::string video_path = argv[3];
	
	cv::VideoCapture video(video_path);
	int nImages = video.get(CV_CAP_PROP_FRAME_COUNT);
	double fps = video.get(CV_CAP_PROP_FPS);
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::FISHEYE, true);
	//ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	float pixel_height = 0.0042;
	float f_image_ = 306.605;
	
	std::string correction_table = argv[4];
	std::cout << "generate corrector" << std::endl;
	std::vector<FisheyeCorrector> correctors(3);
	std::cout << video.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << video.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
	std::cout << pixel_height << std::endl;
	std::cout << f_image_ << std::endl;
	correctors[0] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 60, 40);
	correctors[0].setAxisDirection(0, 40, 0);//30,35,-7
	correctors[0].updateMap();
	correctors[0].setClipRegion(cv::Rect(cv::Point(0, 475), cv::Point(correctors[0].getCorrectedSize().width, correctors[0].getCorrectedSize().height - 200)));
	//correctors[0].setSizeScale(0.5);

	correctors[1] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[1].setAxisDirection(80, 40, -15);//30,35,-7
	correctors[1].updateMap();
	correctors[1].setClipRegion(cv::Rect(cv::Point(0, 605), cv::Point(correctors[1].getCorrectedSize().width, correctors[1].getCorrectedSize().height)));
	//correctors[1].setSizeScale(0.5);


	correctors[2] = FisheyeCorrector(correction_table, video.get(CV_CAP_PROP_FRAME_HEIGHT), video.get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, f_image_, 50, 30);
	correctors[2].setAxisDirection(-80, 40, 15);//30,35,-7
	correctors[2].updateMap();
	correctors[2].setClipRegion(cv::Rect(cv::Point(0, 605), cv::Point(correctors[2].getCorrectedSize().width, correctors[2].getCorrectedSize().height)));
	//correctors[2].setSizeScale(0.5);


	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	std::cout << endl << "-------" << endl;
	std::cout << "Start processing sequence ..." << endl;
	std::cout << "Images in the sequence: " << nImages << endl << endl;

	long vTimeCount = 0;
	// Main loop
	cv::Mat fisheye_im;     
	std::stringstream sst;
	sst << argv[5];
	int start_frame;
	sst >> start_frame;
	for (int ni = 0; ni<nImages; ni++)
	{
		
		// Read image from file
		video >> fisheye_im;
		if (ni < start_frame)
			continue;
		cv::cvtColor(fisheye_im, fisheye_im, cv::COLOR_BGR2GRAY);
		double tframe = vTimeCount;

		if (fisheye_im.empty())
		{
			cerr << endl << "Failed to load image at: " << vTimeCount << endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		std::vector<cv::Mat> imgs;
		for (int view = 0; view < correctors.size(); view++)
		{
			cv::Mat current_view;
			correctors[view].correct(fisheye_im,current_view);
			imgs.push_back(current_view);
			//std::stringstream sst;
			//sst << "view" << view;
			//cv::imshow(sst.str(), current_view);
		}
		cv::waitKey(10);
		// Pass the image to the SLAM system
		SLAM.TrackFisheye(fisheye_im, imgs, tframe, correctors);
		//SLAM.TrackMonocular(imgs[0], tframe);

		if (SLAM.GetTrackingState() == 3 || (ni>=30&&ni%30==0))
		{
			SLAM.SaveMapClouds("pointClouds.vtx");
			SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
		}
		//cv::waitKey(0);
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;
		vTimeCount += 1;//1000.0f / fps;
		// Wait to load the next frame
		
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	std::sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	std::cout << "-------" << endl << endl;
	std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	std::cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
	SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
	SLAM.SaveKeyFrameTrajectoryTUM("CameraTrajectory_keyframe.txt");
	SLAM.SaveMapClouds("pointClouds.vtx");
	return 0;
}
