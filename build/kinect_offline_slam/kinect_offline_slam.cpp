
#include "KinectOfflineReader.h"
#include<System.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <boost\thread\thread.hpp>
using namespace std;


int main(int argc, char **argv)
{
	if (argc != 4)
	{
		cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
		return 1;
	}

	// Retrieve paths to images
	KinectOfflineReader capture(argv[3]);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

	

	cout << "Start processing sequence ..." << endl;

	// Main loop
	cv::Mat imRGB, imD, depth8U, cameraPose;

	int frameID = 0;

	while (capture.getNextFrame(imD, imRGB))
	{
		

		// Read image and depthmap from file
		cv::imshow("rgb", imRGB);
		/*
		imD.convertTo(depth8U, CV_8U);
		cv::imshow("depth", depth8U);
		cv::waitKey(10);*/
		if (imRGB.empty())
		{
			cerr << endl << "Failed to load image at: "
				<< endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

		// Pass the image to the SLAM system
		cameraPose = SLAM.TrackRGBD(imRGB, imD, capture.getFrameID());
		//cameraPose = SLAM.TrackMonocular(imRGB,capture.getFrameID());
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
		if (SLAM.GetTrackingState() == 3 || (frameID >= 30 && frameID % 30 == 0))
		{
			SLAM.SaveMapClouds("pointClouds.vtx");
			SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
		}
		frameID++;
		// Wait to load the next frame
		cv::waitKey(30);
	}

	// Stop all threads
	while (cv::waitKey(30) != 'c'){ Sleep(1); };
	SLAM.Shutdown();

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
	SLAM.SaveKeyFrameTrajectoryTUM("CameraTrajectory_keyframe.txt");
	return 0;
}
