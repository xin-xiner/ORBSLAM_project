
#include "KinectReader.h"
#include<System.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include "frameWriter.h"

#include <boost\thread\thread.hpp>
using namespace std;


int main(int argc, char **argv)
{
	if (argc != 3)
	{
		cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
		return 1;
	}

	// Retrieve paths to images


	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	//ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
	FrameQueue frame_queue("frames\\");
	system("DEL/q  frames\\rgb\\*.*");
	system("DEL/q  frames\\depth\\*.*");
	std::thread  visThread(&FrameQueue::writeThread, &frame_queue);
	KinectReader capture;
	capture.sizeType = 1;

	cout << "Start processing sequence ..." << endl;

	// Main loop
	cv::Mat imRGB, imD, depth8U,cameraPose;

	double frameID = 0;
	cv::namedWindow("rgb", 0);
	while (cv::waitKey(10) != 'c')
	{
		while (!capture.getNextFrame(imD, imRGB)){ Sleep(1); };

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
		//cameraPose = SLAM.TrackRGBD(imRGB, imD, frameID);
		cameraPose = SLAM.TrackMonocular(imRGB, frameID);
		if (!cameraPose.empty())
		{
			frame_queue.pushBack(imRGB, imD, frameID);
			frameID++;
		}
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();


		// Wait to load the next frame

	}

	// Stop all threads

	SLAM.Shutdown();

	frame_queue.close();

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
	SLAM.SaveKeyFrameTrajectoryTUM("CameraTrajectory_keyframe.txt");
	SLAM.SaveMapClouds("MapPoints.vtx");
	return 0;
}



//int main()
//{
//
//}
//int main()
//{
//	KinectReader capture;
// cv::Mat rgb, depth,depthu8;
// while (capture.getNextFrame(depth, rgb))
// {
//	 normalizeDepthImage(depth,depthu8);
//  cv::imshow("rgb", rgb);
//  cv::imshow("depth", depthu8);
//  cv::waitKey(10);
// }
//}
