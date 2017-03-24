#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

#define usleep(x) Sleep((float)x/1000.0f)

int main(int argc, char **argv)
{
	if (argc < 4)
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
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	long vTimeCount = 0;
	// Main loop
	cv::Mat im;
	for (int ni = 0; ni<nImages; ni++)
	{
		// Read image from file
		video >> im;
		double tframe = vTimeCount;

		if (im.empty())
		{
			cerr << endl << "Failed to load image at: " << vTimeCount << endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

		// Pass the image to the SLAM system
		SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;
		vTimeCount += 1.0f / fps;
		// Wait to load the next frame
		SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory_all.txt");
	SLAM.SaveTrajectoryVtx("CameraTrajectory.vtx");
	SLAM.SaveKeyFrameTrajectoryTUM("CameraTrajectory_keyframe.txt");
	SLAM.SaveMapClouds("pointClouds.vtx");
	return 0;
}
