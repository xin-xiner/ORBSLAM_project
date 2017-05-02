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


	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MULTIFRAME, true);
	//ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	// --------------
	// 4. Load image paths and timestamps
	// --------------


	std::vector<cv::VideoCapture> cameras(4);
	cameras[0].open(video_path + "front.avi");
	cameras[1].open(video_path + "right.avi");
	cameras[3].open(video_path + "rear.avi");
	cameras[2].open(video_path + "left.avi");
	

	int nImages = cameras[0].get(CV_CAP_PROP_FRAME_COUNT);
	double fps = cameras[0].get(CV_CAP_PROP_FPS);




	float pixel_height = 0.0042;
	float f_image_ = 306.605;

	std::string correction_table = argv[4];
	std::cout << "generate corrector" << std::endl;
	FisheyeCorrector corrector(correction_table, cameras[0].get(CV_CAP_PROP_FRAME_HEIGHT), cameras[0].get(CV_CAP_PROP_FRAME_WIDTH), pixel_height, 306.6, 50, 60);
	corrector.setAxisDirection(0, 30, -2);
	corrector.updateMap();
	corrector.setClipRegion(cv::Rect(cv::Point(0, 200), cv::Point(corrector.getCorrectedSize().width, corrector.getCorrectedSize().height - 200)));

	//correctors[0].setSizeScale(0.5);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	std::cout << endl << "-------" << endl;
	std::cout << "Start processing sequence ..." << endl;
	std::cout << "Images in the sequence: " << nImages << endl << endl;

	long vTimeCount = 0;
	// Main loop
	
	std::stringstream sst;
	sst << argv[5];
	int start_frame;
	sst >> start_frame;
	for (int ni = 0; ni<nImages; ni++)
	{
		cv::Mat fisheye_im;
		std::vector<cv::Mat> multi_frame_image(4);
		for (int i = 0; i < 4; i++)
		{
			cameras[i] >> multi_frame_image[i];
			if (multi_frame_image[i].empty())
			{
				cerr << endl << "Failed to load image at: " << vTimeCount << endl;
				return 1;
			}
		}
		
		if (ni < start_frame)
			continue;


		for (int i = 0; i < 4; i++)
		{
			cv::cvtColor(multi_frame_image[i], multi_frame_image[i], cv::COLOR_BGR2GRAY);
			corrector.correct(multi_frame_image[i], multi_frame_image[i]);
			/*std::stringstream sst;
			sst << "Current Frame " << i;
			cv::imshow(sst.str(), multi_frame_image[i]);*/
		}
		
		double tframe = vTimeCount;

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		std::vector<cv::Mat> imgs;

		cv::waitKey(10);
		// Pass the image to the SLAM system
		SLAM.TrackMultiFrame(multi_frame_image, tframe);
		//SLAM.TrackMonocular(imgs[0], tframe);

		if (SLAM.GetTrackingState() == 3 || (ni >= 30 && ni % 30 == 0))
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
		cv::waitKey(30);
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
