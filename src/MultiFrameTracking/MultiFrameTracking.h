#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

namespace ORB_SLAM2
{
	class MultiFrameTracking 
	{
		int mNcameras;
		std::vector<Tracking*> mvptrackers;
		std::vector<cv::Mat> mvrelative_pose_MulFrameToCameras;
		std::vector<cv::Mat> mvrelative_pose_CamerasToMulFrame;
		std::vector<FrameDrawer*> mvpFrameDrawer;
		cv::Mat mcamera_pose;

		std::vector<std::vector<Frame>> initialize_reference_frames;

	public:
		// Tracking states
		enum eTrackingState{
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			OK = 2,
			LOST = 3
		};
		
		eTrackingState mState;
		eTrackingState mLastProcessedState;
		int mnMatchesInliers;

	public:
		MultiFrameTracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
			KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

		void track();

		cv::Mat GrabImageMultiFrame(const std::vector<cv::Mat> &ims, const double &timestamp);


		void MultiFrameInitialization();


		protected:
			void transformInitializedTracker(Tracking* tracker, const cv::Mat& transform_to_mulframe);

			bool TrackLocalMap();
			bool TrackReferenceKeyFrame();
			bool TrackWithMotionModel();
			public:
				void SetLocalMapper(LocalMapping *pLocalMapper);
			
				void SetLoopClosing(LoopClosing *pLoopClosing);
	};
}
