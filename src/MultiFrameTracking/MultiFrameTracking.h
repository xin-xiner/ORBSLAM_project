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
		std::vector<cv::Mat> mvcamerasToMulFrame;
		std::vector<FrameDrawer*> mvpFrameDrawer;
		cv::Mat mcamera_pose;

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

		// Input sensor
		int mSensor;


	public:
		MultiFrameTracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
			KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor)
		{
			cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
			float mNcameras = fSettings["MultiCamera.n_frame"];

			for (int i = 0; i < mNcameras; i++)
			{
				std::stringstream sst;
				sst << "camera" << i ;
				std::vector<float> C;
				std::vector<float> R;
				fSettings[(sst.str() + ".C")] >> C;
				fSettings[(sst.str() + ".R")] >> R;
				cv::Mat rot;
				cv::Rodrigues(R, rot);;
				cv::Mat camera_trans = cv::Mat::ones(4, 4, CV_32F);
				camera_trans(cv::Range(0, 3), cv::Range(0, 3)) = rot;
				camera_trans(cv::Range(0, 3), cv::Range(4, 4)) = -rot*cv::Mat(R);
				mvcamerasToMulFrame.push_back(camera_trans.inv());

				mvpFrameDrawer.push_back(new FrameDrawer(pMap));
				Tracking*  tracker= new Tracking(pSys, pVoc, mvpFrameDrawer[i], 0, pMap, 0, strSettingPath,sensor);
				mvptrackers.push_back(tracker);

			}
		}


		void track()
		{
			if (mState == NO_IMAGES_YET)
			{
				mState = NOT_INITIALIZED;
			}

			mLastProcessedState = mState;

			// Get Map Mutex -> Map cannot be changed
			//unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

			if (mState == NOT_INITIALIZED)
			{

				MultiFrameInitialization();

				//mpFrameDrawer->Update(this);

				if (mState != OK)
					return;
			}
			//else
			//{
			//	// System is initialized. Track Frame.
			//	bool bOK;

			//	// Local Mapping is activated. This is the normal behaviour, unless
			//	// you explicitly activate the "only tracking" mode.

//				if (mState == OK)
//				{
//					// Local Mapping might have changed some MapPoints tracked in last frame
//					CheckReplacedInLastFrame();
//
//					if (mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId + 2)
//					{
//						bOK = TrackReferenceKeyFrame();
//					}
//					else
//					{
//						bOK = TrackWithMotionModel();
//						if (!bOK)
//							bOK = TrackReferenceKeyFrame();
//					}
//				}
//				else
//				{
//					bOK = Relocalization();
//				}
//		
//
//				mCurrentFrame.mpReferenceKF = mpReferenceKF;
//
//				// If we have an initial estimation of the camera pose and matching. Track the local map.
//
//					//std::cout << "tracking statue reference frame: " << bOK << std::endl;
//				if (bOK)
//					bOK = TrackLocalMap();
//
//
//
//				if (bOK)
//					mState = OK;
//				else
//					mState = LOST;
//
//				// Update drawer
//				mpFrameDrawer->Update(this);
////****************************************************************add keyframe
//				// If tracking were good, check if we insert a keyframe
//				if (bOK)
//				{
//					// Update motion model
//					if (!mLastFrame.mTcw.empty())
//					{
//						cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
//						mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
//						mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
//						mVelocity = mCurrentFrame.mTcw*LastTwc;
//					}
//					else
//						mVelocity = cv::Mat();
//
//					mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
//
//					// Clean temporal point matches
//					for (int i = 0; i<mCurrentFrame.N; i++)
//					{
//						MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
//						if (pMP)
//							if (pMP->Observations()<1)
//							{
//								mCurrentFrame.mvbOutlier[i] = false;
//								mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
//							}
//					}
//
//					// Delete temporal MapPoints
//					for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
//					{
//						MapPoint* pMP = *lit;
//						delete pMP;
//					}
//					mlpTemporalPoints.clear();
//
//					// Check if we need to insert a new keyframe
//					if (NeedNewKeyFrame())
//						CreateNewKeyFrame();
//
//					// We allow points with high innovation (considererd outliers by the Huber Function)
//					// pass to the new keyframe, so that bundle adjustment will finally decide
//					// if they are outliers or not. We don't want next frame to estimate its position
//					// with those points so we discard them in the frame.
//					for (int i = 0; i<mCurrentFrame.N; i++)
//					{
//						if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
//							mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
//					}
//				}
//
//				// Reset if the camera get lost soon after initialization
//				if (mState == LOST)
//				{
//					if (mpMap->KeyFramesInMap() <= 5)
//					{
//						cout << "Track lost soon after initialisation, reseting..." << endl;
//						mpSystem->Reset();
//						return;
//					}
//				}
//
//				if (!mCurrentFrame.mpReferenceKF)
//					mCurrentFrame.mpReferenceKF = mpReferenceKF;
//
//				mLastFrame = Frame(mCurrentFrame);
//			}
//			//**********************************************************************************
//			// Store frame pose information to retrieve the complete camera trajectory afterwards.
//			if (!mCurrentFrame.mTcw.empty())
//			{
//				cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
//				mlRelativeFramePoses.push_back(Tcr);
//				mlpReferences.push_back(mpReferenceKF);
//				mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
//				mlbLost.push_back(mState == LOST);
//			}
//			else
//			{
//				// This can happen if tracking is lost
//				mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
//				mlpReferences.push_back(mlpReferences.back());
//				mlFrameTimes.push_back(mlFrameTimes.back());
//				mlbLost.push_back(mState == LOST);
//			}
		}


		cv::Mat GrabImageMultiFrame(const std::vector<cv::Mat> &ims, const double &timestamp)
		{
			for (int i = 0; i < mNcameras; i++)
			{
				mvptrackers[i]->GrabImageMonocular(ims[i], timestamp);
			}

			//考虑是否需要创建dummy的MultiFrame
			/*if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
				mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
			else
				mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
*/
			track();

			return mcamera_pose;
		}


		void MultiFrameInitialization()
		{
			for (int i = 0; i < mNcameras; i++)
			{
				if (mvptrackers[i]->MonocularInitialization())
				{
					mcamera_pose = mvcamerasToMulFrame[i].inv()*mvptrackers[i]->mCurrentFrame.mTcw*mvcamerasToMulFrame[i];
					cv::Mat transform_to_mulFrame = mvcamerasToMulFrame[i].inv();
					transformInitializedTracker(mvptrackers[i], transform_to_mulFrame);
					if (mvptrackers[i]->CreateInitialMapMonocular())
						break;
				}
					
			}
		}


		private:
			void transformInitializedTracker(Tracking* tracker, const cv::Mat& transform_to_mulframe)
			{
				Map* map = tracker->mpMap;
				cv::Mat init_cam_pose = transform_to_mulframe*tracker->mInitialFrame.mTcw;
				cv::Mat current_cam_pose = transform_to_mulframe*tracker->mCurrentFrame.mTcw;
				tracker->mInitialFrame.SetPose(init_cam_pose);
				tracker->mCurrentFrame.SetPose(current_cam_pose);
				std::vector<cv::Point3f> iniP3D = tracker->mvIniP3D;
				std::vector<int> iniMatches = tracker->mvIniMatches;
				for (int i = 0; i < iniP3D.size(); i++)
				{
					if (iniMatches[i] < 0)
						continue;
					cv::Mat pos(iniP3D[i]);
					cv::Mat homo_pos(4, 1, CV_32F);
					homo_pos.rowRange(cv::Range(0, 3)) = pos;
					homo_pos.at<float>(3, 0) = 1;
					homo_pos = transform_to_mulframe*homo_pos;
					float* homo_pos_data = (float*)homo_pos.data;
					iniP3D[i].x = homo_pos_data[0];
					iniP3D[i].y = homo_pos_data[1];
					iniP3D[i].z = homo_pos_data[2];
				}
			}
	};
}
