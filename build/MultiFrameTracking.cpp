#include "MultiFrameTracking\MultiFrameTracking.h"
#include "MultiFrameTracking\debug_utils.h"
namespace ORB_SLAM2
{

MultiFrameTracking::MultiFrameTracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
		KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor)
		:mState(NO_IMAGES_YET), mLastProcessedState(NO_IMAGES_YET)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		mNcameras = fSettings["MultiCamera.n_frame"];

		for (int i = 0; i < mNcameras; i++)
		{
			std::stringstream sst;
			sst << "camera" << i;
			std::vector<float> C;
			std::vector<float> rot;
			fSettings[(sst.str() + ".C")] >> C;
			fSettings[(sst.str() + ".R")] >> rot;
			//print_vect(cv::Mat(rot));
			//print_vect(cv::Mat(C));
			cv::Mat R;
			cv::Rodrigues(rot, R);
			//R = R.t();
			//print_mat(R);

			cv::Mat camera_trans = cv::Mat::eye(4, 4, CV_32F);
			R.copyTo(camera_trans(cv::Range(0, 3), cv::Range(0, 3)));
			cv::Mat t = -R*cv::Mat(C);
			t *= 0.1;
			//print_vect(t);
			t.copyTo(camera_trans(cv::Range(0, 3), cv::Range(3, 4)));
			//print_mat(camera_trans);
			mvrelative_pose_MulFrameToCameras.push_back(camera_trans.inv());
			//print_mat(mvrelative_pose_MulFrameToCameras[i]);
			mvpFrameDrawer.push_back(new FrameDrawer(pMap));
			Tracking*  tracker = new Tracking(pSys, pVoc, mvpFrameDrawer[i], pMapDrawer, pMap, pKFDB, strSettingPath, System::MONOCULAR);
			mvptrackers.push_back(tracker);
			//pMapDrawer->addDebugCameras(mvrelative_pose_MulFrameToCameras[i], cv::Scalar(0.5, 0.5, 0.5));
		}
		//pMapDrawer->addDebugCameras(cv::Mat::eye(4,4,CV_32F), cv::Scalar(0.5, 0.5, 0.5));

		//根据摄像机之间的相邻关系增加连接关系，要求摄像机按照顺时针或逆时针顺序给出
		//也可以考虑把所有其他相机都添加进去
		for (int i = 0; i < mvptrackers.size(); i++)
		{
			int prev_tracker = i - 1 >= 0 ? i - 1 : mvptrackers.size() - 1;
			int next_tracker = i + 1 < mvptrackers.size() ? i + 1 : 0;
			if (i != prev_tracker)
				mvptrackers[i]->addNeighborTracker(mvptrackers[prev_tracker]);
			if (i!=next_tracker)
				mvptrackers[i]->addNeighborTracker(mvptrackers[next_tracker]);
		}
		initialize_reference_frames.resize(4, std::vector<Frame>(4));
	}


void MultiFrameTracking::track()
	{
		if (mState == NO_IMAGES_YET)
		{
			mState = NOT_INITIALIZED;
			//std::cout << "original relative pose" << std::endl;
			//cv::waitKey(0);
		}

		mLastProcessedState = mState;

		// Get Map Mutex -> Map cannot be changed
		//unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

		if (mState == NOT_INITIALIZED)
		{

			MultiFrameInitialization();

			//mpFrameDrawer->Update(this);

			for (int i = 0; i < mNcameras; i++)
			{
				mvpFrameDrawer[i]->Update(mvptrackers[i]);
				cv::Mat im = mvpFrameDrawer[i]->DrawFrame();
				std::stringstream sst;
				sst << "Current Frame " << i;
				cv::imshow(sst.str(), im);
			}
			//cv::waitKey(0);
			if (mState != OK)
				return;
		}
		else
		{
			//cv::waitKey(0);
			mcamera_pose = cv::Mat::zeros(4, 4, CV_32F);
			int track_OK_num = 0;
			int track_OK_ID = -1;
			for (int i = 0; i < mNcameras; i++)
			{
				mvptrackers[i]->TrackForMultiFrame();
				if (mvptrackers[i]->mState == OK)
				{
					track_OK_num++;
					track_OK_ID = i;
					mcamera_pose += mvptrackers[i]->mCurrentFrame.mTcw.inv()*mvrelative_pose_MulFrameToCameras[i].inv();
				}
			}
			print_value(track_OK_num);
			if (track_OK_num == 0)
			{
				mState = LOST;
				std::cout << "Multiframe track fail" << std::endl;
				cv::waitKey(0);
			}
			else
			{
				mcamera_pose /= track_OK_num;
				bool need_create_keyframe = false;
				for (int i = 0; i < mNcameras; i++)
				{
					need_create_keyframe |= mvptrackers[i]->setCurrentTrackedPose((mcamera_pose*mvrelative_pose_MulFrameToCameras[i]).inv());
					std::cout << "set camera " << i << std::endl;					
				}
				if (need_create_keyframe)
				{
					for (int i = 0; i < mNcameras; i++)
					{
						mvptrackers[i]->CreateNewKeyFrame();
					}
				}
			}
			//	// System is initialized. Track Frame.
			//	bool bOK;

			//	// Local Mapping is activated. This is the normal behaviour, unless
			//	// you explicitly activate the "only tracking" mode.

							//if (mState == OK)
							//{
							//	// Local Mapping might have changed some MapPoints tracked in last frame
							//	CheckReplacedInLastFrame();
			
							//	if (mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId + 2)
							//	{
							//		bOK = TrackReferenceKeyFrame();
							//	}
							//	else
							//	{
							//		bOK = TrackWithMotionModel();
							//		if (!bOK)
							//			bOK = TrackReferenceKeyFrame();
							//	}
							//}
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
		for (int i = 0; i < mNcameras; i++)
		{
			cv::Mat im = mvpFrameDrawer[i]->DrawFrame();
			std::stringstream sst;
			sst << "Current Frame " << i;
			cv::imshow(sst.str(), im);
		}
		cv::waitKey(10);
	}


	bool MultiFrameTracking::TrackLocalMap()
	{
		for (int i = 0; i < mvptrackers.size(); i++)
		{
			mvptrackers[i]->UpdateLocalMap();
			mvptrackers[i]->SearchLocalPoints();
		}
	}


	cv::Mat MultiFrameTracking::GrabImageMultiFrame(const std::vector<cv::Mat> &ims, const double &timestamp)
	{
		for (int i = 0; i < mNcameras; i++)
		{
			mvptrackers[i]->GrabImageOnly(ims[i], timestamp);
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


	void MultiFrameTracking::MultiFrameInitialization()
	{
		int initialized_camera_id = -1;
		for (int i = 0; i < mNcameras; i++)
		{
			Tracking::eInitilizationState initialize_state = mvptrackers[i]->MonocularInitialization();
			if (initialize_state == Tracking::INITIALIZE_SUCCESS)
			{
				
				
				if (mvptrackers[i]->CreateInitialMapMonocular())
				{
					initialized_camera_id = i;
					cv::Mat transform_to_mulFrame = mvrelative_pose_MulFrameToCameras[i];
					transformInitializedTracker(mvptrackers[i], transform_to_mulFrame);
					//mvptrackers[i]->mpMapDrawer->addDebugCameras(mvptrackers[i]->mCurrentFrame.mTcw,cv::Scalar(0,0,0));
					mcamera_pose = mvptrackers[i]->mCurrentFrame.mTcw.inv()*mvrelative_pose_MulFrameToCameras[i].inv();
					break;
				}
			}
			else if (initialize_state == Tracking::INITIALIZE_CREATE)
			{
				for (int c = 0; c < mNcameras; c++)
				{
					initialize_reference_frames[i][c] = mvptrackers[c]->mCurrentFrame;
				}
			}

		}

		if (initialized_camera_id >= 0)
		{
			//mvptrackers[0]->mpMapDrawer->addDebugCameras(mcamera_pose,cv::Scalar(1, 1, 0));
			mState = OK;
			//print_value(mvptrackers[initialized_camera_id]->mpMap->MapPointsInMap());
			//cv::waitKey(0);
			for (int i = 0; i < mNcameras; i++)
			{
				
				
				cv::Mat reference_pose = mvrelative_pose_MulFrameToCameras[i];
				cv::Mat current_pose = mcamera_pose*mvrelative_pose_MulFrameToCameras[i];
				//print_mat(mcamera_pose);
				//print_mat(reference_pose);
				//print_mat(current_pose);

				//mvptrackers[i]->mpMapDrawer->addDebugCameras(reference_pose.inv(), cv::Scalar(1, 0, 1));
				//mvptrackers[i]->mpMapDrawer->addDebugCameras(current_pose.inv(), cv::Scalar(0, 1, 1));

				//print_value(initialize_reference_frames[initialized_camera_id][i].mTimeStamp);
				if (i == initialized_camera_id)
					continue;
				mvptrackers[i]->MonocularInitializationUsingFramePose(initialize_reference_frames[initialized_camera_id][i], reference_pose, current_pose);
				//print_value(mvptrackers[initialized_camera_id]->mpMap->MapPointsInMap());
				//cv::waitKey(0);
			}
			for (int i = 0; i < mNcameras-1; i++)
			{
				mvptrackers[i]->mpLastKeyFrame->multiFrame_neighbor = mvptrackers[i + 1]->mpLastKeyFrame;
				//print_value(mvptrackers[initialized_camera_id]->mpMap->MapPointsInMap());
			}
			if (mNcameras>1)
				mvptrackers[mNcameras - 1]->mpLastKeyFrame->multiFrame_neighbor = mvptrackers[0]->mpLastKeyFrame;
			//print_value(mvptrackers[initialized_camera_id]->mpMap->MapPointsInMap());
			mState = OK;
		}
	}

	void MultiFrameTracking::transformInitializedTracker(Tracking* tracker, const cv::Mat& transform_to_mulframe)
	{
		Map* map = tracker->mpMap;
		//print_mat(transform_to_mulframe);
		cv::Mat init_cam_pose = transform_to_mulframe*tracker->mInitialFrame.mTcw.inv();
		//print_mat(init_cam_pose);
		cv::Mat current_cam_pose = transform_to_mulframe*tracker->mCurrentFrame.mTcw.inv();

		tracker->mvpLocalKeyFrames[0]->SetPose(current_cam_pose.inv());
		tracker->mvpLocalKeyFrames[1]->SetPose(init_cam_pose.inv());
		//print_mat(current_cam_pose);
		tracker->mInitialFrame.SetPose(init_cam_pose.inv());
		tracker->mCurrentFrame.SetPose(current_cam_pose.inv());


		std::vector<cv::Point3f>& iniP3D = tracker->mvIniP3D;
		std::vector<int> iniMatches = tracker->mvIniMatches;
		//std::ofstream file("original.vtx");
		//std::ofstream file2("inMulframe.vtx");
		//print_mat(transform_to_mulframe.inv());
		for (int i = 0; i < iniP3D.size(); i++)
		{
			if (iniMatches[i] < 0)
				continue;
			cv::Mat pos(iniP3D[i]);
			//print_vect(pos);
			//file << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << std::endl;
			cv::Mat homo_pos = cv::Mat::ones(4, 1, CV_32F);
			pos.copyTo(homo_pos.rowRange(cv::Range(0, 3)));
			//print_vect(homo_pos);
			homo_pos = transform_to_mulframe*homo_pos;
			//print_vect(homo_pos);
			
			//cv::Mat remap_pos = transform_to_mulframe.inv()*homo_pos;
			//print_vect(remap_pos);
			//file2 << homo_pos.at<float>(0) << " " << homo_pos.at<float>(1) << " " << homo_pos.at<float>(2) << std::endl;
			float* homo_pos_data = (float*)homo_pos.data;
			iniP3D[i].x = homo_pos_data[0];
			iniP3D[i].y = homo_pos_data[1];
			iniP3D[i].z = homo_pos_data[2];
			//print_value(iniP3D[i]);
			//print_value(tracker->mvIniP3D[i]);
		}
		vector<MapPoint*> vpAllMapPoints = tracker->mpMap->GetAllMapPoints();
		for (size_t iMP = 0; iMP<vpAllMapPoints.size(); iMP++)
		{
			if (vpAllMapPoints[iMP])
			{
				MapPoint* pMP = vpAllMapPoints[iMP];
				cv::Mat pos = pMP->GetWorldPos();
				cv::Mat homo_pos = cv::Mat::ones(4, 1, CV_32F);
				pos.copyTo(homo_pos.rowRange(cv::Range(0, 3)));
				homo_pos = transform_to_mulframe*homo_pos;
				pMP->SetWorldPos(homo_pos.rowRange(0,3));
			}
		}

	}



	void MultiFrameTracking::SetLocalMapper(LocalMapping *pLocalMapper)
	{
		for (int i = 0; i < mNcameras; i++)
		{
			mvptrackers[i]->SetLocalMapper(pLocalMapper);
		}
	}

	void MultiFrameTracking::SetLoopClosing(LoopClosing *pLoopClosing)
	{
		for (int i = 0; i < mNcameras; i++)
		{
			mvptrackers[i]->SetLoopClosing(pLoopClosing);
		}

	}
}
