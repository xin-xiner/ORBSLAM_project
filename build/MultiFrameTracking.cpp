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
			mvrelative_pose_CamerasToMulFrame.push_back(camera_trans);
			//print_mat(mvrelative_pose_MulFrameToCameras[i]);
			mvpFrameDrawer.push_back(new FrameDrawer(pMap));
			Tracking*  tracker = new Tracking(pSys, pVoc, mvpFrameDrawer[i], pMapDrawer, pMap, pKFDB, strSettingPath, System::MONOCULAR);
			mvptrackers.push_back(tracker);
			//pMapDrawer->addDebugCameras(mvrelative_pose_MulFrameToCameras[i], cv::Scalar(0.5, 0.5, 0.5));
		}
		mpMapDrawer = pMapDrawer;
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
		for (int i = 0; i < mNcameras; i++)
		{
			std::stringstream sst;
			sst << "Current Frame " << i;
			cv::namedWindow(sst.str(), 0);
		}
		if (mState == NO_IMAGES_YET)
		{
			mState = NOT_INITIALIZED;
			//std::cout << "original relative pose" << std::endl;
			//cv::waitKey(0);
		}

		mLastProcessedState = mState;

		// Get Map Mutex -> Map cannot be changed
		unique_lock<mutex> lock(mvptrackers[0]->mpMap->mMutexMapUpdate);
		bool bOK;
		if (mState == NOT_INITIALIZED)
		{

			MultiFrameInitialization();
			for (int i = 0; i < mNcameras; i++)
			{
				mvpFrameDrawer[i]->Update(mvptrackers[i]);
				cv::Mat im = mvpFrameDrawer[i]->DrawFrame();
				std::stringstream sst;
				sst << "Current Frame " << i;
				cv::namedWindow(sst.str(),0);
				cv::imshow(sst.str(), im);
			}
			//mpFrameDrawer->Update(this);
			//cv::waitKey(0);
			if (mState != OK)
				return;
		}
		else
		{
			cv::waitKey(0);
			
			create_new_keyframe = false;
			if (mState == OK)
			{
				// Local Mapping might have changed some MapPoints tracked in last frame
				CheckReplacedInLastFrame();

				if (mVelocity.empty())// || mCurrentFrame.mnId<mnLastRelocFrameId + 2)
				{
					bOK = TrackReferenceKeyFrame();
				}
				else
				{
					bOK = TrackWithMotionModel();
					if (!bOK)
						bOK = TrackReferenceKeyFrame();
				}
			}
			else
			{
				//bOK = Relocalization();
			}

			for (int c = 0; c < mvptrackers.size(); c++)
			{
				mvptrackers[c]->mCurrentFrame.mpReferenceKF = mvptrackers[c]->mpReferenceKF;
			}
			bOK |= TrackLocalMap();
			if (bOK)
			{
				bool need_create_keyframe = false;
				for (int i = 0; i < mNcameras; i++)
				{
					need_create_keyframe |= mvptrackers[i]->setCurrentTrackedPose((mcamera_pose*mvrelative_pose_MulFrameToCameras[i]).inv());
					//std::cout << "set camera " << i << std::endl;
				}
				if (need_create_keyframe)
				{
					for (int i = 0; i < mNcameras; i++)
					{
						//std::stringstream sst;
						/*sst << "Current Frame " << i;
						std::cout << sst.str() << std::endl;
						std::cout << "before "<<std::endl << mvptrackers[i]->mpReferenceKF->GetPose()<<std::endl;*/
						mvptrackers[i]->CreateNewKeyFrame();
						
						//std::cout <<"after "<<std::endl<< mvptrackers[i]->mpReferenceKF->GetPose()<<std::endl;
					}
				}
				if (!last_camera_pose.empty())
				{
					mVelocity = mcamera_pose*last_camera_pose.inv();
				}
				mcamera_pose.copyTo(last_camera_pose);
				mpMapDrawer->SetCurrentCameraPose(mcamera_pose.inv());
			}
			else
			{
				mState = LOST;
				std::cout << "Multiframe track fail" << std::endl;
				cv::waitKey(0);
			}

				
		}
		for (int i = 0; i < mNcameras; i++)
		{
			mvpFrameDrawer[i]->Update(mvptrackers[i]);
			cv::Mat im = mvpFrameDrawer[i]->DrawFrame();
			std::stringstream sst;
			sst << "Current Frame " << i;
			cv::imshow(sst.str(), im);
		}
		cv::waitKey(10);
	}


	bool MultiFrameTracking::TrackLocalMap()
	{
		std::vector<Frame*> current_frames(mvptrackers.size());
		for (int i = 0; i < mvptrackers.size(); i++)
		{
			mvptrackers[i]->UpdateLocalMap();
			mvptrackers[i]->SearchLocalPoints();
			current_frames[i] = &mvptrackers[i]->mCurrentFrame;
		}
		cv::Mat transform_from_mulframe_to_camera = mcamera_pose.inv();
		Optimizer::PoseOptimizationMultiframe(current_frames, mvrelative_pose_CamerasToMulFrame, transform_from_mulframe_to_camera);
		mcamera_pose = transform_from_mulframe_to_camera.inv();
		mnMatchesInliers = 0;

		// Update MapPoints Statistics
		for (int c = 0; c < current_frames.size(); c++)
		{
			Frame& mCurrentFrame = *current_frames[c];
			mvptrackers[c]->mnMatchesInliers = 0;
			for (int i = 0; i<mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvpMapPoints[i])
				{
					if (!mCurrentFrame.mvbOutlier[i])
					{
						mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
						if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
						{
							mnMatchesInliers++;
							mvptrackers[c]->mnMatchesInliers++;
						}
					}
				}
			}

			mvpFrameDrawer[c]->Update(mvptrackers[c]);
		}
		//为relocalization预留
		//// Decide if the tracking was succesful
		//// More restrictive if there was a relocalization recently
		//if (mCurrentFrame.mnId<mnLastRelocFrameId + mMaxFrames && mnMatchesInliers<50)//original value is 50
		//	return false;
		std::cout << "MultiFrameTracking TrackLocalMap： ";
		print_value(mnMatchesInliers);
		if (mnMatchesInliers<30)//wx-parameter-adjust 2016-12-31 original value is 30
			return false;
		else
			return true;

		
	}

	bool MultiFrameTracking::TrackReferenceKeyFrame()
	{
		std::vector<Frame*> current_frames(mvptrackers.size());
		int nmatches = 0;
		for (int i = 0; i < mvptrackers.size(); i++)
		{
			nmatches += mvptrackers[i]->prepareForTrackReference();
			current_frames[i] = &mvptrackers[i]->mCurrentFrame;
		}
		cv::Mat transform_from_mulframe_to_camera = mcamera_pose.inv();
		Optimizer::PoseOptimizationMultiframe(current_frames, mvrelative_pose_CamerasToMulFrame, transform_from_mulframe_to_camera);
		mcamera_pose = transform_from_mulframe_to_camera.inv();
		// Discard outliers
		int nmatchesMap = 0;
		for (int c = 0; c < current_frames.size(); c++)
		{
			Frame& mCurrentFrame = *current_frames[c];
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvpMapPoints[i])
				{
					if (mCurrentFrame.mvbOutlier[i])
					{
						MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
						mCurrentFrame.mvbOutlier[i] = false;
						pMP->mbTrackInView = false;
						pMP->mnLastFrameSeen = mCurrentFrame.mnId;
						nmatches--;
					}
					else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
						nmatchesMap++;
				}
			}
		}
		std::cout << "TrackReferenceKeyFrame after optimization" << nmatches << std::endl;
		return nmatchesMap >= 10;
	}

	bool MultiFrameTracking::TrackWithMotionModel()
	{
		std::vector<Frame*> current_frames(mvptrackers.size());
		int nmatches = 0;
		mcamera_pose = mVelocity*last_camera_pose;
		for (int i = 0; i < mvptrackers.size(); i++)
		{
			nmatches += mvptrackers[i]->prepareForTrackMotionModel();
			current_frames[i] = &mvptrackers[i]->mCurrentFrame;
		}
		cv::Mat transform_from_mulframe_to_camera = mcamera_pose.inv();
		Optimizer::PoseOptimizationMultiframe(current_frames, mvrelative_pose_CamerasToMulFrame, transform_from_mulframe_to_camera);
		mcamera_pose = transform_from_mulframe_to_camera.inv();
		// Discard outliers
		int nmatchesMap = 0;
		for (int c = 0; c < current_frames.size(); c++)
		{
			Frame& mCurrentFrame = *current_frames[c];
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvpMapPoints[i])
				{
					if (mCurrentFrame.mvbOutlier[i])
					{
						MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
						mCurrentFrame.mvbOutlier[i] = false;
						pMP->mbTrackInView = false;
						pMP->mnLastFrameSeen = mCurrentFrame.mnId;
						nmatches--;
					}
					else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
						nmatchesMap++;
				}
			}
		}
		std::cout << "TrackWithMotionModel after optimization" << nmatches << std::endl;
		return nmatchesMap >= 10;
	}

	void MultiFrameTracking::CheckReplacedInLastFrame()
	{
		for (int i = 0; i<mvptrackers.size(); i++)
			mvptrackers[i]->CheckReplacedInLastFrame();
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
