#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <Eigen\Geometry>
#include <Eigen\core>




#include <GL\GLew.h>
#include <GL\gl.h>
#include <GL\glu.h>
#include <GL\glut.h>

#include "PointCloud.h"
#include <mutex>



class FrameReader
{

	class Frame
	{
		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		unsigned int frameID_;
		Eigen::Matrix4f transform_camera_to_world_;
		Frame(unsigned int frameID, Eigen::Vector3f& translation, Eigen::Quaternion<float>& rotation)
			:frameID_(frameID)
		{
			Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();
			transform_camera_to_world_
				<< 
				rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), translation(0),
				rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2), translation(1),
				rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2), translation(2),
				0, 0, 0, 1;
		}

		Frame(const Frame& f)
		{
			frameID_ = f.frameID_;
			transform_camera_to_world_ = f.transform_camera_to_world_;
		}

	};

	PointCloud point_cloud_;
	float fx_;
	float fy_;
	float cx_;
	float cy_;
	float factor_;


	std::string depth_image_path_;
	std::string rgb_image_path_;

	std::mutex point_cloud_mutex_;

	unsigned int vertex_buffer_Id_ = 0;
	int point_size_;


	bool point_cloud_is_uptate = false;
public:
	std::vector<Frame, Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4f> >> frame_list_;

	FrameReader(std::string depth_image_path, std::string rgb_image_path,std::string trajectory_path ,std::string camera_info_path)
		:depth_image_path_(depth_image_path), rgb_image_path_(rgb_image_path)
	{
		cv::FileStorage fSettings(camera_info_path, cv::FileStorage::READ);
		if (!fSettings.isOpened())
		{
			std::cout << "open trajectory file error: " << camera_info_path << std::endl;
			system("pause");
			exit(0);
		}

		fx_ = fSettings["Camera.fx"];
		fy_ = fSettings["Camera.fy"];
		cx_ = fSettings["Camera.cx"];
		cy_ = fSettings["Camera.cy"];
		factor_ = fSettings["DepthMapFactor"];

		std::ifstream trajectory_file(trajectory_path);
		if (!trajectory_file.is_open())
		{
			std::cout << "open trajectory file error: " << trajectory_path << std::endl;
			system("pause");
			exit(0);
		}
		std::stringstream sst;
		std::string line;

		frame_list_.reserve(1000);
		while (std::getline(trajectory_file,line))
		{
			sst.str(""); sst.clear();
			sst << line;
			double frameID;
			float x, y, z, qx, qy, qz, qw;
			sst >> frameID >> x >> y >> z >> qx >> qy >> qz >> qw;
			Frame current_frame(static_cast<unsigned int>(std::round(frameID)),Eigen::Vector3f(x, y, z), Eigen::Quaternion<float>(qw, qx, qy, qz));
			frame_list_.push_back(current_frame);
		}
		point_size_ = 3;
	}



	unsigned int generate_model()
	{
		for (int i = 0; i < frame_list_.size(); i++)
		//for (int i = 0; i < 840; i++)
		{
			//if (i % 10 != 0)
				//continue;
			std::stringstream sst;
			sst<<std::setw(7)<<std::setfill('0') << frame_list_[i].frameID_<<".png";
			cv::Mat depth = cv::imread(depth_image_path_ + "\\" + sst.str(), CV_LOAD_IMAGE_ANYDEPTH);
			cv::Mat rgb = cv::imread(rgb_image_path_ + "\\" + sst.str());

			if (depth.empty() || rgb.empty())
			{
				std::cout << "open image fail: " << depth_image_path_ + "\\" + sst.str() << std::endl;
				system("pause");
				exit(0);
			}
			point_cloud_mutex_.lock();
			point_cloud_.addPoints(rgb,depth,fx_,fy_,cx_,cy_,factor_,frame_list_[i].transform_camera_to_world_);
			point_cloud_mutex_.unlock();
			point_cloud_is_uptate = true;
			std::cout << frame_list_[i].frameID_ << "updated" << std::endl;
		}
		std::cout << "saving point cloud"<< std::endl;
		savePointCloud("frames\\pointCloud.vtx");
		std::cout << "save point cloud to " << "frames\\pointCloud.vtx" << std::endl;
	}



	void displayOpenGL()
	{
		glPushMatrix();
		if (point_cloud_is_uptate)
		{
			if (vertex_buffer_Id_)
			glDeleteBuffers(1, &vertex_buffer_Id_);
			vertex_buffer_Id_ = 0;
			glGenBuffers(1, &vertex_buffer_Id_);
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_Id_);         // for vertex coordinates
			point_cloud_mutex_.lock();
			glBufferData(GL_ARRAY_BUFFER, sizeof(Point) * point_cloud_.size(), point_cloud_.point_array(), GL_STATIC_DRAW);//wx-将创建好的点云数据传递给openGL的数据结构中
			point_cloud_mutex_.unlock();
			point_cloud_is_uptate = false;
		}
		if (vertex_buffer_Id_ == 0)
			return;
		glPushMatrix();
		glPointSize(point_size_);
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_Id_);
		glVertexPointer(3, GL_FLOAT, sizeof(Point), 0);
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Point), (const void*)(3 * sizeof(float)));
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glDrawArrays(GL_POINTS, 0, point_cloud_.size());
		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glPopMatrix();
	}

	int savePointCloud(std::string path)
	{
		return point_cloud_.savePointCloud_vtx(path);
	}

};