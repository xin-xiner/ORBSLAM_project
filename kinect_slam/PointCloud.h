#include <vector>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <Eigen\Geometry>
#include <Eigen\core>
struct Point
{
	float point[3];
	unsigned char color[4];

	Point(float x, float y, float z, uchar r, uchar g, uchar b)
	{
		point[0] = x; point[1] = y; point[2] = z;
		color[0] = b; color[1] = g; color[2] = r; color[3] = 1;
	}
	friend std::ostream& operator<<(std::ostream& o,Point& p)
	{
		o << std::fixed << std::setprecision(7) << p.point[0] << " " << p.point[1] << " " << p.point[2] << std::fixed << std::setprecision(0) << " " << (int)p.color[0] << " " << (int)p.color[1] << " " << (int)p.color[2];
		return o;
	}
};

class PointCloud
{
	
	std::vector<Point> points_;

public:
	int addPoints(cv::Mat& rgb, cv::Mat& depth, float fx, float fy, float cx, float cy,float factor, Eigen::Matrix4f& T)
	{
		points_.reserve(points_.size() + depth.cols*depth.rows);

		cv::Vec3b* rgb_data = (cv::Vec3b*)rgb.data;
		ushort* depth_data = (ushort*)depth.data;
		int height = depth.rows;
		int width = depth.cols;

		
		int points_count = 0;

		for (int h = 0; h < height; h++)
			for (int w = 0; w < width; w++)
		{
			float x, y, z;
			uchar r, g, b;

			int idx = h*width+w;
			if (idx >= width*height)
				continue;
			if (depth_data[idx] <= 0)//No depth information
				continue;
			points_count++;
			z = depth_data[idx] / factor;
			x = (w - cx) * z / fx;
			y = (h - cy) * z / fy;
			Eigen::Vector4f current_point(x, y, z, 1);
			current_point = T*current_point;
			current_point /= current_point(3, 0);
			x = current_point(0, 0);
			y = current_point(1, 0);
			z = current_point(2, 0);
			r = rgb_data[idx](0);
			g = rgb_data[idx](1);
			b = rgb_data[idx](2);


			points_.push_back(Point(x,y,z,r,g,b));
		}
		return points_count;
	}


	int size()
	{
		return points_.size();
	}

	Point* point_array()
	{
		if (points_.size() > 0)
			return &points_[0];
		else
			return 0;
	}

	int savePointCloud_vtx(std::string fileName)
	{
		std::ofstream point_file(fileName);
		for (int i = 0; i < points_.size(); i++)
		{
			point_file << points_[i] << std::endl;
		}
		point_file.close();
		return points_.size();
	}
};