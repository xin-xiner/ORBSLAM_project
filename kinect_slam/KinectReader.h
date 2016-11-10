#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <Kinect.h>
#include <iostream>


#define IMAGE_SCALE 0.3

class KinectReader
{
private:
	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	ColorSpacePoint*        m_pColorCoordinates;//This should be the ColorSpacePoint
	DepthSpacePoint*        m_pDepthCoordinates;//This should be the ColorSpacePoint
	RGBQUAD*                m_pColorRGBX;
	CameraIntrinsics intrinsics = {};
	// Frame reader
	IMultiSourceFrameReader*m_pMultiSourceFrameReader;
	void createFrame(
		const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
		const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
		cv::Mat& depth, cv::Mat& rgb
		);


	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

public:
	int sizeType;
	signed long long timeStamp;
	
	cv::Matx33f K;
	KinectReader();
	bool getNextFrame(cv::Mat& depth, cv::Mat& rgb);
	bool close();

};

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}



void normalizeDepthImage(cv::Mat& depth, cv::Mat& disp);