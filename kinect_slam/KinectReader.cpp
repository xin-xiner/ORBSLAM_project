#include "KinectReader.h"
#include <opencv2\imgproc\imgproc.hpp>




KinectReader::KinectReader()
	:
	m_pKinectSensor(NULL),
	m_pCoordinateMapper(NULL),
	m_pMultiSourceFrameReader(NULL),
	m_pColorRGBX(NULL),
	m_pColorCoordinates(NULL),
	m_pDepthCoordinates(NULL)
{

	
	sizeType = 1;
	HRESULT hr;
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pColorCoordinates = new ColorSpacePoint[cDepthWidth * cDepthHeight];
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		exit(-9);
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
				&m_pMultiSourceFrameReader);
		}
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		exit(-9);
	}

	cv::Mat depthImg, rgbImg;
	while (!getNextFrame(depthImg, rgbImg))
	{
	}


}


bool KinectReader::getNextFrame(cv::Mat& depth, cv::Mat& rgb)
{
	if (!m_pMultiSourceFrameReader)
	{
		return false;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;


	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}
		SafeRelease(pColorFrameReference);
	}


	if (SUCCEEDED(hr))
	{
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		// get depth frame data

		hr = pDepthFrame->get_RelativeTime(&timeStamp);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		// get body index frame data



		if (SUCCEEDED(hr))
		{
			createFrame(pDepthBuffer, nDepthWidth, nDepthHeight,
				pColorBuffer, nColorWidth, nColorHeight,
				depth, rgb);
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pMultiSourceFrame);

	return SUCCEEDED(hr);
}


void KinectReader::createFrame(
	const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	cv::Mat& depth, cv::Mat& rgb
	)
{
	depth = cv::Mat(cDepthHeight, cDepthWidth, CV_16U);
	rgb = cv::Mat(cDepthHeight, cDepthWidth, CV_8UC3);//The size should equal with depth map！！！！！！！！！！！！！！！！！
	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pColorCoordinates &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight)
		)
	{
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pColorCoordinates);
		HRESULT hrr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, m_pDepthCoordinates);
		//HRESULT hrr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pDepthCoordinates);
		
		//hr = 10;
		if (SUCCEEDED(hr)&&SUCCEEDED(hrr))
		{
			//**************************************************
			if (sizeType == 0)
			{
				ushort* depthData = depth.ptr<ushort>();
				for (int i = 0; i < (nDepthWidth*nDepthHeight); i++)
				{
					depthData[i] = 0;
				}
				cv::Vec3b* colorData = rgb.ptr<cv::Vec3b>();
				for (int depthIndex = 0; depthIndex < (nDepthWidth*nDepthHeight); ++depthIndex)
				{

					ColorSpacePoint p = m_pColorCoordinates[depthIndex];
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);

						/*	if (colorX + (colorY * nColorWidth))
								std::cout << depthIndex << std::endl;*/

						if ((colorX >= 0 && colorX < nColorWidth) && (colorY >= 0 && colorY < nColorHeight))
						{
							RGBQUAD pixel = m_pColorRGBX[colorX + (colorY * nColorWidth)];
							depthData[depthIndex] = pDepthBuffer[depthIndex];
							colorData[depthIndex] = cv::Vec3b(pixel.rgbBlue, pixel.rgbGreen, pixel.rgbRed);
						}
					}
				}
				m_pCoordinateMapper->GetDepthCameraIntrinsics(&intrinsics);


				float focalLengthX = intrinsics.FocalLengthX;// / (depth.cols);// cColorWidth*IMAGE_SCALE);
				float focalLengthY = intrinsics.FocalLengthY;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
				float principalPointX = intrinsics.PrincipalPointX;// / (depth.cols);//cColorWidth*IMAGE_SCALE);
				float principalPointY = intrinsics.PrincipalPointY;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
				K = cv::Matx33f(focalLengthX, 0, principalPointX,
					0, focalLengthY, principalPointY,
					0, 0, 1
					);
			}


			//*******************************************
			if (sizeType == 1)
			{
				depth = cv::Mat(cColorHeight, cColorWidth, CV_16U);
				rgb = cv::Mat(cColorHeight, cColorWidth, CV_8UC3);
				ushort* depthData = depth.ptr<ushort>();
				for (int i = 0; i < (cColorWidth*cColorHeight); i++)
				{
					depthData[i] = 0;
				}
				cv::Vec3b* colorData = rgb.ptr<cv::Vec3b>();
				for (int colorIndex = 0; colorIndex < (cColorWidth*cColorHeight); ++colorIndex)
				{

					DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int depthX = static_cast<int>(p.X + 0.5f);
						int depthY = static_cast<int>(p.Y + 0.5f);

						if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
						{
							UINT16 pixel = pDepthBuffer[depthX + (depthY * nDepthWidth)];
							depthData[colorIndex] = pDepthBuffer[depthX + (depthY * nDepthWidth)];
							/*RGBQUAD rgbpixel = m_pColorRGBX[colorIndex];
							colorData[depthX + (depthY * nDepthWidth)] = cv::Vec3b(rgbpixel.rgbBlue, rgbpixel.rgbGreen, rgbpixel.rgbRed);*/
						}
					}
					RGBQUAD rgbpixel = m_pColorRGBX[colorIndex];
					colorData[colorIndex] = cv::Vec3b(rgbpixel.rgbBlue, rgbpixel.rgbGreen, rgbpixel.rgbRed);
				}

				//
				///*for (int i = 0; i < nDepthWidth*nDepthHeight; i++)
				//{
				//	logLine(depthData[i]);
				//}*/
				
				cv::resize(depth, depth, cv::Size(640, 360), 0, 0,cv::INTER_NEAREST);
				cv::resize(rgb, rgb, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
				cv::flip(depth, depth, 1);
				cv::flip(rgb, rgb, 1);
				////cDepthHeight
				//cv::resize(depth, depth, cv::Size(cDepthHeight, cDepthWidth));

				///*
				//for (int i = 0; i < nDepthWidth*nDepthHeight; i++)
				//{
				//	if (depthData[i]>50000)
				//		depthData[i] = 0;
				//}*/
				////depthData = depth.ptr<ushort>();
				//cv::resize(rgb, rgb, cv::Size(cDepthHeight, cDepthWidth));




				float focalLengthX = 1052.78555;// / (depth.cols);// cColorWidth*IMAGE_SCALE);
				float focalLengthY = 1050.85892;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
				float principalPointX = 945.27896;// / (depth.cols);//cColorWidth*IMAGE_SCALE);
				float principalPointY = 520.37130;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
				K = cv::Matx33f(focalLengthX, 0, principalPointX,
					0, focalLengthY, principalPointY,
					0, 0, 1
					);
			}

			//m_pCoordinateMapper->GetDepthCameraIntrinsics(&intrinsics);
			//float focalLengthX = cColorWidth*intrinsics.FocalLengthX / cDepthWidth;// / (depth.cols);// cColorWidth*IMAGE_SCALE);
			//float focalLengthY = cColorHeight*intrinsics.FocalLengthY / cDepthHeight;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
			//float principalPointX = cColorWidth*intrinsics.PrincipalPointX / cDepthWidth;// / (depth.cols);//cColorWidth*IMAGE_SCALE);
			//float principalPointY = cColorHeight*intrinsics.PrincipalPointY / cDepthHeight;// / (depth.rows);//cColorHeight*IMAGE_SCALE);
			//K = cv::Matx33f(focalLengthX, 0, principalPointX,
			//	0, focalLengthY, principalPointY,
			//	0, 0, 1
			//	);
			


			//**************************************************
			/*for (int depthIndex = 0; depthIndex < (nDepthWidth*nDepthHeight); ++depthIndex)
			{
			depthData[depthIndex] = pDepthBuffer[depthIndex];
			}*/
			//for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
			//{
			//	//if (depthData[colorIndex]==0)
			//	//colorData[colorIndex] /= 3;
			//}


			/*Focal Length : fc = [1049.86674   1048.02687] �[1.89372   1.99448]
				Principal point : cc = [945.27896   520.37130] �[1.47574   0.81709]
			Skew : alpha_c = [0.00000] �[0.00000] = > angle of pixel axes = 90.00000 � 0.00000 degrees
			   Distortion : kc = [0.05251 - 0.04806   0.00049   0.00192  0.00000] �[0.00139   0.00216   0.00024   0.00031  0.00000]
							Pixel error : err = [0.17650   0.17013]

			Focal Length : fc = [1052.78555   1050.85892] �[1.30916   1.28205]
			Principal point : cc = [940.82803   521.59128] �[1.26246   0.97861]
		Skew : alpha_c = [0.00000] �[0.00000] = > angle of pixel axes = 90.00000 � 0.00000 degrees
			Distortion : kc = [0.05229 - 0.04983 - 0.00093 - 0.00145  0.00000] �[0.00231   0.00469   0.00035   0.00045  0.00000]
						Pixel error : err = [0.30835   0.25174]

			Focal Length : fc = [1058.64954   1055.90759] �[2.90873   2.96334]
			Principal point : cc = [939.91086   524.59582] �[3.93990   2.94518]
		Skew : alpha_c = [0.00000] �[0.00000] = > angle of pixel axes = 90.00000 � 0.00000 degrees
			Distortion : kc = [0.03369 - 0.00669 - 0.00192 - 0.00146  0.00000] �[0.00756   0.02073   0.00104   0.00144  0.00000]
						Pixel error : err = [0.32443   0.48491]

			fx 1060.707250708333, cx 956.354471815484
			fy 1058.608326305465, cy 518.9784429882449*/
			

		}

	}
}



bool KinectReader::close()
{
	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	// done with frame reader
	SafeRelease(m_pMultiSourceFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
	return true;
}



void normalizeDepthImage(cv::Mat& depth, cv::Mat& disp)
{

	ushort* depthData = (ushort*)depth.data;
	int width = depth.cols;
	int height = depth.rows;

	ushort max = *std::max_element(depthData, depthData + width*height);
	ushort min = *std::min_element(depthData, depthData + width*height);

	disp = cv::Mat(depth.size(), CV_8U);
	uchar* dispData = disp.data;


	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
		{
			dispData[i*width + j] = (((double)(depthData[i*width + j] - min)) / ((double)(max - min))) * 255;
		}

	//logMatrix(dispData, height, width);
}
