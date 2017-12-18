#include <iostream>
#include <thread>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Kinect.h>

#include "KinectRgbd.h"


using namespace std;


KinectRgbd::KinectRgbd()
	: kSensor(NULL), cMapper(NULL), cfReader(NULL), dfReader(NULL), valid(false), stopDepth(false), stopColor(false)
{
	unique_lock<mutex> lc(mtxC);
	unique_lock<mutex> ld(mtxD);
	// Get sensor
	HRESULT hr = GetDefaultKinectSensor(&kSensor);
	if (SUCCEEDED(hr))
	{
		hr = kSensor->Open();
	}

	// Get coordinate mapper
	if (SUCCEEDED(hr))
	{
		hr = kSensor->get_CoordinateMapper(&cMapper);
	}

	// Get color frame source/reader, release source
	IColorFrameSource *cfSource = NULL;
	if (SUCCEEDED(hr))
	{
		hr = kSensor->get_ColorFrameSource(&cfSource);
	}
	if (SUCCEEDED(hr))
	{
		hr = cfSource->OpenReader(&cfReader);
	}
	SafeRelease(cfSource);

	// Get depth frame source/reader, release source
	IDepthFrameSource *dfSource = NULL;
	if (SUCCEEDED(hr))
	{
		hr = kSensor->get_DepthFrameSource(&dfSource);
	}
	if (SUCCEEDED(hr))
	{
		UINT16 mn, mx;
		dfSource->get_DepthMinReliableDistance(&mn);
		dfSource->get_DepthMaxReliableDistance(&mx);
		min = double(mn);
		max = double(mx);
		cout << "min: " << min << " max: " << max << endl;
		hr = dfSource->OpenReader(&dfReader);
	}
	SafeRelease(dfSource);

	if (FAILED(hr))
	{
		cerr << "Failed to detect default Kinect" << endl;
	}
	else
	{
		valid = true;
	}
}

KinectRgbd::~KinectRgbd()
{
	unique_lock<mutex> lc(mtxC);
	unique_lock<mutex> ld(mtxD);

	StopColorStream();
	StopDepthStream();
	tColor.join();
	tDepth.join();

	// Release readers, mapper, and sensor
	SafeRelease(cfReader);
	SafeRelease(dfReader);
	SafeRelease(cMapper);

	kSensor->Close();
	SafeRelease(kSensor);
}

bool KinectRgbd::IsValid()
{
	return valid;
}

void KinectRgbd::StartColorStream(void(*colorFrameCb)(cv::Mat &cf))
{
	tColor = thread(&KinectRgbd::StreamColor, this, colorFrameCb);
}

void KinectRgbd::StartDepthStream(void(*depthFrameCb)(cv::Mat &df))
{
	tDepth = thread(&KinectRgbd::StreamDepth, this, depthFrameCb);
}

void KinectRgbd::StopColorStream()
{
	unique_lock<mutex> lock(mtxC);
	stopColor = true;
}

void KinectRgbd::StopDepthStream()
{
	unique_lock<mutex> lock(mtxD);
	stopDepth = true;
}

void KinectRgbd::StreamColor(void(*colorFrameCb)(cv::Mat &cf))
{
	while (true)
	{
		{
			unique_lock<mutex> lock(mtxC);
			if (stopColor)
				break;
		}

		if (cfReader == NULL)
			break;

		// check for a color frame
		IColorFrame *cf = NULL;
		HRESULT hr = cfReader->AcquireLatestFrame(&cf);
		if (cf != NULL)
		{
			IFrameDescription *cfDescription = NULL;
			if (SUCCEEDED(hr))
			{
				hr = cf->get_FrameDescription(&cfDescription);
			}

			int cWidth, cHeight;
			UINT bpp;
			if (SUCCEEDED(hr))
			{
				hr = cfDescription->get_Width(&cWidth);
				hr = cfDescription->get_Height(&cHeight);
				hr = cfDescription->get_BytesPerPixel(&bpp);
			}
			SafeRelease(cfDescription);

			UINT capacity;
			BYTE *cfBuffer = NULL;
			if (SUCCEEDED(hr))
			{
				cf->AccessRawUnderlyingBuffer(&capacity, &cfBuffer);
			}

			if (SUCCEEDED(hr))
			{
				// generate opencv rgb cv::Mat
				cv::Mat yuv(cHeight, cWidth, CV_8UC2, cfBuffer, bpp * cWidth);
				cv::Mat rgb;
				cvtColor(yuv, rgb, CV_YUV2RGB_YUYV);

				// do the thing with the cv::Mat
				colorFrameCb(rgb);
			}
		}
		SafeRelease(cf);

	}
}

void KinectRgbd::StreamDepth(void(*depthFrameCb)(cv::Mat &df))
{
	while (true)
	{
		{
			unique_lock<mutex> lock(mtxD);
			if (stopDepth)
				break;
		}

		if (dfReader == NULL || cMapper == NULL)
			break;

		// check for a color frame
		IDepthFrame *df = NULL;
		HRESULT hr = dfReader->AcquireLatestFrame(&df);
		if (df != NULL)
		{
			IFrameDescription *dfDescription = NULL;
			if (SUCCEEDED(hr))
			{
				hr = df->get_FrameDescription(&dfDescription);
			}

			int dWidth, dHeight;
			UINT bpp;
			if (SUCCEEDED(hr))
			{
				hr = dfDescription->get_Width(&dWidth);
				hr = dfDescription->get_Height(&dHeight);
				hr = dfDescription->get_BytesPerPixel(&bpp);
			}
			SafeRelease(dfDescription);

			UINT dfCapacity;
			UINT16* dfBuffer = NULL;
			hr = df->AccessUnderlyingBuffer(&dfCapacity, &dfBuffer);

			// Test two, mapping rgb to depth (good)
			int cWidth = 1920, cHeight = 1080;
			UINT ndSpacePoints = cWidth * cHeight;
			DepthSpacePoint *dSpacePoints = new DepthSpacePoint[ndSpacePoints];
			if (SUCCEEDED(hr))
			{
				hr = cMapper->MapColorFrameToDepthSpace(dWidth * dHeight, dfBuffer, ndSpacePoints, dSpacePoints);
			}
			if (SUCCEEDED(hr))
			{
				cv::Mat rDepth, d;
				// generate opencv depth cv::Mat
				d = cv::Mat(dHeight, dWidth, CV_16SC1, dfBuffer, sizeof(UINT16) * dWidth);

				// generate opencv registered rgb cv::Mat
				rDepth = cv::Mat::zeros(cHeight, cWidth, CV_16SC1);
				float nInf = -numeric_limits<float>::infinity();
				for (int r = 0, i = 0; r < cHeight; ++r)
				{
					for (int c = 0; c < cWidth; ++c, ++i)
					{
						DepthSpacePoint dsp = dSpacePoints[i];
						int dspX = int(dsp.X + 0.5f), dspY = int(dsp.Y + 0.5f);
						if (dspX >= 0 && dspX < dWidth &&
							dspY >= 0 && dspY < dHeight)
						{
							UINT16 dspVal = d.at<UINT16>(dspY, dspX);
							rDepth.at<UINT16>(r, c) = dspVal;
						}
					}
				}
				// threshold near values
				cv::threshold(rDepth, rDepth, min, 255.0, cv::THRESH_TOZERO);
				//imshow("rDepth", rDepth); cv::waitKey();

				// threshold far values
				cv::threshold(rDepth, rDepth, max, 255.0, cv::THRESH_TOZERO_INV);
				//imshow("rDepth", rDepth); cv::waitKey();

				// scale into byte space
				//double scale = 1.0 / (0xFFFF);
				//rDepth.convertTo(rDepth, CV_32FC1, scale);
				//imshow("rDepth", rDepth); cv::waitKey();

				// do the thing with the frame
				depthFrameCb(rDepth);
			}
			delete[] dSpacePoints;
		}
		SafeRelease(df);

	}
}


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

