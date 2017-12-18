#pragma once


#include <thread>
#include <mutex>


namespace cv
{
	class Mat;
}
struct IKinectSensor;
struct ICoordinateMapper;
struct IDepthFrameReader;
struct IColorFrameReader;


class KinectRgbd
{
public:
	KinectRgbd();
	~KinectRgbd();
	bool IsValid();
	void StartColorStream(void(*colorFrameCb)(cv::Mat &cf));
	void StartDepthStream(void(*depthFrameCb)(cv::Mat &df));
	void StopColorStream();
	void StopDepthStream();

private:
	void StreamColor(void(*colorFrameCb)(cv::Mat &cf));
	void StreamDepth(void(*depthFrameCb)(cv::Mat &df));

	IKinectSensor *kSensor;
	ICoordinateMapper *cMapper;
	IColorFrameReader *cfReader;
	IDepthFrameReader *dfReader;
	bool valid;

	std::thread tColor;
	std::thread tDepth;
	std::mutex mtxC;
	std::mutex mtxD;
	bool stopColor;
	bool stopDepth;
	double min;
	double max;
};


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease);

