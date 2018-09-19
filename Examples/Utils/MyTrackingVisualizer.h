#pragma once


#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <System.h>


class MyTrackingVisualizer
{
	std::string mModelFilename;
	std::vector<cv::Vec4f> mPoints;
	std::vector<cv::Vec3i> mColors;
	cv::Mat mR;
	cv::Mat mt;
	bool mWindowAlive;
	bool mModelAligned;
	void readFromPlyFile();
public:
	MyTrackingVisualizer();
	MyTrackingVisualizer(const std::string &modelFilename);
	bool alignModel(const cv::Mat &frame, const cv::Mat &K);
	cv::Mat getReferencePose();
};

