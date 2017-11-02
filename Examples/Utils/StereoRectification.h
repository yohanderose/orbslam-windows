#pragma once

#include <string>

#include <opencv2/core.hpp>


class StereoRectification
{
public:
	StereoRectification(std::string settingsPath);
	~StereoRectification();

	void stereoRectify(cv::Mat &leftSrc, cv::Mat &rightSrc, cv::Mat &leftDst, cv::Mat &rightDst);

private:
	cv::FileStorage fsSettings;
	cv::Mat M1l, M2l, M1r, M2r;
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
};

