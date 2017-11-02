#include "StereoRectification.h"

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;


StereoRectification::StereoRectification(string settingsPath)
{
	fsSettings = FileStorage(settingsPath, FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
	}

	fsSettings["M1"] >> K_l;
	fsSettings["M2"] >> K_r;

	fsSettings["P1"] >> P_l;
	fsSettings["P2"] >> P_r;

	fsSettings["R1"] >> R_l;
	fsSettings["R2"] >> R_r;

	fsSettings["D1"] >> D_l;
	fsSettings["D2"] >> D_r;

	int rows_l = fsSettings["Lheight"];
	int cols_l = fsSettings["Lwidth"];
	int rows_r = fsSettings["Rheight"];
	int cols_r = fsSettings["Rwidth"];

	if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
		rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
	{
		cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
	}

	initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), Size(cols_l, rows_l), CV_32F, M1l, M2l);
	initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), Size(cols_r, rows_r), CV_32F, M1r, M2r);
}

StereoRectification::~StereoRectification()
{
}

void StereoRectification::stereoRectify(Mat &leftSrc, Mat &rightSrc, Mat &leftDst, Mat &rightDst)
{
	remap(leftSrc, leftDst, M1l, M2l, INTER_LINEAR);
	remap(rightSrc, rightDst, M1r, M2r, INTER_LINEAR);
}

