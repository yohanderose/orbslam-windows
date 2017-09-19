/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>

#include <System.h>
#include <tclap/CmdLine.h>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

#include "ViewerAR.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


int parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(string &filePath, string name, string val);


string vocabPath;
string settingsPath;

int cameraIndex;
int cameraFps;
int cameraWidth;
int cameraHeight;

bool loadMap;
bool bRGB;

VideoCapture cap;
Mat cameraPose, K, DistCoef;

ViewerAR viewerAR;
thread tViewer;

System *SLAM = NULL;


void setupViewerAR()
{
	viewerAR.SetSLAM(SLAM);
	cv::FileStorage fSettings(settingsPath, cv::FileStorage::READ);
	bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
	float fps = fSettings["Camera.fps"];
	viewerAR.SetFPS(fps);

	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];

	viewerAR.SetCameraCalibration(fx, fy, cx, cy);

	K = cv::Mat::eye(3, 3, CV_32F);
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(0, 2) = cx;
	K.at<float>(1, 2) = cy;

	DistCoef = cv::Mat::zeros(4, 1, CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if (k3 != 0)
	{
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}

	tViewer = thread(&ORB_SLAM2::ViewerAR::Run, &viewerAR);
}


int main(int argc, const char *argv[])
{
	// parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Warning] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	// Set up webcam
	cap = VideoCapture(cameraIndex);
	if (cameraFps > 0) cap.set(CV_CAP_PROP_FPS, cameraFps);
	if (cameraWidth > 0) cap.set(CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
	if (cameraHeight > 0) cap.set(CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);

	scaleCalib();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (loadMap)
	{
		SLAM = new System(vocabPath, settingsPath, System::MONOCULAR, false, true);
	}
	else
	{
		SLAM = new System(vocabPath, settingsPath, System::MONOCULAR, false, false);
	}

	if (SLAM == NULL)
	{
		cout << "SLAM is NULL" << endl;
		return EXIT_FAILURE;
	}

	// Initialise the AR viewer thread
	setupViewerAR();

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	Mat im;

	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

	while (true)
	{
		cap >> im;

		if (im.empty() || im.channels() != 3) continue;
		scaleIm(im);

		__int64 curNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackMonocular(im, curNow / 1000.0);

		Mat imu;
		int state = SLAM->GetTrackingState();
		vector<ORB_SLAM2::MapPoint*> vMPs = SLAM->GetTrackedMapPoints();
		vector<cv::KeyPoint> vKeys = SLAM->GetTrackedKeyPointsUn();

		cv::undistort(im, imu, K, DistCoef);
		if (bRGB)
			viewerAR.SetImagePose(imu, cameraPose, state, vKeys, vMPs);
		else
		{
			cv::cvtColor(imu, imu, CV_RGB2BGR);
			viewerAR.SetImagePose(imu, cameraPose, state, vKeys, vMPs);
		}

		// This will make a third window with the color images, you need to click on this then press any key to quit
		imshow("Image", im);
		if (waitKey(1) != 255)
		{
			break;
		}
	}

	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	cap.release();

	delete SLAM;
	return EXIT_SUCCESS;
}

int parseProgramArguments(int argc, const char *argv[])
{
	using namespace TCLAP;
	try {
		// set up the args
		CmdLine cmd("Runs ORB_SLAM2 with a monocular webcam", ' ', "0.1");
		ValueArg<string> vocabPathArg("v", "vocabPath", "Path to ORB vocabulary", false, "../ORBvoc.txt", "string");
		ValueArg<string> settingsPathArg("s", "settingsPath", "Path to webcam calibration and ORB settings yaml file", false, "../webcam.yaml", "string");
		ValueArg<int> cameraIndexArg("c", "camIndex", "Index of the webcam to use", false, 0, "integer");
		ValueArg<int> cameraFpsArg("f", "fps", "Desired framerate of the camera", false, 0, "integer");
		ValueArg<int> cameraWidthArg("W", "width", "Desired width of the camera", false, 0, "integer");
		ValueArg<int> cameraHeightArg("H", "height", "Desired height of the camera", false, 0, "integer");
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(cameraIndexArg);
		cmd.add(cameraFpsArg);
		cmd.add(cameraWidthArg);
		cmd.add(cameraHeightArg);
		cmd.add(loadMapArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		cameraIndex = cameraIndexArg.getValue();
		cameraFps = cameraFpsArg.getValue();
		cameraWidth = cameraWidthArg.getValue();
		cameraHeight = cameraHeightArg.getValue();
		loadMap = loadMapArg.getValue();

	} // catch any exceptions 
	catch (ArgException &e) {
		cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
		return 1;
	}
	return 0;
}


/* Scales the calibration data to the input video resolution */
void scaleCalib()
{
	// open the original calibration
	FileStorage fsSettings(settingsPath, FileStorage::READ);

	// get the calibration and input resolutions
	double calibWidth = double(fsSettings["Image.width"]);
	double calibHeight = double(fsSettings["Image.height"]);
	double videoWidth = cameraWidth > 0 ? cameraWidth : cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double videoHeight = cameraHeight > 0 ? cameraHeight : cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	// only continue if the calibration file actually had the calibration resolution inside
	if (calibWidth <= 0 || calibHeight <= 0)
	{
		cout << "Image.width and Image.height not found in calibration file, scaling is impossible." << endl;
		return;
	}

	// calculate scaling
	double sw = videoWidth / calibWidth;
	double sh = videoHeight / calibHeight;

	// vertical and horizontal scaling must be equal and not 1
	if (sw == 1.0 || sw != sh)
	{
		cout << "Calibration file does not require scaling, or is unable to be scaled." << endl;
		return;
	}
	else
	{
		cout << "Scaling calibration file by factor: " << sw << endl;
	}
	// delete any traces of the previously used scaled calibration file
	const char *tempSettingsPath = "s-calib.yaml";
	remove(tempSettingsPath);

	// copy the calibration file
	ifstream src(settingsPath, ios::binary);
	ofstream dst(tempSettingsPath, ios::binary);
	dst << src.rdbuf();
	dst.close();

	// edit the new file
	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];
	settingsFileUpdate(string(tempSettingsPath), "Camera.fx", to_string(fx * sw));
	settingsFileUpdate(string(tempSettingsPath), "Camera.fy", to_string(fy * sw));
	settingsFileUpdate(string(tempSettingsPath), "Camera.cx", to_string(cx * sw));
	settingsFileUpdate(string(tempSettingsPath), "Camera.cy", to_string(cy * sw));

	// overwrite the settings path
	settingsPath = tempSettingsPath;
}
void scaleIm(Mat &im)
{
	if (cameraWidth != im.cols && cameraWidth != -1)
		resize(im, im, Size(cameraWidth, cameraHeight));
}
void settingsFileUpdate(string &filePath, string name, string val)
{
	remove((filePath + ".tmp").c_str());

	ifstream inFile(filePath);
	ofstream outFile(filePath + ".tmp");

	string line;
	if (inFile.is_open())
	{
		if (outFile.is_open())
		{
			while (!inFile.eof())
			{
				getline(inFile, line);
				if (line.compare(0, name.length(), name) == 0)
				{
					outFile << name << ": " << val << endl;
				}
				else
				{
					outFile << line << endl;
				}
			}
			outFile.close();
		}
		inFile.close();
	}
	remove(filePath.c_str());
	rename((filePath + ".tmp").c_str(), filePath.c_str());
}
