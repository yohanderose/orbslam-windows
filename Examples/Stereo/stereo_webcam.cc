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
#include <thread>

#include <System.h>
#include <tclap/CmdLine.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include "../Utils/MyARViewer.h"
#include "../Utils/StereoRectification.h"
#include "../Utils/StereoSync.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


/* Forward function declarations */
int parseProgramArguments(int argc, const char *argv[]);
bool scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);
void getFrameAsync(cv::VideoCapture &cap, int idx);

/* Program arg globals */
string vocabPath;
string settingsPath;
string filePathLeft;
string filePathRight;
int cameraIndexLeft;
int cameraIndexRight;
int width;
int height;
bool loadMap;
bool webcamMode;

/* Misc globals */
Mat cameraPose;

/* Stereo globals */
Mat frameLeft, frameRight;

/* Main */
int main(int argc, const char *argv[])
{
	// Parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Error] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	System *SLAM = NULL;

	FileStorage fsSettings(settingsPath, FileStorage::READ);
	int calibWidth = fsSettings["Camera.width"];
	int calibHeight = fsSettings["Camera.height"];

	StereoSync sync;
	VideoCapture capLeft, capRight;
	if (webcamMode)
	{
		// Set up video sources
		capLeft = VideoCapture(cameraIndexLeft);
		capRight = VideoCapture(cameraIndexRight);
		if (!capLeft.isOpened() || !capRight.isOpened())
		{
			cerr << "[Error] -- Failed to open camera sources -- exiting." << endl;
			return EXIT_FAILURE;
		}

		capLeft.set(CAP_PROP_FRAME_WIDTH, calibWidth);
		capLeft.set(CAP_PROP_FRAME_HEIGHT, calibHeight);
		capRight.set(CAP_PROP_FRAME_WIDTH, calibWidth);
		capRight.set(CAP_PROP_FRAME_HEIGHT, calibHeight);
	}
	else
	{
		// Set up stereo frame syncer
		sync = StereoSync(filePathLeft);
	}

	// If we are trying to resize, try to scale the calibration file, if that works, resize rectified images as they arrive
	bool sizeArgsSet = false;
	if (width > 0 && height > 0)
	{
		sizeArgsSet = scaleCalib();
	}
	if (!sizeArgsSet)
	{
		width = calibWidth;
		height = calibHeight;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (loadMap)
	{
		SLAM = new System(vocabPath, settingsPath, System::STEREO, true, true);
	}
	else
	{
		SLAM = new System(vocabPath, settingsPath, System::STEREO, true, false);
	}

	// Set up the opengl AR frame
	MyARViewer viewer(SLAM, settingsPath, width, height);

	// Set up stereo rectifier
	StereoRectification rectifier(settingsPath);

	// Main loop
	Mat frameLeftRect, frameRightRect;
	while (true)
	{
		// Get the frames
		if (webcamMode)
		{
			// TODO: bad idea creating new thread for each frame
			thread leftTh = thread(getFrameAsync, capLeft, 0);
			thread rightTh = thread(getFrameAsync, capRight, 1);
			leftTh.join();
			rightTh.join();
		}
		else
		{
			vector<string> paths;
			sync.GetSyncedFramePaths(paths);
			frameLeft = imread(paths[0], CV_LOAD_IMAGE_COLOR);
			frameRight = imread(paths[1], CV_LOAD_IMAGE_COLOR);
		}

		// Get the timestamp
		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Check that it is all good
		if (frameLeft.empty() || frameRight.empty())
		{
			break;
		}

		// Rectify images
		rectifier.stereoRectify(frameLeft, frameRight, frameLeftRect, frameRightRect);

		// Resize them if necessary
		if (sizeArgsSet) 
		{
			scaleIm(frameLeftRect);
			scaleIm(frameRightRect);
		}

		imshow("l", frameLeftRect);
		imshow("r", frameRightRect);
		waitKey(1);

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackStereo(frameLeftRect, frameRightRect, curNow / 1000.0);
		viewer.renderARFrame(cameraPose, frameLeftRect);
	}

	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	capLeft.release();
	capRight.release();

	delete SLAM;
	return EXIT_SUCCESS;
}

bool is_number(const string& s)
{
	return !s.empty() && find_if(s.begin(),
		s.end(), [](char c) { return !isdigit(c); }) == s.end();
}

/* Parses the program arguments and sets the global variables using tclap */
int parseProgramArguments(int argc, const char *argv[])
{
	using namespace TCLAP;
	try {
		// set up the args
		CmdLine cmd("Runs ORB_SLAM2 with stereo webcam or synchronized video frame directories", ' ', "0.1");
		ValueArg<string> vocabPathArg("v", "vocabPath", "Path to ORB vocabulary", false, "../ORBvoc.bin", "string");
		ValueArg<string> settingsPathArg("s", "settingsPath", "Path to webcam calibration and ORB settings yaml file", false, "../webcam.yaml", "string");
		ValueArg<string> sourceLeftArg("L", "sourceLeft", "Directory or camera-index of right camera data source", false, "-1", "string");
		ValueArg<string> sourceRightArg("R", "sourceRight", "Directory or camera-index of right camera data source", false, "-1", "string");
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);
		ValueArg<int> widthArg("W", "resizeWidth", "Width to resize the video to", false, -1, "integer");
		ValueArg<int> heightArg("H", "resizeHeight", "Height to resize the video to", false, -1, "integer");

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(sourceLeftArg);
		cmd.add(sourceRightArg);
		cmd.add(loadMapArg);
		cmd.add(widthArg);
		cmd.add(heightArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		filePathRight = sourceRightArg.getValue();
		filePathLeft = sourceLeftArg.getValue();
		loadMap = loadMapArg.getValue();
		width = widthArg.getValue();
		height = heightArg.getValue();

		if (is_number(filePathLeft) && is_number(filePathRight))
		{
			webcamMode = true;
			cameraIndexLeft = atoi(filePathLeft.c_str());
			cameraIndexRight = atoi(filePathRight.c_str());
		}
		else
		{
			webcamMode = false;
		}

	} // catch any exceptions 
	catch (ArgException &e) {
		return 1;
	}
	return 0;
}

void getFrameAsync(cv::VideoCapture &cap, int idx)
{
	if (idx == 0)
	{
		cap >> frameLeft;
	}
	else if (idx == 1)
	{
		cap >> frameRight;
	}
}

/* Scales the calibration data to the input video resolution */
bool scaleCalib()
{
	// open the original calibration
	FileStorage fsSettings(settingsPath, FileStorage::READ);

	// get the calibration and input resolutions
	double calibWidth = fsSettings["Camera.width"];
	double calibHeight = fsSettings["Camera.height"];

	// only continue if the calibration file actually had the calibration resolution inside
	if (calibWidth <= 0 || calibHeight <= 0)
	{
		cout << "Camera.width and Camera.height not found in calibration file, scaling is impossible." << endl;
		return false;
	}

	// calculate scaling
	double sw = width / calibWidth;
	double sh = height / calibHeight;

	// vertical and horizontal scaling must be equal and not 1
	if (width == calibWidth || sw != sh)
	{
		cout << "Calibration file does not require scaling, or is unable to be scaled." << endl;
		return false;
	}
	else
	{
		cout << "Scaling calibration file by factor: " << sw << endl;
	}

	// delete any traces of the previously used scaled calibration file
	const char *tempSettingsPath = "s-calib.yaml";
	remove(tempSettingsPath);

	// copy the calibration file
	std::ifstream src(settingsPath, std::ios::binary);
	std::ofstream dst(tempSettingsPath, std::ios::binary);
	dst << src.rdbuf();
	dst.close();

	// edit the new file
	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];
	float bf = fsSettings["Camera.bf"];
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.fx", std::to_string(fx * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.fy", std::to_string(fy * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.cx", std::to_string(cx * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.cy", std::to_string(cy * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.bf", std::to_string((bf / fx) * (sw * fx)));

	// overwrite the settings path
	settingsPath = tempSettingsPath;
	return true;
}
void scaleIm(Mat &im)
{
	if (width != im.cols || height != im.rows)
		resize(im, im, Size(width, height), 0.0, 0.0, CV_INTER_AREA);
}
void settingsFileUpdate(std::string &filePath, std::string name, std::string val)
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

