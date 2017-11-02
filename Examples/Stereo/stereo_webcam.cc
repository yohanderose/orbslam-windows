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
void scaleCalib();
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
bool loadMap;
bool webcamMode;

/* Misc globals */
VideoCapture capLeft, capRight;
Mat cameraPose;
System *SLAM = NULL;

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

	// Set up video source
	FileStorage fsSettings(settingsPath, FileStorage::READ);
	int width = fsSettings["Camera.width"];
	int height = fsSettings["Camera.height"];

	if (webcamMode)
	{
		capLeft = VideoCapture(cameraIndexLeft);
		capRight = VideoCapture(cameraIndexRight);
	}
	else
	{
		//capLeft = VideoCapture(filePathLeft);
		//capRight = VideoCapture(filePathRight);
	}

	// Check it's good
	if (!capLeft.isOpened() || !capRight.isOpened())
	{
		//cerr << "[Error] -- Failed to open camera sources -- exiting." << endl;
		//return EXIT_FAILURE;
	}

	else if (webcamMode) // if they were set, and we are using a webcam, we can try set the webcam resolution
	{
		//FileStorage fsSettings(settingsPath, FileStorage::READ);
		//width = fsSettings["Camera.width"];
		//height = fsSettings["Camera.height"];

		//capLeft.set(CAP_PROP_FRAME_WIDTH, width);
		//capRight.set(CAP_PROP_FRAME_WIDTH, width);
		//capLeft.set(CAP_PROP_FRAME_HEIGHT, height);
		//capRight.set(CAP_PROP_FRAME_HEIGHT, height);
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

	// Set up stereo frame syncer
	StereoSync sync(filePathLeft, filePathRight);

	// Main loop
	Mat frameLeftRect, frameRightRect;
	while (true)
	{
		// Get the frames
		if (webcamMode)
		{
			thread leftTh = thread(getFrameAsync, capLeft, 0);
			thread rightTh = thread(getFrameAsync, capRight, 1);
			leftTh.join();
			rightTh.join();
		}
		else
		{
			string l, r;
			sync.GetSyncedFramePaths(l, r);
			frameLeft = imread(l, CV_LOAD_IMAGE_COLOR);
			frameRight = imread(r, CV_LOAD_IMAGE_COLOR);
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
		CmdLine cmd("Runs ORB_SLAM2 with a monocular video file", ' ', "0.1");
		ValueArg<string> vocabPathArg("v", "vocabPath", "Path to ORB vocabulary", false, "../ORBvoc.bin", "string");
		ValueArg<string> settingsPathArg("s", "settingsPath", "Path to webcam calibration and ORB settings yaml file", false, "../webcam.yaml", "string");
		ValueArg<string> sourceLeftArg("L", "cameraIndexLeft", "Index of the left camera to use", false, "-1", "string");
		ValueArg<string> sourceRightArg("R", "cameraIndexRight", "Index of the right camera to use", false, "-1", "string");
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(sourceLeftArg);
		cmd.add(sourceRightArg);
		cmd.add(loadMapArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		filePathRight = sourceRightArg.getValue();
		filePathLeft = sourceLeftArg.getValue();
		loadMap = loadMapArg.getValue();

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

