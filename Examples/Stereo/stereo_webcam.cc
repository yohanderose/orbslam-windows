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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>

#include "../Utils/MyARViewer.h"
#include "../Utils/StereoRectification.h"
#include "../Utils/StereoSync.h"
#include "../Utils/CommandLine.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


/* Forward function declarations */
bool parseProgramArguments(int argc, const char *argv[]);
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
	if (!parseProgramArguments(argc, argv)) {
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

	string inputWindowTitle = "Rectified stereo input -- Press Esc to quit";
	namedWindow(inputWindowTitle, CV_WINDOW_NORMAL);
	for (long long i = 0; true; ++i)
	{
		long long ts = 0;
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
			if (i < 1800) continue;
			frameLeft = imread(paths[1], CV_LOAD_IMAGE_COLOR);
			frameRight = imread(paths[2], CV_LOAD_IMAGE_COLOR);
			ts = stoll(boost::filesystem::path(paths[0]).stem().generic_string());
		}

		// Get the timestamp
		__int64 curNow = ts ? ts : std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

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

		Mat both = Mat(max(frameLeftRect.rows, frameRightRect.rows), frameLeftRect.cols + frameRightRect.cols, frameLeftRect.type());
		Mat lRoi = both(Rect(0, 0, frameLeftRect.cols, frameLeftRect.rows));
		Mat rRoi = both(Rect(frameLeftRect.cols, 0, frameRightRect.cols, frameRightRect.rows));
		frameLeftRect.copyTo(lRoi);
		frameRightRect.copyTo(rRoi);
		imshow(inputWindowTitle, both);
		int c = waitKey(1);
		if (c == 27)
		{
			break;
		}

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

/* Parses the program arguments and sets the global variables using tclap */
bool parseProgramArguments(int argc, const char *argv[])
{
	bool result = true;
	CommandLine cmd(argc, argv);

	if (cmd.ContainsKey("v"))
		if (!cmd.GetStringValue("v", vocabPath)) result = false;

	if (cmd.ContainsKey("s"))
		if (!cmd.GetStringValue("s", settingsPath)) result = false;

	if (cmd.ContainsKey("L"))
	{
		if (cmd.GetIntValue("L", cameraIndexLeft))
		{
			webcamMode = true;
		}
		else if (!cmd.GetStringValue("L", filePathLeft))
		{
			result = false;
		}
	}

	if (cmd.ContainsKey("R"))
	{
		if (webcamMode)
		{
			if (!cmd.GetIntValue("R", cameraIndexRight)) result = false;
		}
		else
		{
			if (!cmd.GetStringValue("R", filePathRight)) result = false;
		}
	}

	if (cmd.ContainsKey("l"))
		loadMap = true;

	if (cmd.ContainsKey("r"))
	{
		vector<int> resolution;
		if (!cmd.GetMultiIntValue("r", resolution))
			result = false;
		else
		{
			width = resolution[0];
			height = resolution[1];
		}
	}

	if (cmd.ContainsKey("h"))
	{

	}

	return result;
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

