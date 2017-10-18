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
#include <opencv2/highgui.hpp>

#include "../Utils/MyARViewer.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


/* Forward function declarations */
int parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);

/* Program arg globals */
string vocabPath;
string settingsPath;
string filePath;
int cameraIndex;
int width;
int height;
bool loadMap;
bool webcamMode;

/* Misc globals */
VideoCapture cap;
Mat cameraPose;
System *SLAM = NULL;

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
	if (webcamMode)
	{
		cap = VideoCapture(cameraIndex);
	}
	else
	{
		cap = VideoCapture(filePath);
	}

	// Check it's good
	if (!cap.isOpened())
	{
		cerr << "[Error] -- Failed to open camera source -- exiting." << endl;
		return EXIT_FAILURE;
	}

	if (width <= 0 || height <= 0) // if these values were not set in program args, we need to get them
	{
		width = cap.get(CAP_PROP_FRAME_WIDTH);
		height = cap.get(CAP_PROP_FRAME_HEIGHT);
	}
	else if (webcamMode) // if they were set, and we are using a webcam, we can try set the webcam resolution
	{
		bool res = true;
		res &= cap.set(CAP_PROP_FRAME_WIDTH, width);
		res &= cap.set(CAP_PROP_FRAME_HEIGHT, height);
		if (!res)
		{
			cerr << "[Error] -- Failed to set webcam resolution to " << width << "x" << height << " -- exiting." << endl;
			return EXIT_FAILURE;
		}
	}

	// Scale the calibration file to potentially different input resolution
	scaleCalib();
	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (loadMap)
	{
		SLAM = new System(vocabPath, settingsPath, System::MONOCULAR, true, true);
	}
	else
	{
		SLAM = new System(vocabPath, settingsPath, System::MONOCULAR, true, false);
	}

	// Set up the opengl AR frame
	MyARViewer viewer(SLAM, settingsPath, width, height);

	// Main loop
	Mat im;
	while (true)
	{

		// Get the frame
		cap >> im;

		// Check that it is all good
		if (im.empty() || im.channels() != 3)
		{
			break;
		}

		// Scale the image if required
		scaleIm(im);

		// Get the timestamp
		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackMonocular(im, curNow / 1000.0);
		viewer.renderARFrame(cameraPose, im);
	}

	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	cap.release();

	delete SLAM;
	return EXIT_SUCCESS;
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
		ValueArg<string> filePathArg("f", "filePath", "Path to input video file", false, "../test.avi", "string");
		ValueArg<int> cameraIndexArg("c", "cameraIndex", "Index of the camera to use", false, -1, "integer");
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);
		ValueArg<int> widthArg("W", "resizeWidth", "Width to resize the video to", false, -1, "integer");
		ValueArg<int> heightArg("H", "resizeHeight", "Height to resize the video to", false, -1, "integer");

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(filePathArg);
		cmd.add(cameraIndexArg);
		cmd.add(widthArg);
		cmd.add(heightArg);
		cmd.add(loadMapArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		filePath = filePathArg.getValue();
		cameraIndex = cameraIndexArg.getValue();
		width = widthArg.getValue();
		height = heightArg.getValue();
		loadMap = loadMapArg.getValue();

		// make sure that either camera index or file path is set
		if (!cameraIndexArg.isSet() && !filePathArg.isSet())
		{
			return 1;
		}

		// set the mode as webcam or file -- webcam mode overrides file mode
		webcamMode = cameraIndexArg.isSet() ? true : false;

	} // catch any exceptions 
	catch (ArgException &e) {
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
	double calibWidth = fsSettings["Image.width"];
	double calibHeight = fsSettings["Image.height"];

	// only continue if the calibration file actually had the calibration resolution inside
	if (calibWidth <= 0 || calibHeight <= 0)
	{
		cout << "Image.width and Image.height not found in calibration file, scaling is impossible." << endl;
		return;
	}

	// calculate scaling
	double sw = width / calibWidth;
	double sh = height / calibHeight;

	// vertical and horizontal scaling must be equal and not 1
	if (width == calibWidth || sw != sh)
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
	std::ifstream src(settingsPath, std::ios::binary);
	std::ofstream dst(tempSettingsPath, std::ios::binary);
	dst << src.rdbuf();
	dst.close();

	// edit the new file
	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.fx", std::to_string(fx * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.fy", std::to_string(fy * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.cx", std::to_string(cx * sw));
	settingsFileUpdate(std::string(tempSettingsPath), "Camera.cy", std::to_string(cy * sw));

	// overwrite the settings path
	settingsPath = tempSettingsPath;
}
void scaleIm(Mat &im)
{
	if (width != im.cols)
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

