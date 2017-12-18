/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published byf
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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Utils/MyARViewer.h"
#include "../Utils/CommandLine.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


/* Forward function declarations */
bool parseProgramArguments(int argc, const char *argv[]);
bool scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(string &filePath, string name, string val);

/* Program arg globals */
string vocabPath;
string settingsPrefix;
string settingsPath;
string filePath;
int width;
int height;
bool loadMap;

/* Misc globals */
VideoCapture cap;
Mat cameraPose;
System *SLAM = NULL;

/* Panorama stuff */
int faceIndexOrder[] = { 2,3,4,5,6,7,0,1 };
bool changeCalib = false;

/* Main */
int main(int argc, const char *argv[])
{
	// Parse the command line args
	if (!parseProgramArguments(argc, argv)) {
		cerr << "[Error] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	int nFaces = 8;
	int faceIndex = 0;
	string filename = "C:\\Users\\Lewis\\Desktop\\PhD\\PhdFiles\\thetas\\video\\er\\R0010055_er.MP4.d\\face" + to_string(faceIndexOrder[faceIndex]) + ".image%05d.jpg";

	cout << filename << endl;
	cap = VideoCapture(filename);

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

	if (changeCalib)
	{
		settingsPath = settingsPrefix + to_string(faceIndexOrder[faceIndex]) + ".yaml";
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

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	Mat im;

	// Out file stream for the path of each face of the panoramic camera
	ofstream facePathOfs;
	string facePathFname = "path-f" + to_string(faceIndexOrder[faceIndex]) + ".txt";
	facePathOfs.open(facePathFname, ofstream::out);
	while (true)
	{
		// Get the frame
		cap >> im;

		// Check that it is all good
		if (im.empty() || im.channels() != 3)
		{
			// increment the face number if we run out of frames, quit if we run out of facess
			faceIndex++;
			if (faceIndex == nFaces)
			{
				break;
			}

			// re-open the capture with a difference face
			cap.release();
			filename = "C:\\Users\\Lewis\\Desktop\\PhD\\PhdFiles\\thetas\\video\\er\\R0010055_er.MP4.d\\face" + to_string(faceIndexOrder[faceIndex]) + ".image%05d.jpg";
			cap = VideoCapture(filename);

			// re-open the path file with a different filename
			facePathOfs.close();
			facePathFname = "path-f" + to_string(faceIndexOrder[faceIndex]) + ".txt";
			facePathOfs.open(facePathFname, ofstream::out);

			// re-load the calibration
			if (changeCalib)
			{
				settingsPath = settingsPrefix + to_string(faceIndexOrder[faceIndex]) + ".yaml";
				scaleCalib();
				SLAM->ChangeCalibration(settingsPath);
			}

			continue;
		}

		// Scale the image if required
		scaleIm(im);

		// Get the timestamp
		__int64 curNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackMonocular(im, curNow / 1000.0);
		if (viewer.renderARFrame(cameraPose, im))
		{
			break;
		}

		// Save the path of this face to the file
		if (!cameraPose.empty())
		{
			Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
			Mat twc = -Rwc*cameraPose.rowRange(0, 3).col(3);
			facePathOfs << twc.at<float>(0) << ";" << twc.at<float>(1) << ";" << twc.at<float>(2) << endl;
		}
	}
	Mat m = Mat::zeros(100, 100, CV_8UC1);
	imshow("Press any key to quit", m);
	waitKey();

	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	cap.release();

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

	if (cmd.ContainsKey("f"))
		if (!cmd.GetStringValue("f", filePath)) result = false;

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

	if (cmd.ContainsKey("C"))
	{
		changeCalib = true;
		settingsPrefix = settingsPath;
	}

	if (cmd.ContainsKey("h"))
	{

	}

	return result;
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
	const char tempSettingsPath[] = "s-calib.yaml";
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
	return true;
}
void scaleIm(Mat &im)
{
	if (width != im.cols || height != im.rows)
		resize(im, im, Size(width, height), 0.0, 0.0, CV_INTER_AREA);
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

