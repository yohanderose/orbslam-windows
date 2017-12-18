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
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

#include "../Utils/KinectRgbd.h"
#include "../Utils/CommandLine.h"
#include "../Utils/MyARViewer.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


bool parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(string &filePath, string name, string val);


string vocabPath;
string settingsPath;

int cameraIndex;
int cameraWidth;
int cameraHeight;

bool loadMap;

Mat cameraPose;



#include <mutex>
mutex cMtx, dMtx;
Mat cfMat;
Mat dfMat;
bool cReady = false;
bool dReady = false;
double ts;

void GetColorFrame(Mat &cf)
{
	unique_lock<mutex> lock(cMtx);
	cf.copyTo(cfMat);
	cReady = true;
}
void GetDepthFrame(Mat &df)
{
	unique_lock<mutex> lock(dMtx);
	df.copyTo(dfMat);
	dReady = true;
}

int main(int argc, const char *argv[])
{
	// parse the command line args
	if (!parseProgramArguments(argc, argv)) {
		cerr << "[Warning] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	System *SLAM = NULL;

	scaleCalib();

	
	KinectRgbd k;
	if (!k.IsValid())
	{
		cerr << "[Warning] -- Could not detect Kinect sensor." << endl;
		return EXIT_FAILURE;
	}
	k.StartColorStream(GetColorFrame);
	k.StartDepthStream(GetDepthFrame);
	

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (loadMap)
	{
		SLAM = new System(vocabPath, settingsPath, System::RGBD, true, true);
	}
	else
	{
		SLAM = new System(vocabPath, settingsPath, System::RGBD, true, false);
	}

	// Set up the opengl AR frame
	MyARViewer viewer(SLAM, settingsPath, cameraWidth, cameraHeight);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

	while (true)
	{
		unique_lock<mutex> lock(dMtx);
		unique_lock<mutex> lock2(cMtx);

		if (cReady && dReady)
		{
			scaleIm(cfMat);
			scaleIm(dfMat);

			__int64 curNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

			// Pass the image to the SLAM system
			cameraPose = SLAM->TrackRGBD(cfMat, dfMat, curNow / 1000.0);		
			viewer.renderARFrame(cameraPose, cfMat);

			cReady = false;
			dReady = false;
		}
	}

	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");


	delete SLAM;
	return EXIT_SUCCESS;
}

bool parseProgramArguments(int argc, const char *argv[])
{
	bool result = true;
	CommandLine cmd(argc, argv);

	if (cmd.ContainsKey("v"))
		if (!cmd.GetStringValue("v", vocabPath)) result = false;

	if (cmd.ContainsKey("s"))
		if (!cmd.GetStringValue("s", settingsPath)) result = false;

	if (cmd.ContainsKey("c"))
	{
		if (!cmd.GetIntValue("c", cameraIndex)) result = false;
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
			cameraWidth = resolution[0];
			cameraHeight = resolution[1];
		}
	}

	if (cmd.ContainsKey("h"))
	{

	}

	return result;
}


/* Scales the calibration data to the input video resolution */
void scaleCalib()
{
	// open the original calibration
	FileStorage fsSettings(settingsPath, FileStorage::READ);

	// get the calibration and input resolutions
	double calibWidth = double(fsSettings["Camera.width"]);
	double calibHeight = double(fsSettings["Camera.height"]);
	double videoWidth = cameraWidth > 0 ? cameraWidth : calibWidth;
	double videoHeight = cameraHeight > 0 ? cameraHeight : calibHeight;

	// only continue if the calibration file actually had the calibration resolution inside
	if (calibWidth <= 0 || calibHeight <= 0)
	{
		cout << "Camera.width and Camera.height not found in calibration file, scaling is impossible." << endl;
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
	std::remove(tempSettingsPath);

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
	std::remove((filePath + ".tmp").c_str());

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
	std::remove(filePath.c_str());
	rename((filePath + ".tmp").c_str(), filePath.c_str());
}