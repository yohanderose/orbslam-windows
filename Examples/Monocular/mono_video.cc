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


using namespace std;


int parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(cv::Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);


string vocabPath;
string settingsPath;
string filePath;
bool loadMap;
int rWidth;
int rHeight;


cv::VideoCapture cap;


int main(int argc, const char *argv[])
{
	// parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Warning] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	// Set up webcam
	cap = cv::VideoCapture(filePath);

	// Test the webcam
	double frameCount = cap.get(CV_CAP_PROP_FRAME_COUNT);

	scaleCalib();
	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System *SLAM = NULL;
	if (loadMap)
	{
		SLAM = new ORB_SLAM2::System(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, true);
	}
	else
	{
		SLAM = new ORB_SLAM2::System(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, false);
	}

	if (SLAM == NULL)
	{
		cout << "SLAM is NULL" << endl;
		return EXIT_FAILURE;
	}

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	cv::Mat im;
	cv::Mat Tcw;
	for (int i = 0; i < frameCount; ++i)
	{
		// Get the frame
		cap >> im;

		if (im.empty() || im.channels() != 3) continue;
		scaleIm(im);

		// Get the timestamp
		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		Tcw = SLAM->TrackMonocular(im, curNow / 1000.0);

		// This will make a third window with the color images, you need to click on this then press any key to quit
		//cv::imshow("Image", im);
		//if (cv::waitKey(1) != 255)
		//{
			//break;
		//}
	}


	// Stop all threads
	SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	cap.release();

	delete SLAM;
	return EXIT_SUCCESS;
}

// Parses the program arguments and sets the global variables using tclap
int parseProgramArguments(int argc, const char *argv[])
{
	using namespace TCLAP;
	try {
		// set up the args
		CmdLine cmd("Runs ORB_SLAM2 with a monocular video file", ' ', "0.1");
		ValueArg<string> vocabPathArg("v", "vocabPath", "Path to ORB vocabulary", false, "../ORBvoc.bin", "string");
		ValueArg<string> settingsPathArg("s", "settingsPath", "Path to webcam calibration and ORB settings yaml file", false, "../webcam.yaml", "string");
		ValueArg<string> filePathArg("f", "filePath", "Path to input video file", false, "../test.mp4", "string");
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);
		ValueArg<int> widthArg("W", "resizeWidth", "Width to resize the video to", false, -1, "integer");
		ValueArg<int> heightArg("H", "resizeHeight", "Height to resize the video to", false, -1, "integer");

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(filePathArg);
		cmd.add(loadMapArg);
		cmd.add(widthArg);
		cmd.add(heightArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		filePath = filePathArg.getValue();
		loadMap = loadMapArg.getValue();
		rWidth = widthArg.getValue();
		rHeight = heightArg.getValue();

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
	cv::FileStorage fsSettings(settingsPath, cv::FileStorage::READ);

	// get the calibration and input resolutions
	double calibWidth = double(fsSettings["Image.width"]);
	double calibHeight = double(fsSettings["Image.height"]);
	double videoWidth = rWidth > 0 ? rWidth : cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double videoHeight = rHeight > 0 ? rHeight : cap.get(CV_CAP_PROP_FRAME_HEIGHT);

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
void scaleIm(cv::Mat &im)
{
	if (rWidth != im.cols && rWidth != -1)
		cv::resize(im, im, cv::Size(rWidth, rHeight), 0.0, 0.0, CV_INTER_AREA);
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

