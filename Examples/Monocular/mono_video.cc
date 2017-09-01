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


using namespace std;


int parseProgramArguments(int argc, const char *argv[]);


string vocabPath;
string settingsPath;
string filePath;


int main(int argc, const char *argv[])
{
	// parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Warning] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	// Set up webcam
	cv::VideoCapture cap(filePath);

	// Test the webcam
	cv::Mat test;
	double frameCount = cap.get(CV_CAP_PROP_FRAME_COUNT);
	for (int i = 0; i < frameCount; ++i)
	{
		cap >> test;
		imshow("Testing video - press any key to continue", test);
		if (cv::waitKey(1) != 255) break;
	}
	cv::destroyAllWindows();
	cap.set(CV_CAP_PROP_POS_FRAMES, 0);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	cv::Mat im;
	cv::Mat Tcw;

	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	for (int i = 0; i < frameCount; ++i)
	{
		cap >> im;

		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		Tcw = SLAM.TrackMonocular(im, curNow / 1000.0);

		// This will make a third window with the color images, you need to click on this then press any key to quit
		cv::imshow("Image", im);
		if (cv::waitKey(1) != 255)
		{
			break;
		}
	}

	// Stop all threads
	SLAM.Shutdown();
	cap.release();

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;
}

// Parses the program arguments and sets the global variables using tclap
int parseProgramArguments(int argc, const char *argv[])
{
	using namespace TCLAP;
	try {
		// set up the args
		CmdLine cmd("Runs ORB_SLAM2 with a monocular video file", ' ', "0.1");
		ValueArg<string> vocabPathArg("v", "vocabPath", "Path to ORB vocabulary", false, "../ORBvoc.txt", "string");
		ValueArg<string> settingsPathArg("s", "settingsPath", "Path to webcam calibration and ORB settings yaml file", false, "../webcam.yaml", "string");
		ValueArg<string> filePathArg("f", "filePath", "Path to input video file", false, "../test.mp4", "string");

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(filePathArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		filePath = filePathArg.getValue();

	} // catch any exceptions 
	catch (ArgException &e) {
		cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
		return 1;
	}
	return 0;
}
