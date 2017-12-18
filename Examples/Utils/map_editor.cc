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


#include "../Utils/CommandLine.h"


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;


/* Forward function declarations */
bool parseProgramArguments(int argc, const char *argv[]);

/* Program arg globals */
string vocabPath;
string settingsPath;
string filePath;

/* Main */
int main(int argc, const char *argv[])
{
	// Parse the command line args
	if (!parseProgramArguments(argc, argv)) {
		cerr << "[Error] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	System *SLAM = NULL;
	
	SLAM = new System(vocabPath, settingsPath, System::MONOCULAR, true, true);

	// Main loop
	Mat black = Mat::zeros(500, 500, CV_8UC1);
	while (true)
	{
		imshow("Press Esc key to quit", black);
		if (27 == waitKey(1)) break;
	}

	// Stop all threads
	SLAM->Shutdown();
	SLAM->SaveMap("Slam_Map_Edited.bin");

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

	if (cmd.ContainsKey("m"))
		if (!cmd.GetStringValue("m", filePath)) result = false;

	return result;
}

