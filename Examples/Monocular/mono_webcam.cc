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


#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


int parseProgramArguments(int argc, const char *argv[]);
void initPangolinARWindow(pangolin::View& viewReal);
void renderPangolinARFrame(const string& strSettingPath, pangolin::View& viewReal, cv::Mat& pose, cv::Mat& camFrame);


string vocabPath;
string settingsPath;

int cameraIndex;
int cameraFps;
int cameraWidth;
int cameraHeight;


using namespace std;


void renderPangolinARFrame(const string& strSettingPath, pangolin::View& viewReal, cv::Mat& pose, cv::Mat& camFrame)
{
	if (!pangolin::ShouldQuit())
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		cv::cvtColor(camFrame.clone(), camFrame, CV_BGR2RGB);

		pangolin::GlTexture imageTexture(camFrame.cols, camFrame.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.Upload(camFrame.ptr(), GL_RGB, GL_UNSIGNED_BYTE);

		GLfloat znear = 0.01, zfar = 20;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, cameraWidth, cameraHeight);
		glMatrixMode(GL_PROJECTION);

		viewReal.Activate();
		glDisable(GL_DEPTH_TEST);
		glColor3f(1.0, 1.0, 1.0);
		imageTexture.RenderToViewport(true);

		if (!pose.empty())
		{
			float rxd = radiansToDegrees(atan2f(pose.at<float>(2, 1), pose.at<float>(2, 2)));
			float ryd = radiansToDegrees(atan2f(-pose.at<float>(2, 0), sqrtf(pow(pose.at<float>(2, 1), 2) + pow(pose.at<float>(2, 2), 2))));
			float rzd = radiansToDegrees(atan2f(pose.at<float>(1, 0), pose.at<float>(0, 0)));
			float tx = pose.at<float>(0, 3);
			float ty = pose.at<float>(1, 3);
			float tz = pose.at<float>(2, 3);

			// cout << rxd << " " << ryd << " " << rzd << endl;
			// cout << tx << " " << ty << " " << tz << endl << endl;

			GLfloat m[4][4];
			GLfloat fx = cameraWidth, fy = cameraWidth, cx = cameraWidth / 2, cy = cameraHeight / 2;
			m[0][0] = 2.0 * fx / cameraWidth;
			m[0][1] = 0.0;
			m[0][2] = 0.0;
			m[0][3] = 0.0;
			m[1][0] = 0.0;
			m[1][1] = -2.0 * fy / cameraHeight;
			m[1][2] = 0.0;
			m[1][3] = 0.0;
			m[2][0] = 1.0 - 2.0 * cx / cameraWidth;
			m[2][1] = 2.0 * cy / cameraHeight - 1.0;
			m[2][2] = (zfar + znear) / (znear - zfar);
			m[2][3] = -1.0;
			m[3][0] = 0.0;
			m[3][1] = 0.0;
			m[3][2] = 2.0 * zfar * znear / (znear - zfar);
			m[3][3] = 0.0;

			glLoadIdentity();
			glMultMatrixf((GLfloat *)m);

			glTranslated(tx, ty, -tz);
			glRotated(rzd, 0.0, 0.0, 1.0);
			glRotated(-ryd, 0.0, 1.0, 0.0);
			glRotated(-rxd, 1.0, 0.0, 0.0);

			glEnable(GL_DEPTH_TEST);
			pangolin::glDrawColouredCube(-0.05f, 0.05f);
			
		}
		else {
			pangolin::glDrawAxis(1000000.f);
		}

		// Swap frames and Process Events
		pangolin::FinishFrame();
	}
}

void initPangolinARWindow(pangolin::View& viewReal)
{
	pangolin::CreateWindowAndBind("Main", cameraWidth, cameraHeight);
	glEnable(GL_DEPTH_TEST);
	viewReal = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-cameraWidth / (double)cameraHeight);	

	return;
}


int main(int argc, const char *argv[])
{
	// parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Warning] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, true);

	// Set up the opengl AR frame
	pangolin::View viewReal;
	initPangolinARWindow(viewReal);
	
	// Set up webcam
	cv::VideoCapture cap(cameraIndex);
	if (cameraFps > 0) cap.set(CV_CAP_PROP_FPS, cameraFps);
	if (cameraWidth > 0) cap.set(CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
	if (cameraHeight > 0) cap.set(CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);

    cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	cv::Mat im;
	cv::Mat cameraPose;

	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    while (true)
    {
		cap >> im;
		if (im.empty() || im.channels() != 3) continue;

		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		cout << curNow << endl;
		// Pass the image to the SLAM system
		cameraPose = SLAM.TrackMonocular(im, curNow / 1000.0);
		renderPangolinARFrame(settingsPath, viewReal, cameraPose, im);

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

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(cameraIndexArg);
		cmd.add(cameraFpsArg);
		cmd.add(cameraWidthArg);
		cmd.add(cameraHeightArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		cameraIndex = cameraIndexArg.getValue();
		cameraFps = cameraFpsArg.getValue();
		cameraWidth = cameraWidthArg.getValue();
		cameraHeight = cameraHeightArg.getValue();

	} // catch any exceptions 
	catch (ArgException &e) {
		cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
		return 1;
	}
	return 0;
}

