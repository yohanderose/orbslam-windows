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
void scaleCalib();
void scaleIm(cv::Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);
void initPangolinARWindow(pangolin::View& viewReal);
void renderPangolinARFrame(const string& strSettingPath, pangolin::View& viewReal, cv::Mat& pose, cv::Mat& camFrame);


string vocabPath;
string settingsPath;

int cameraIndex;
int cameraFps;
int cameraWidth;
int cameraHeight;

bool loadMap;

cv::VideoCapture cap;


using namespace std;


void renderPangolinARFrame(const string& strSettingPath, pangolin::View& viewReal, cv::Mat& pose, cv::Mat& camFrame)
{
	if (!pangolin::ShouldQuit())
	{
		cv::Mat camFrameRgb;
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		cv::cvtColor(camFrame, camFrameRgb, CV_BGR2RGB);

		pangolin::GlTexture imageTexture(camFrameRgb.cols, camFrameRgb.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.Upload(camFrameRgb.ptr(), GL_RGB, GL_UNSIGNED_BYTE);

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

	// Set up the opengl AR frame
	pangolin::View viewReal;
	initPangolinARWindow(viewReal);
	
	// Set up webcam
	cap = cv::VideoCapture(cameraIndex);
	if (cameraFps > 0) cap.set(CV_CAP_PROP_FPS, cameraFps);
	if (cameraWidth > 0) cap.set(CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
	if (cameraHeight > 0) cap.set(CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);

	scaleCalib();

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
		scaleIm(im);

		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackMonocular(im, curNow / 1000.0);
		renderPangolinARFrame(settingsPath, viewReal, cameraPose, im);

		// This will make a third window with the color images, you need to click on this then press any key to quit
		cv::imshow("Image", im); 
		if (cv::waitKey(1) != 255)
		{
			break;
		}
    }

    // Stop all threads
    SLAM->Shutdown();
	if (!loadMap)
		SLAM->SaveMap("Slam_Map.bin");

	cap.release();

	delete SLAM;
	return EXIT_SUCCESS;
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
		SwitchArg loadMapArg("l", "loadMap", "Load map file", false);

		// add the args
		cmd.add(vocabPathArg);
		cmd.add(settingsPathArg);
		cmd.add(cameraIndexArg);
		cmd.add(cameraFpsArg);
		cmd.add(cameraWidthArg);
		cmd.add(cameraHeightArg);
		cmd.add(loadMapArg);

		// parse the args
		cmd.parse(argc, argv);

		// get the results
		vocabPath = vocabPathArg.getValue();
		settingsPath = settingsPathArg.getValue();
		cameraIndex = cameraIndexArg.getValue();
		cameraFps = cameraFpsArg.getValue();
		cameraWidth = cameraWidthArg.getValue();
		cameraHeight = cameraHeightArg.getValue();
		loadMap = loadMapArg.getValue();

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
	double videoWidth = cameraWidth > 0 ? cameraWidth : cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double videoHeight = cameraHeight > 0 ? cameraHeight : cap.get(CV_CAP_PROP_FRAME_HEIGHT);

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
	if (cameraWidth != im.cols && cameraWidth != -1)
		cv::resize(im, im, cv::Size(cameraWidth, cameraHeight));
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