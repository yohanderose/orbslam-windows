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
#include <tclap/CmdLine.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/handler/handler.h>


using namespace std;
using namespace cv;
using namespace ORB_SLAM2;
using namespace pangolin;


#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


/* Forward function declarations */
int parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);
void initPangolinARWindow(View& viewReal);
void renderPangolinARFrame(const string& strSettingPath, View& viewReal, Mat& pose, Mat& camFrame);

/* AR display globals */
vector<Point3f> worldPoints;
vector<Point3f> worldPointNormals;
bool drawNormals = false;
bool paused = false;
Point3f cubeLocation(0, 0, 0);

/* Program arg globals */
string vocabPath;
string settingsPrefix;
char settingsFilename[512];
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

/* Panorama stuff */
int faceIndexOrder[] = { 4,5,6,7,0,1,2,3 };


/* Main */
int main(int argc, const char *argv[])
{
	// Parse the command line args
	int res = parseProgramArguments(argc, argv);
	if (res == 1) {
		cerr << "[Error] -- Failed to parse command line arguments -- exiting." << endl;
		return EXIT_FAILURE;
	}

	int nFaces = 8;
	int faceIndex = 0;
	char filename[512];
	sprintf_s(filename, "C:\\Users\\Lewis\\Desktop\\PhD\\PhdFiles\\thetas\\video\\er\\R0010029_er.MP4.d\\face%d.image%%05d.jpg", faceIndexOrder[faceIndex]);

	// Set up video source
	if (webcamMode)
	{
		cout << cameraIndex << endl;
		cap = VideoCapture(cameraIndex);
	}
	else
	{
		cout << filename << endl;
		cap = VideoCapture(filename);
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

	settingsFilename[512];
	sprintf_s(settingsFilename, "%s%d.yaml", settingsPrefix.c_str(), faceIndexOrder[faceIndex]);

	// Scale the calibration file to potentially different input resolution
	scaleCalib();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (loadMap)
	{
		SLAM = new System(vocabPath, settingsFilename, System::MONOCULAR, true, true);
	}
	else
	{
		SLAM = new System(vocabPath, settingsFilename, System::MONOCULAR, true, false);
	}

	// Set up the opengl AR frame
	View viewReal;
	initPangolinARWindow(viewReal);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	// Main loop
	Mat im;

	// Out file stream for the path of each face of the panoramic camera
	ofstream facePathOfs;
	string facePathFname = "path-f" + to_string(faceIndex) + ".txt";
	facePathOfs.open(facePathFname, ofstream::out);
	while (true)
	{
		// If we are paused, keep the AR window working
		if (paused)
		{
			renderPangolinARFrame(settingsFilename, viewReal, cameraPose, im);
			continue;
		}

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
			sprintf_s(filename, "C:\\Users\\Lewis\\Desktop\\PhD\\PhdFiles\\thetas\\video\\er\\R0010029_er.MP4.d\\face%d.image%%05d.jpg", faceIndexOrder[faceIndex]);
			cap = VideoCapture(filename);

			// re-open the path file with a different filename
			facePathOfs.close();
			facePathFname = "path-f" + to_string(faceIndex) + ".txt";
			facePathOfs.open(facePathFname, ofstream::out);

			// re-load the calibration
			settingsFilename[512];
			sprintf_s(settingsFilename, "%s%d.yaml", settingsPrefix.c_str(), faceIndexOrder[faceIndex]);
			scaleCalib();
			SLAM->ChangeCalibration(settingsFilename);

			continue;
		}

		// Scale the image if required
		scaleIm(im);

		// Get the timestamp
		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		cameraPose = SLAM->TrackMonocular(im, curNow / 1000.0);

		// Save the path of this face to the file
		if (!cameraPose.empty())
		{
			cv::Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*cameraPose.rowRange(0, 3).col(3);
			facePathOfs << twc.at<float>(0) << ";" << twc.at<float>(1) << ";" << twc.at<float>(2) << endl;
		}
		
		// Render the AR window
		renderPangolinARFrame(settingsFilename, viewReal, cameraPose, im);
	}

	// Confirm to quit
	//imshow("Press any key to quit", im);
	//cv::waitKey();

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
		settingsPrefix = settingsPathArg.getValue();
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
	FileStorage fsSettings(settingsFilename, FileStorage::READ);

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
	const char tempSettingsPath[] = "s-calib.yaml";
	remove(tempSettingsPath);

	// copy the calibration file
	std::ifstream src(settingsFilename, std::ios::binary);
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

	sprintf(settingsFilename, "%s", tempSettingsPath);
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

/* AR functions */
struct MyEventHandler : public Handler
{
	void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state)
	{
		y = height - y - 10;
		if (!pressed)
		{
			std::vector<MapPoint *> vMP = SLAM->GetTrackedMapPoints();
			int N = vMP.size();
			vector<bool> mvbMap(N, false);

			for (int i = 0; i<N; i++)
			{
				MapPoint* pMP = vMP[i];
				if (pMP)
				{
					//if (!pTracker->mCurrentFrame.mvbOutlier[i]) // not good
					{
						if (pMP->Observations()>0)
							mvbMap[i] = true;
					}
				}
			}

			vector<KeyPoint> kps = SLAM->GetTrackedKeyPointsUn();
			bool found = false;
			if (kps.size() > 0 && kps.size() == vMP.size())
			{
				double minDist = DBL_MAX;
				int minIdx = INT_MAX;
				for (int i = 0; i < kps.size(); ++i)
				{
					if (mvbMap[i])
					{
						double dist = abs(norm(kps[i].pt - Point2f(x, y)));
						if (dist < minDist)
						{
							minDist = dist;
							minIdx = i;
							found = true;
						}
					}
				}

				if (found && vMP[minIdx] != NULL)
				{
					Mat wp = vMP[minIdx]->GetWorldPos();
					cubeLocation = Point3f(
						wp.at<float>(0, 0),
						wp.at<float>(1, 0),
						wp.at<float>(2, 0)
					);

					cout << Point2f(x, y) << " " << kps[minIdx].pt << " " << cubeLocation << endl;
				}
				else
				{
					cout << "it's null!\n";
				}
			}
		}
	}
	void Keyboard(View&, unsigned char key, int x, int y, bool pressed)
	{
		if (!pressed)
		{
			switch (key)
			{
			case 'a':
			{
				worldPoints.clear();
				worldPointNormals.clear();

				if (cameraPose.empty())
					return;

				vector<MapPoint *> mapPoints = SLAM->GetAllMapPoints();

				for (vector<MapPoint *>::iterator mit = mapPoints.begin(); mit != mapPoints.end(); mit++)
				{
					if (*mit == NULL)
						continue;

					Mat wp = (*mit)->GetWorldPos();
					worldPoints.push_back(Point3f(
						wp.at<float>(0, 0),
						wp.at<float>(1, 0),
						wp.at<float>(2, 0)));

					Mat n = (*mit)->GetNormal() * 0.05;
					worldPointNormals.push_back(Point3f(
						n.at<float>(0, 0),
						n.at<float>(1, 0),
						n.at<float>(2, 0)));
				}
				break;
			}
			case 's':
			{
				worldPoints.clear();
				worldPointNormals.clear();
				break;
			}
			case 'n':
			{
				drawNormals = !drawNormals;
				break;
			}
			case 'p':
			{
				paused = !paused;
				break;
			}
			}
		}
	}
};
void renderPangolinARFrame(const string& strSettingPath, View& viewReal, Mat& pose, Mat& camFrame)
{
	if (!ShouldQuit())
	{
		Mat camFrameRgb, camFrameRgbUn;
		FileStorage fSettings(strSettingPath, FileStorage::READ);
		cvtColor(camFrame, camFrameRgbUn, CV_BGR2RGB);

		Mat K = Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = fSettings["Camera.fx"];
		K.at<float>(1, 1) = fSettings["Camera.fy"];
		K.at<float>(0, 2) = fSettings["Camera.cx"];
		K.at<float>(1, 2) = fSettings["Camera.cy"];

		Mat distCoeffs = Mat::zeros(5, 1, CV_32F);
		distCoeffs.at<float>(0) = fSettings["Camera.k1"];
		distCoeffs.at<float>(1) = fSettings["Camera.k2"];
		distCoeffs.at<float>(2) = fSettings["Camera.p1"];
		distCoeffs.at<float>(3) = fSettings["Camera.p2"];
		distCoeffs.at<float>(4) = fSettings["Camera.k3"];
		undistort(camFrameRgbUn, camFrameRgb, K, distCoeffs);

		GlTexture imageTexture(camFrameRgb.cols, camFrameRgb.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.Upload(camFrameRgb.ptr(), GL_RGB, GL_UNSIGNED_BYTE);

		GLfloat znear = 0.01, zfar = 20;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, width, height);
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

			GLfloat m[4][4];
			GLfloat fx = fSettings["Camera.fx"];
			GLfloat fy = fSettings["Camera.fy"];
			GLfloat cx = fSettings["Camera.cx"];
			GLfloat cy = fSettings["Camera.cy"];

			m[0][0] = 2.0 * fx / width;
			m[0][1] = 0.0;
			m[0][2] = 0.0;
			m[0][3] = 0.0;

			m[1][0] = 0.0;
			m[1][1] = -2.0 * fy / height;
			m[1][2] = 0.0;
			m[1][3] = 0.0;

			m[2][0] = 1.0 - 2.0 * cx / width;
			m[2][1] = 2.0 * cy / height - 1.0;
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

			glPushMatrix();
			// set cube location
			glTranslated(cubeLocation.x, cubeLocation.y, -cubeLocation.z);

			glEnable(GL_DEPTH_TEST);
			glDrawColouredCube(-0.005f, 0.005f);
			glPopMatrix();

			glPointSize(5.0f);
			for (int i = 0; i < worldPoints.size(); ++i)
			{
				glBegin(GL_POINTS);
				glColor3f(1.0f, 1.0f, 1.0f);
				glVertex3f(worldPoints[i].x, worldPoints[i].y, -worldPoints[i].z);
				glEnd();

				if (drawNormals)
				{
					glBegin(GL_LINES);
					glColor3f(0.0f, 0.0f, 1.0f);
					glVertex3f(worldPoints[i].x, worldPoints[i].y, -worldPoints[i].z);
					glVertex3f(worldPoints[i].x + worldPointNormals[i].x, worldPoints[i].y + worldPointNormals[i].y, -worldPoints[i].z + worldPointNormals[i].z);
					glEnd();
				}
			}

		}
		else {
			glDrawAxis(1000000.f);
		}

		// Swap frames and Process Events
		FinishFrame();
	}
}
void initPangolinARWindow(View& viewReal)
{
	CreateWindowAndBind("Main", width, height);
	glEnable(GL_DEPTH_TEST);
	MyEventHandler *handler = new MyEventHandler();
	viewReal = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-width / (double)height).SetHandler(handler);


	return;
}

