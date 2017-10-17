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


class Plane;
struct ARWindowState;
struct ARCube;


/* Forward function declarations */
int parseProgramArguments(int argc, const char *argv[]);
void scaleCalib();
void scaleIm(Mat &im);
void settingsFileUpdate(std::string &filePath, std::string name, std::string val);
bool getPlaneRansac(System *SLAM, Plane &p, vector<MapPoint *> vMP, vector<bool> vbMap);
void renderPangolinARFrame(ARWindowState& state, Mat& pose, Mat& camFrame);
void initPangolinARWindow(ARWindowState& result, const string& settingsPath, int width, int height);

/* AR display globals */
vector<Point3f> worldPoints;
bool paused = false;
int cubesIdx = -1;
vector<ARCube> cubes;

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
const bool changeCalib = false;

/* AR stuff */
class Plane
{
public:
	Plane()
	{ }
	Plane(Eigen::Vector3d n, Eigen::Vector3d p)
		: mvN(n), mvP(p)
	{ }
	Plane(Vec3d n, Vec3d p)
	{
		mvN = Eigen::Vector3d(n[0], n[1], n[2]);
		mvP = Eigen::Vector3d(p[0], p[1], p[2]);
	}
private:
	Eigen::Vector3d mvN;
	Eigen::Vector3d mvP;
public:
	Vec3d getNormalCV()
	{

		return Vec3d(mvN[0], mvN[1], mvN[2]);
	}
	Vec3d getPointCV()
	{
		return Vec3d(mvP[0], mvP[1], mvP[2]);
	}
	Eigen::Vector3d getNormal()
	{
		return mvN;
	}
	Eigen::Vector3d getPoint()
	{
		return mvP;
	}
};
struct ARCube
{
	Point3d location;
	Vec3d normal;
	double scale;
};
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
					ARCube cube;
					cube.location = Point3f(
						wp.at<float>(0, 0),
						wp.at<float>(1, 0),
						wp.at<float>(2, 0)
					);
					cube.scale = 0.005;

					Plane p;
					getPlaneRansac(SLAM, p, vMP, mvbMap);
					cube.normal = p.getNormalCV();

					cout << Point2f(x, y) << " " << kps[minIdx].pt << " " << cube.location << endl;

					cubes.push_back(cube);
					++cubesIdx;
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
				}
				break;
			}
			case 's':
			{
				worldPoints.clear();
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
	void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
	{
		if (inType == InputSpecialScroll)
		{
			const int scrollSpeed = 24;
			double scaleFactor = p2 / (scrollSpeed * 10.0);
			cubes[cubesIdx].scale = cubes[cubesIdx].scale * (1 + scaleFactor);
		}
	}
};
struct ARWindowState
{
	View viewReal;
	View viewCam;
	MyEventHandler handler;
	OpenGlRenderState camState;
	OpenGlMatrix cameraMatrix;
	Mat K;
	Mat distCoeffs;
};

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
	ARWindowState winState;
	initPangolinARWindow(winState, settingsFilename, width, height);

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
		// If we are paused, keep the AR window working
		if (paused)
		{
			renderPangolinARFrame(winState, cameraPose, im);
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
			facePathFname = "path-f" + to_string(faceIndexOrder[faceIndex]) + ".txt";
			facePathOfs.open(facePathFname, ofstream::out);

			// re-load the calibration
			if (changeCalib)
			{
				settingsFilename[512];
				sprintf_s(settingsFilename, "%s%d.yaml", settingsPrefix.c_str(), faceIndexOrder[faceIndex]);
				scaleCalib();
				SLAM->ChangeCalibration(settingsFilename);
			}

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
		renderPangolinARFrame(winState, cameraPose, im);
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

void selectRandomMapPoints(vector<MapPoint *>& vMPgood, MapPoint *randomSampleMP[3], Eigen::Vector3d randomSample[3])
{
	int r0, r1, r2;

	r0 = rand() % vMPgood.size();
	r1 = rand() % vMPgood.size();
	r2 = rand() % vMPgood.size();

	Mat m0, m1, m2;
	m0 = vMPgood[r0]->GetWorldPos();
	m1 = vMPgood[r1]->GetWorldPos();
	m2 = vMPgood[r2]->GetWorldPos();

	randomSampleMP[0] = vMPgood[r0];
	randomSampleMP[1] = vMPgood[r1];
	randomSampleMP[2] = vMPgood[r2];

	randomSample[0] = Eigen::Vector3d(m0.at<float>(0, 0), m0.at<float>(1, 0), m0.at<float>(2, 0));
	randomSample[1] = Eigen::Vector3d(m1.at<float>(0, 0), m1.at<float>(1, 0), m1.at<float>(2, 0));
	randomSample[2] = Eigen::Vector3d(m2.at<float>(0, 0), m2.at<float>(1, 0), m2.at<float>(2, 0));
}
bool getPlaneRansac(System *SLAM, Plane &p, vector<MapPoint *> vMP, vector<bool> vbMap)
{
	using namespace Eigen;

	int N = vMP.size();

	// store the "true" map points to make random sampling easier
	vector<MapPoint *> vMPgood;
	for (int i = 0; i < N; ++i)
	{
		if (vbMap[i])
		{
			vMPgood.push_back(vMP[i]);
		}
	}

	// Ransac
	int bestConsensusSetSize = -1;
	Plane bestPlane;

	int maxI = 1000;
	for (int i = 0; i < maxI; ++i)
	{
		MapPoint *randomSampleMP[3];
		Vector3d randomSample[3];
		selectRandomMapPoints(vMPgood, randomSampleMP, randomSample);

		// fit plane using 3 points
		Vector3d u = randomSample[1] - randomSample[0];
		Vector3d v = randomSample[2] - randomSample[0];
		Vector3d n = u.cross(v);

		// check collinearity
		if (n.norm() == 0)
		{
			continue;
		}

		// find consensus set size
		n.normalize();
		Plane tmp = Plane(n, randomSample[0]);
		int consensusSetSize = 0;
		const double distThresh = 0.05;
		for (int j = 0; j < vMPgood.size(); ++j)
		{
			Mat pointMat = vMPgood[j]->GetWorldPos();
			Vector3d point(pointMat.at<float>(0, 0), pointMat.at<float>(1, 0), pointMat.at<float>(2, 0));

			double dist = tmp.getNormal().dot(point - tmp.getPoint());
			if (abs(dist) < distThresh)
			{
				++consensusSetSize;
			}
		}
		if (consensusSetSize > bestConsensusSetSize && consensusSetSize != 0)
		{
			bestPlane = tmp;
			bestConsensusSetSize = consensusSetSize;
		}
	}
	if (bestConsensusSetSize > 50)
	{
		p = bestPlane;
		cout << "DETECTED PLANE " << bestConsensusSetSize << " " << bestPlane.getNormal().transpose() << " " << bestPlane.getPoint().transpose() << endl;
		return true;
	}
	else
	{
		cout << "FAILED TO DETECT PLANE" << endl;
		return false;
	}
}

void renderPangolinARFrame(ARWindowState& state, Mat& pose, Mat& camFrame)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (!ShouldQuit())
	{
		// correct the input frame
		cvtColor(camFrame, camFrame, CV_BGR2RGB);
		Mat camFrameUn;
		undistort(camFrame, camFrameUn, state.K, state.distCoeffs);

		// render the camera feed to the background
		glDisable(GL_DEPTH_TEST);
		GlTexture imageTexture(camFrameUn.cols, camFrameUn.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.Upload(camFrameUn.ptr(), GL_RGB, GL_UNSIGNED_BYTE);
		state.viewReal.Activate();
		glColor3f(1.0, 1.0, 1.0);
		imageTexture.RenderToViewport(true);

		// render the AR objects
		if (!pose.empty())
		{
			glEnable(GL_DEPTH_TEST);
			state.viewCam.Activate(state.camState);

			// Extract the pose information
			Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
			Mat twc = -Rwc*cameraPose.rowRange(0, 3).col(3);
			float rx = atan2f(Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
			float ry = atan2f(-Rwc.at<float>(2, 0), sqrtf(pow(Rwc.at<float>(2, 1), 2) + pow(Rwc.at<float>(2, 2), 2)));
			float rz = atan2f(Rwc.at<float>(1, 0), Rwc.at<float>(0, 0));

			// Transform the camera
			OpenGlMatrix viewMatrix = IdentityMatrix();
			viewMatrix = viewMatrix * OpenGlMatrix::RotateX(-rx);
			viewMatrix = viewMatrix * OpenGlMatrix::RotateY(ry);
			viewMatrix = viewMatrix * OpenGlMatrix::RotateZ(rz);
			viewMatrix = viewMatrix * OpenGlMatrix::Translate(-twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
			viewMatrix.Multiply();

			// Draw map points
			glPointSize(5.0f);
			glColor3f(1.0f, 1.0f, 1.0f);
			glBegin(GL_POINTS);
			for (int i = 0; i < worldPoints.size(); ++i)
			{
				glVertex3f(worldPoints[i].x, -worldPoints[i].y, -worldPoints[i].z);
			}
			glEnd();

			// Draw cubes
			for (ARCube cube : cubes)
			{
				glPushMatrix();
				OpenGlMatrix mvMatrix = IdentityMatrix();
				float rzz = atanf(cube.normal[0] / cube.normal[1]);
				float rxx = atanf(cube.normal[2] / cube.normal[1]);
				float ryy = atanf(cube.normal[0] / cube.normal[2]);

				mvMatrix = mvMatrix * OpenGlMatrix::Translate(cube.location.x, -cube.location.y, -cube.location.z);
				mvMatrix = mvMatrix * OpenGlMatrix::RotateX(rxx);
				mvMatrix = mvMatrix * OpenGlMatrix::RotateY(ryy);
				mvMatrix = mvMatrix * OpenGlMatrix::RotateZ(rzz);
				mvMatrix.Multiply();
				glDrawColouredCube(-cube.scale, cube.scale);
				glPopMatrix();
			}
		}

		// Swap frames and Process Events
		FinishFrame();
	}
}
void initPangolinARWindow(ARWindowState& result, const string& settingsPath, int width, int height)
{
	CreateWindowAndBind("Main", width, height);
	glEnable(GL_DEPTH_TEST);

	result.viewReal = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-width / (double)height).SetHandler(&result.handler);
	result.viewCam = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-width / (double)height).SetHandler(&result.handler);

	FileStorage fsSettings(settingsPath, FileStorage::READ);
	result.cameraMatrix = ProjectionMatrix(
		width,
		height,
		fsSettings["Camera.fx"],
		fsSettings["Camera.fy"],
		fsSettings["Camera.cx"],
		fsSettings["Camera.cy"],
		0.2,
		100
	);
	result.camState = OpenGlRenderState(result.cameraMatrix);
	result.K = Mat::eye(3, 3, CV_32F);
	result.K.at<float>(0, 0) = fsSettings["Camera.fx"];
	result.K.at<float>(1, 1) = fsSettings["Camera.fy"];
	result.K.at<float>(0, 2) = fsSettings["Camera.cx"];
	result.K.at<float>(1, 2) = fsSettings["Camera.cy"];

	result.distCoeffs = Mat::zeros(5, 1, CV_32F);
	result.distCoeffs.at<float>(0) = fsSettings["Camera.k1"];
	result.distCoeffs.at<float>(1) = fsSettings["Camera.k2"];
	result.distCoeffs.at<float>(2) = fsSettings["Camera.p1"];
	result.distCoeffs.at<float>(3) = fsSettings["Camera.p2"];
	result.distCoeffs.at<float>(4) = fsSettings["Camera.k3"];
}

