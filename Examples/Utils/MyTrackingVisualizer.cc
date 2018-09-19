#include "MyTrackingVisualizer.h"

#include <string>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;


void MyTrackingVisualizer::readFromPlyFile()
{
	ifstream ifs(mModelFilename, ifstream::in);
	if (!ifs.is_open())
	{
		cout << "Error: could not open ply file " << mModelFilename << endl;
		exit(1);
	}
	char line[256];
	bool readingHeader = true;
	while (!ifs.eof())
	{
		ifs.getline(line, 256);
		if (!readingHeader)
		{
			float x, y, z;
			int r, g, b;
			stringstream ss(line);
			ss >> x >> y >> z >> r >> g >> b;
			mPoints.push_back(Vec4f(x, y, z, 1.0f));
			mColors.push_back(Vec3i(b, g, r));
		}
		if (strcmp(line, "end_header") == 0)
		{
			readingHeader = false;
		}

	}
}


MyTrackingVisualizer::MyTrackingVisualizer()
{

}
MyTrackingVisualizer::MyTrackingVisualizer(const string &modelFilename)
{
	mModelFilename = modelFilename;
	mR = Mat::eye(Size(3, 3), CV_32F);
	mt = Mat::zeros(Size(1, 3), CV_32F);
	mModelAligned = false;
	mWindowAlive = false;	
	readFromPlyFile();
}
bool MyTrackingVisualizer::alignModel(const Mat &frame, const Mat &K)
{
	string winTitle = "Use WASD to move, mouse to rotate, mouse wheel to scale";
	if (!mWindowAlive)
	{
		namedWindow(winTitle, CV_WINDOW_AUTOSIZE);
		//setMouseCallback(mouseFunc);
	}

	Mat tmp = Mat::zeros(frame.size(), CV_8UC3);
	//frame.copyTo(tmp);
	for (int i = 0; i < mPoints.size(); ++i)
	{
		Mat modelPoint = Mat(mPoints[i]);
		Mat currPose = getReferencePose();
		Mat imagePoint = K * currPose * modelPoint;

		Vec3d imagePointVec = Vec3d(imagePoint);
		imagePointVec[0] = imagePointVec[0] / imagePointVec[2];
		imagePointVec[1] = imagePointVec[1] / imagePointVec[2];

		if (imagePointVec[0] > 0 && imagePointVec[0] < tmp.cols &&
			imagePointVec[1] > 0 && imagePointVec[1] < tmp.rows && 
			imagePointVec[2] > 0)
		{
			circle(tmp, Point(imagePointVec[0], imagePointVec[1]), 1, Scalar(mColors[i][0], mColors[i][1], mColors[i][2]), 2);
		}
	}
	imshow(winTitle, tmp);
	char key = waitKey(1);
	if ('s' == key)
	{
		mt.at<float>(0, 2) = mt.at<float>(0, 2) - 0.1;
	}
	if ('w' == key)
	{
		mt.at<float>(0, 2) = mt.at<float>(0, 2) + 0.1;
	}
	if ('a' == key)
	{
		mt.at<float>(0, 0) = mt.at<float>(0, 0) + 0.1;
	}
	if ('d' == key)
	{
		mt.at<float>(0, 0) = mt.at<float>(0, 0) - 0.1;
	}
	if ('z' == key)
	{
		mt.at<float>(0, 1) = mt.at<float>(0, 1) - 0.1;
	}
	if ('x' == key)
	{
		mt.at<float>(0, 1) = mt.at<float>(0, 1) + 0.1;
	}

	if (!mModelAligned)
	{
		return false;
	}
	else
	{
		destroyWindow(winTitle);
		return true;
	}
}
Mat MyTrackingVisualizer::getReferencePose()
{
	Mat pose(Size(4, 3), CV_32F);
	Mat R_roi = pose(Rect(0, 0, 3, 3));
	Mat t_roi = pose(Rect(3, 0, 1, 3));
	mR.copyTo(R_roi);
	mt.copyTo(t_roi);
	return pose;
}

