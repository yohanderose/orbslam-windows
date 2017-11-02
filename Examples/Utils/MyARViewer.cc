#include "MyARViewer.h"

#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/handler/handler.h>
#include <System.h>


using namespace std;
using namespace cv;
using namespace pangolin;


MyARViewer::MyARViewer(ORB_SLAM2::System* system, const string &settingsPath, int width, int height)
	: system(system), settingsPath(settingsPath), width(width), height(height)
{
	CreateWindowAndBind("MyARViewer", width, height);
	glEnable(GL_DEPTH_TEST);

	winState.handler.SetContext(this);
	winState.viewReal = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-width / (double)height).SetHandler(&winState.handler);
	winState.viewCam = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, (double)-width / (double)height).SetHandler(&winState.handler);

	FileStorage fsSettings(settingsPath, FileStorage::READ);
	winState.cameraMatrix = ProjectionMatrix(
		width,
		height,
		fsSettings["Camera.fx"],
		fsSettings["Camera.fy"],
		fsSettings["Camera.cx"],
		fsSettings["Camera.cy"],
		0.2,
		100
	);
	winState.camState = OpenGlRenderState(winState.cameraMatrix);
	winState.K = Mat::eye(3, 3, CV_32F);
	winState.K.at<float>(0, 0) = fsSettings["Camera.fx"];
	winState.K.at<float>(1, 1) = fsSettings["Camera.fy"];
	winState.K.at<float>(0, 2) = fsSettings["Camera.cx"];
	winState.K.at<float>(1, 2) = fsSettings["Camera.cy"];

	winState.distCoeffs = Mat::zeros(5, 1, CV_32F);
	winState.distCoeffs.at<float>(0) = fsSettings["Camera.k1"];
	winState.distCoeffs.at<float>(1) = fsSettings["Camera.k2"];
	winState.distCoeffs.at<float>(2) = fsSettings["Camera.p1"];
	winState.distCoeffs.at<float>(3) = fsSettings["Camera.p2"];
	winState.distCoeffs.at<float>(4) = fsSettings["Camera.k3"];
}

MyARViewer::~MyARViewer()
{
}

inline void selectRandomMapPoints(vector<ORB_SLAM2::MapPoint *>& vMPgood, ORB_SLAM2::MapPoint *randomSampleMP[3], Eigen::Vector3d randomSample[3])
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

bool MyARViewer::getPlaneRansac(Plane & p, vector<ORB_SLAM2::MapPoint*> vMP, vector<bool> vbMap)
{
	using namespace Eigen;

	int N = vMP.size();

	// store the "true" map points to make random sampling easier
	vector<ORB_SLAM2::MapPoint *> vMPgood;
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
		ORB_SLAM2::MapPoint *randomSampleMP[3];
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
		return true;
	}
	else
	{
		return false;
	}
}

bool MyARViewer::renderARFrame(Mat cameraPose, Mat im)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (!ShouldQuit())
	{
		// correct the input frame
		cvtColor(im, im, CV_BGR2RGB);
		Mat camFrameUn;
		undistort(im, camFrameUn, winState.K, winState.distCoeffs);

		// render the camera feed to the background
		glDisable(GL_DEPTH_TEST);
		GlTexture imageTexture(camFrameUn.cols, camFrameUn.rows, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
		imageTexture.Upload(camFrameUn.ptr(), GL_RGB, GL_UNSIGNED_BYTE);
		winState.viewReal.Activate();
		glColor3f(1.0, 1.0, 1.0);
		imageTexture.RenderToViewport(true);

		// render the AR objects
		if (!cameraPose.empty())
		{
			lastCameraPose = cameraPose.clone();

			glEnable(GL_DEPTH_TEST);
			winState.viewCam.Activate(winState.camState);

			// Extract the pose information
			Mat Rwc = cameraPose.rowRange(0, 3).colRange(0, 3).t();
			Mat twc = -Rwc*cameraPose.rowRange(0, 3).col(3);
			float rx = atan2f(Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
			float ry = atan2f(-Rwc.at<float>(2, 0), sqrtf(pow(Rwc.at<float>(2, 1), 2) + pow(Rwc.at<float>(2, 2), 2)));
			float rz = atan2f(Rwc.at<float>(1, 0), Rwc.at<float>(0, 0));

			// Transform into camera space
			OpenGlMatrix viewMatrix = IdentityMatrix();
			viewMatrix = viewMatrix * OpenGlMatrix::RotateX(-rx);
			viewMatrix = viewMatrix * OpenGlMatrix::RotateY(ry);
			viewMatrix = viewMatrix * OpenGlMatrix::RotateZ(rz);
			viewMatrix = viewMatrix * OpenGlMatrix::Translate(-twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
			viewMatrix.Multiply();

			// Draw camera space map points
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
				// Transform into per-cube model space
				glPushMatrix();
				OpenGlMatrix mvMatrix = IdentityMatrix();
				float rzz = atanf(cube.normal[0] / cube.normal[1]);
				float rxx = atanf(cube.normal[2] / cube.normal[1]);
				float ryy = atanf(cube.normal[0] / cube.normal[2]);

				mvMatrix = mvMatrix * OpenGlMatrix::Translate(cube.location.x, -cube.location.y + cube.scale, -cube.location.z);
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

		return ShouldQuit();
	}
}

