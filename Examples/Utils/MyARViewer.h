#pragma once


#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/handler/handler.h>
#include <System.h>


class MyARViewer
{

	struct MyEventHandler : public pangolin::Handler
	{
		MyARViewer *ctx;
		void SetContext(MyARViewer *_ctx)
		{
			ctx = _ctx;
		}
		void Mouse(pangolin::View&, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
		{
			if (ctx)
			{
				y = ctx->height - y - 10;
				if (!pressed)
				{
					std::vector<ORB_SLAM2::MapPoint *> vMP = ctx->system->GetTrackedMapPoints();
					int N = vMP.size();
					std::vector<bool> mvbMap(N, false);

					for (int i = 0; i<N; i++)
					{
						ORB_SLAM2::MapPoint* pMP = vMP[i];
						if (pMP)
						{
							//if (!pTracker->mCurrentFrame.mvbOutlier[i]) // not good
							{
								if (pMP->Observations()>0)
									mvbMap[i] = true;
							}
						}
					}

					std::vector<cv::KeyPoint> kps = ctx->system->GetTrackedKeyPointsUn();
					bool found = false;
					if (kps.size() > 0 && kps.size() == vMP.size())
					{
						double minDist = DBL_MAX;
						int minIdx = INT_MAX;
						for (int i = 0; i < kps.size(); ++i)
						{
							if (mvbMap[i])
							{
								double dist = abs(cv::norm(kps[i].pt - cv::Point2f(x, y)));
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
							cv::Mat wp = vMP[minIdx]->GetWorldPos();
							ARCube cube;
							cube.location = cv::Point3f(
								wp.at<float>(0, 0),
								wp.at<float>(1, 0),
								wp.at<float>(2, 0)
							);
							cout << "Placing cube at mapPoint: " << endl;
							cout << cv::Vec3f(wp) << endl;
							cube.scale = 0.005;

							Plane p;
							ctx->getPlaneRansac(p, vMP, mvbMap);
							cube.normal = p.getNormalCV();

							ctx->cubes.push_back(cube);
							++(ctx->cubesIdx);
						}
						else
						{
						}
					}
				}
			}
		}
		void Keyboard(pangolin::View&, unsigned char key, int x, int y, bool pressed)
		{
			if (ctx)
			{
				if (!pressed)
				{
					switch (key)
					{
					case 'a':
					{
						ctx->worldPoints.clear();

						if (ctx->lastCameraPose.empty())
							return;

						std::vector<ORB_SLAM2::MapPoint *> mapPoints = ctx->system->GetAllMapPoints();

						for (std::vector<ORB_SLAM2::MapPoint *>::iterator mit = mapPoints.begin(); mit != mapPoints.end(); mit++)
						{
							if (*mit == NULL)
								continue;

							cv::Mat wp = (*mit)->GetWorldPos();
							ctx->worldPoints.push_back(cv::Point3f(
								wp.at<float>(0, 0),
								wp.at<float>(1, 0),
								wp.at<float>(2, 0)));

							cv::Mat n = (*mit)->GetNormal() * 0.05;
						}
						break;
					}
					case 's':
					{
						ctx->worldPoints.clear();
						break;
					}
					}
				}
			}
		}

		void Special(pangolin::View&, pangolin::InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
		{
			if (ctx)
			{
				if (inType == pangolin::InputSpecialScroll)
				{
					const int scrollSpeed = 24;
					double scaleFactor = p2 / (scrollSpeed * 10.0);
					ctx->cubes[ctx->cubesIdx].scale = ctx->cubes[ctx->cubesIdx].scale * (1 + scaleFactor);
				}
			}
		}
	};
	struct ARWindowState
	{
		pangolin::View viewReal;
		pangolin::View viewCam;
		pangolin::OpenGlRenderState camState;
		pangolin::OpenGlMatrix cameraMatrix;
		MyEventHandler handler;
		cv::Mat K;
		cv::Mat distCoeffs;
	};
	struct ARCube
	{
		cv::Point3d location;
		cv::Vec3d normal;
		double scale;
	};
	struct Plane
	{
		Plane()
		{ }
		Plane(Eigen::Vector3d n, Eigen::Vector3d p)
			: mvN(n), mvP(p)
		{ }
		Plane(cv::Vec3d n, cv::Vec3d p)
		{
			mvN = Eigen::Vector3d(n[0], n[1], n[2]);
			mvP = Eigen::Vector3d(p[0], p[1], p[2]);
		}
		Eigen::Vector3d mvN;
		Eigen::Vector3d mvP;

		cv::Vec3d getNormalCV()
		{

			return cv::Vec3d(mvN[0], mvN[1], mvN[2]);
		}
		cv::Vec3d getPointCV()
		{
			return cv::Vec3d(mvP[0], mvP[1], mvP[2]);
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

public:
	MyARViewer(ORB_SLAM2::System* system, const std::string &settingsPath, int width, int height);
	~MyARViewer();

private:
	ORB_SLAM2::System *system;
	ARWindowState winState;
	std::string settingsPath;
	int width;
	int height;

	std::vector<cv::Point3f> worldPoints;
	std::vector<ARCube> cubes;
	std::vector<cv::Point3f> groundPlaneCorners;
	int cubesIdx = -1;

	cv::Mat lastCameraPose;

public:
	bool getPlaneRansac(Plane &p, std::vector<ORB_SLAM2::MapPoint *> vMP, std::vector<bool> vbMap);
	bool renderARFrame(cv::Mat cameraPose, cv::Mat im);

};

