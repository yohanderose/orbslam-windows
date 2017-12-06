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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <opencv2/core.hpp>

#include <mutex>

namespace ORB_SLAM2
{

	struct EditHandler : public pangolin::Handler
	{
		EditHandler()
		{ }
		EditHandler(pangolin::OpenGlRenderState& s_cam)
			: s_cam(&s_cam)
		{ }
		System *ctx;
		Viewer *viewer;
		pangolin::OpenGlRenderState *s_cam;

		int rx1, ry1;
		void SetContext(System *_ctx, Viewer *_viewer)
		{
			ctx = _ctx;
			viewer = _viewer;
		}
		void Mouse(pangolin::View& d_cam, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
		{
			if (ctx && viewer)
			{
				if (button == pangolin::MouseButtonLeft)
				{
					// button released
					if (button_state == 0)
					{
						viewer->ClearSelectionBoundingBox();

						// set the selection region second co-ordinate
						int rx2 = x;
						int ry2 = y;

						// get the stuff required for glProject
						GLint viewport[4] = { d_cam.v.l, d_cam.v.b, d_cam.v.w, d_cam.v.h };
						double *modelViewMat = s_cam->GetModelViewMatrix().m;
						double *projectionMat = s_cam->GetProjectionMatrix().m;

						// loop over all the map points
						std::vector<MapPoint *> mps = ctx->GetAllMapPoints();
						for (MapPoint *mp : mps)
						{
							cv::Mat wp = mp->GetWorldPos();

							// project the map point into screen co-ordinates
							double winX, winY, winZ;
							pangolin::glProject(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2), modelViewMat, projectionMat, viewport, &winX, &winY, &winZ);

							// check if it is contained
							if (winX > min(rx1, rx2) &&
								winX < max(rx1, rx2) &&
								winY > min(ry1, ry2) &&
								winY < max(ry1, ry2))
							{
								mp->SetSelectedFlag(true);
							}
						}
					}
					// button pressed
					else
					{
						// set the selection region first co-ordinate
						rx1 = x;
						ry1 = y;
					}
				}
			}
		}
		void Keyboard(pangolin::View&, unsigned char key, int x, int y, bool pressed)
		{
			if (key == 127 && !pressed) // delete
			{
				Map *map = ctx->GetMap();
				// loop over all the map points
				std::vector<MapPoint *> mps = ctx->GetAllMapPoints();
				for (MapPoint *mp : mps)
				{
					if (mp->isSelected())
						map->EraseMapPoint(mp);
				}
			}
		}
		void MouseMotion(pangolin::View& d_cam, int x, int y, int button_state)
		{
			if (viewer)
			{
				viewer->SetSelectionBoundingBox(rx1, ry1, x, y);
			}
		}
	};

	Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, bool bReuse) :
		mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
		mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		float fps = fSettings["Camera.fps"];
		if (fps < 1)
			fps = 30;
		mT = 1e3 / fps;

		mImageWidth = fSettings["Camera.width"];
		mImageHeight = fSettings["Camera.height"];
		if (mImageWidth < 1 || mImageHeight < 1)
		{
			mImageWidth = 640;
			mImageHeight = 480;
		}

		mViewpointX = fSettings["Viewer.ViewpointX"];
		mViewpointY = fSettings["Viewer.ViewpointY"];
		mViewpointZ = fSettings["Viewer.ViewpointZ"];
		mViewpointF = fSettings["Viewer.ViewpointF"];
		mbReuse = bReuse;
		mbEdit = false;
	}

	void Viewer::Run()
	{
		mbFinished = false;
		mbStopped = false;

		pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer", 1024, 768);

		// 3D Mouse handler requires depth testing to be enabled
		glEnable(GL_DEPTH_TEST);

		// Issue specific OpenGl we might need
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
		pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
		pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
		pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
		pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
		pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", mbReuse, true);
		pangolin::Var<bool> menuReset("menu.Reset", false, false);
		pangolin::Var<bool> menuEditMode("menu.Edit Mode", mbEdit, true);

		// Define Camera Render Object (for view / scene browsing)
		pangolin::OpenGlRenderState s_cam(
			pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
			pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
		);

		// Add named OpenGL viewport to window and provide 3D Handler
		handler3D = new pangolin::Handler3D(s_cam);
		handlerEdit = new EditHandler(s_cam);
		handlerEdit->SetContext(mpSystem, this);

		pangolin::View& d_cam = pangolin::CreateDisplay()
			.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
			.SetHandler(handler3D);

		pangolin::OpenGlMatrix Twc;
		Twc.SetIdentity();

		cv::namedWindow("ORB-SLAM2: Current Frame");

		bool bFollow = true;
		bool bLocalizationMode = mbReuse;
		bool bEditMode = mbEdit;

		while (1)
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

			if (menuFollowCamera && bFollow)
			{
				s_cam.Follow(Twc);
			}
			else if (menuFollowCamera && !bFollow)
			{
				s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
				s_cam.Follow(Twc);
				bFollow = true;
			}
			else if (!menuFollowCamera && bFollow)
			{
				bFollow = false;
			}

			if (menuLocalizationMode && !bLocalizationMode)
			{
				mpSystem->ActivateLocalizationMode();
				bLocalizationMode = true;
			}
			else if (!menuLocalizationMode && bLocalizationMode)
			{
				mpSystem->DeactivateLocalizationMode();
				bLocalizationMode = false;
			}

			// bakelew Map edit
			if (menuEditMode && !bEditMode)
			{
				mpSystem->ActivateEditMode();
				bEditMode = true;
				d_cam.SetHandler(handlerEdit);
			}
			else if (!menuEditMode && bEditMode)
			{
				mpSystem->DeactivateEditMode();
				bEditMode = false;
				d_cam.SetHandler(handler3D);
			}

			if (!mvSelectionBoundingBox.empty())
				DrawSelectionBoundingBox(d_cam);

			d_cam.Activate(s_cam);
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			mpMapDrawer->DrawCurrentCamera(Twc);
			if (menuShowKeyFrames || menuShowGraph)
				mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
			if (menuShowPoints)
				mpMapDrawer->DrawMapPoints();

			pangolin::FinishFrame();

			cv::Mat im = mpFrameDrawer->DrawFrame();
			cv::imshow("ORB-SLAM2: Current Frame", im);
			cv::waitKey(mT);

			if (menuReset)
			{
				menuShowGraph = true;
				menuShowKeyFrames = true;
				menuShowPoints = true;
				menuLocalizationMode = false;
				if (bLocalizationMode)
					mpSystem->DeactivateLocalizationMode();
				bLocalizationMode = false;
				bFollow = true;
				menuFollowCamera = true;
				mpSystem->Reset();
				menuReset = false;
			}

			if (Stop())
			{
				while (isStopped())
				{
					usleep(3000);
				}
			}

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	void Viewer::RequestFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}

	bool Viewer::CheckFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinishRequested;
	}

	void Viewer::SetFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		mbFinished = true;
	}

	bool Viewer::isFinished()
	{
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinished;
	}

	void Viewer::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (!mbStopped)
			mbStopRequested = true;
	}

	bool Viewer::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	bool Viewer::Stop()
	{
		unique_lock<mutex> lock(mMutexStop);
		unique_lock<mutex> lock2(mMutexFinish);

		if (mbFinishRequested)
			return false;
		else if (mbStopRequested)
		{
			mbStopped = true;
			mbStopRequested = false;
			return true;
		}

		return false;

	}

	void Viewer::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
	}

	void Viewer::SetSelectionBoundingBox(int x1, int y1, int x2, int y2)
	{
		ClearSelectionBoundingBox();
		mvSelectionBoundingBox.push_back(x1);
		mvSelectionBoundingBox.push_back(y1);
		mvSelectionBoundingBox.push_back(x2);
		mvSelectionBoundingBox.push_back(y2);
	}

	void Viewer::ClearSelectionBoundingBox()
	{
		mvSelectionBoundingBox.clear();
	}

	void Viewer::DrawSelectionBoundingBox(pangolin::View &d_cam)
	{
		d_cam.ActivatePixelOrthographic();
		glColor3f(0, 0.8, 0.7);
		pangolin::glDrawRectPerimeter(mvSelectionBoundingBox[0] - d_cam.v.l, mvSelectionBoundingBox[1] - d_cam.v.b, mvSelectionBoundingBox[2] - d_cam.v.l, mvSelectionBoundingBox[3] - d_cam.v.b);
	}
}