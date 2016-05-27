//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>


class TrackerData;
struct Trail    // This struct is used for initial correspondences of the first stereo pair. 只是在初始建图的是偶使用
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);
  
  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw); 

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  
protected:
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct     当前帧
  
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes        地图(包括了地图点和关键帧)
  MapMaker &mMapMaker;            // The class which maintains the map				    更新和优化地图的类
  ATANCamera mCamera;             // Projection model									摄像机模型(用来计算投影)
  Relocaliser mRelocaliser;       // Relocalisation module								重定位模块

  CVD::ImageRef mirSize;          // Image size of whole image							图像大小
  
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences): 下面的成员都是用来计算初始化地图的
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // Methods for tracking the map once it has been made:有地图之后，来跟踪地图
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD, 
		      int nRange, 
		      int nFineIts);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); // Updates pose from found points.
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.   摄像机位姿
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model													相机运动模型
  double mdVelocityMagnitude;     // Used to decide on coarse tracking 								决定是否需要进行粗跟踪(当运动速度过大的时候，就需要进行粗跟踪)
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.        被平均场景深度缩放之后的速度大小
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?					指示是否需要进行Coarse跟踪
  
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?					指示是否需要进行绘图
  
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset								自从复位之后，处理过的总帧数
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.								
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe  增加新的关键帧
  
  // Tracking quality control:  跟跟踪质量相关的量
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.						尝试重新进行定位
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!						恢复定位之后，总是要进行Coarse跟踪

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;																	//上一帧的SBI
  SmallBlurryImage *mpSBIThisFrame;																	//这一帧的SBI
  void CalcSBIRotation();																			//计算两帧之间的旋转
  Vector<6> mv6SBIRot;																				
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
  
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
};

#endif






