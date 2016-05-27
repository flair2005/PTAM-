// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This file declares the MapPoint class
// 
// The map is made up of a bunch of mappoints.
// Each one is just a 3D point in the world;
// it also includes information on where and in which key-frame the point was
// originally made from, so that pixels from that keyframe can be used
// to search for that point.
// Also stores stuff like inlier/outlier counts, and privat information for 
// both Tracker and MapMaker.

#ifndef __MAP_POINT_H
#define __MAP_POINT_H
#include <TooN/TooN.h>
using namespace TooN;
#include <cvd/image_ref.h>
#include <cvd/timer.h>
#include <set>

class KeyFrame;
class TrackerData;
class MapMakerData;

//地图点结构体
struct MapPoint
{
  // Constructor inserts sensible defaults and zeros pointers.
  inline MapPoint()
  {
    bBad = false;
    pTData = NULL;
    pMMData = NULL;
    nMEstimatorOutlierCount = 0;
    nMEstimatorInlierCount = 0;
    dCreationTime = CVD::timer.get_time();
  };
  
  // Where in the world is this point? The main bit of information, really.
  Vector<3> v3WorldPos;  //地图点的坐标(patch中心点的坐标)
  // Is it a dud? In that case it'll be moved to the trash soon.
  //只是质量，质量不好要被丢到trash里面去
  bool bBad;
  
  // What pixels should be used to search for this point?
  KeyFrame *pPatchSourceKF; // The KeyFrame the point was originally made in   地图点对应的初始关键帧
  int nSourceLevel;         // Pyramid level in source KeyFrame				   关键帧中的金字塔层数
  CVD::ImageRef irCenter;   // This is in level-coords in the source pyramid level 金字塔层数中的坐标
  
  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  //下面的东西是需要求解affinewarp需要的中间向量
  Vector<3> v3Center_NC;             // Unit vector in Source-KF coords pointing at the patch center//指向patch中心的单位向量(相机坐标系中下同) 指向的是摄像机坐标系中的实际坐标(图见有道云笔记)
									 // 即是patch center的反投影线的方向向量(反投影线会通过对应的像素点)
  Vector<3> v3OneDownFromCenter_NC;  // Unit vector in Source-KF coords pointing towards one pixel down of the patch center 指向patch中心向下一个像素点的单位向量()
  Vector<3> v3OneRightFromCenter_NC; // Unit vector in Source-KF coords pointing towards one pixel right of the patch center指向pathch中心向右一个像素点的单位向量
  Vector<3> v3Normal_NC;             // Unit vector in Source-KF coords indicating patch normal    patch的法向量(0,0,-1)
  
  Vector<3> v3PixelDown_W;           // 3-Vector in World coords corresponding to a one-pixel move down the source image   patch中向下移动一个像素在世界坐标系中对应的向量
  Vector<3> v3PixelRight_W;          // 3-Vector in World coords corresponding to a one-pixel move right the source imagg  patch中向右移动一个像素在世界坐标系中对应的向量
  void RefreshPixelVectors();        // Calculates above two vectors   //计算上面的两个向量
  
  // Info for the Mapmaker (not to be trashed by the tracker:) //没有被tracker放进trash里面的特征点
  MapMakerData *pMMData;
  
  // Info for the Tracker (not to be trashed by the MapMaker:) //没有被mapmaker放进trash里面的特征点
  TrackerData *pTData;
  
  // Info provided by the tracker for the mapmaker:
  int nMEstimatorOutlierCount;					//tracker提供的Outlier数量
  int nMEstimatorInlierCount;					//tracker提供的Inlier数量
  
  // Random junk (e.g. for visualisation)
  double dCreationTime; //timer.get_time() time of creation
};

#endif
