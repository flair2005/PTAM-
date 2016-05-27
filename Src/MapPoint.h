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

//��ͼ��ṹ��
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
  Vector<3> v3WorldPos;  //��ͼ�������(patch���ĵ������)
  // Is it a dud? In that case it'll be moved to the trash soon.
  //ֻ����������������Ҫ������trash����ȥ
  bool bBad;
  
  // What pixels should be used to search for this point?
  KeyFrame *pPatchSourceKF; // The KeyFrame the point was originally made in   ��ͼ���Ӧ�ĳ�ʼ�ؼ�֡
  int nSourceLevel;         // Pyramid level in source KeyFrame				   �ؼ�֡�еĽ���������
  CVD::ImageRef irCenter;   // This is in level-coords in the source pyramid level �����������е�����
  
  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  //����Ķ�������Ҫ���affinewarp��Ҫ���м�����
  Vector<3> v3Center_NC;             // Unit vector in Source-KF coords pointing at the patch center//ָ��patch���ĵĵ�λ����(�������ϵ����ͬ) ָ��������������ϵ�е�ʵ������(ͼ���е��Ʊʼ�)
									 // ����patch center�ķ�ͶӰ�ߵķ�������(��ͶӰ�߻�ͨ����Ӧ�����ص�)
  Vector<3> v3OneDownFromCenter_NC;  // Unit vector in Source-KF coords pointing towards one pixel down of the patch center ָ��patch��������һ�����ص�ĵ�λ����()
  Vector<3> v3OneRightFromCenter_NC; // Unit vector in Source-KF coords pointing towards one pixel right of the patch centerָ��pathch��������һ�����ص�ĵ�λ����
  Vector<3> v3Normal_NC;             // Unit vector in Source-KF coords indicating patch normal    patch�ķ�����(0,0,-1)
  
  Vector<3> v3PixelDown_W;           // 3-Vector in World coords corresponding to a one-pixel move down the source image   patch�������ƶ�һ����������������ϵ�ж�Ӧ������
  Vector<3> v3PixelRight_W;          // 3-Vector in World coords corresponding to a one-pixel move right the source imagg  patch�������ƶ�һ����������������ϵ�ж�Ӧ������
  void RefreshPixelVectors();        // Calculates above two vectors   //�����������������
  
  // Info for the Mapmaker (not to be trashed by the tracker:) //û�б�tracker�Ž�trash�����������
  MapMakerData *pMMData;
  
  // Info for the Tracker (not to be trashed by the MapMaker:) //û�б�mapmaker�Ž�trash�����������
  TrackerData *pTData;
  
  // Info provided by the tracker for the mapmaker:
  int nMEstimatorOutlierCount;					//tracker�ṩ��Outlier����
  int nMEstimatorInlierCount;					//tracker�ṩ��Inlier����
  
  // Random junk (e.g. for visualisation)
  double dCreationTime; //timer.get_time() time of creation
};

#endif
