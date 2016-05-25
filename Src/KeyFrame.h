// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <set>
#include <map>

class MapPoint;
class SmallBlurryImage;

#define LEVELS 4

// Candidate: a feature in an image which could be made into a map point
// 可能会被选为地图点的信息
// 即Shi-Tomasi Score大于某一个阀值的特征点
struct Candidate
{
  CVD::ImageRef irLevelPos;		//对应层数的坐标
  Vector<2> v2RootPos;			//相对于金字塔0层的坐标
  double dSTScore;              //ShiTomasi Score
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
//存储地图点的图像信息
struct Measurement
{
  int nLevel;   // Which image level?  金字塔的层数
  bool bSubPix; // Has this measurement been refined to sub-pixel level? 是否是亚像素的精度
  Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO  在0层金字塔的位置
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome? 地图点的来源
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
// 金字塔层数的结构体
struct Level
{
  inline Level()
  {
    bImplaneCornersCached = false;
  };
  
  CVD::Image<CVD::byte> im;                // The pyramid level pixels	该层对应的图像
  std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level 该层的所有的FAST角点坐标
  std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access 该层所有角点对应的在图像中的行坐标
  std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners							普通的FAST经过非极大值抑制之后留下来的角点
  Level& operator=(const Level &rhs);
  
  std::vector<Candidate> vCandidates;   // Potential locations of new map points				该层的潜在的地图点的信息
  
  bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
  std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
  inline KeyFrame()
  {
    pSBI = NULL;
  }
  SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation  关键帧的在世界坐标系中的位置
  bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)       是否是固定的帧(只有第一帧是固定的)
  Level aLevels[LEVELS];  // Images, corners, etc lives in this array of pyramid levels				   金字塔（一共有4层）
  std::map<MapPoint*, Measurement> mMeasurements;           // All the measurements associated with the keyframe  关键帧中所有地图点信息
  
  //从图像生成关键帧 包括构建金字塔和tracker线程需要的所有信息
  void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im);   // This takes an image and calculates pyramid levels etc to fill the 
                                                            // keyframe data structures with everything that's needed by the tracker..
  void MakeKeyFrame_Rest();                                 // ... while this calculates the rest of the data which the mapmaker needs.
  
  double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.   关键帧深度的均值
  double dSceneDepthSigma;	   //												 关键帧深度的方差
  
  SmallBlurryImage *pSBI; // The relocaliser uses this							 用于重定位，看不懂
};

typedef std::map<MapPoint*, Measurement>::iterator meas_it;  // For convenience, and to work around an emacs paren-matching bug


#endif

