// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.
// 把图像变小并且模糊化
#ifndef __SMALLBLURRYIMAGE_H
#define __SMALLBLURRYIMAGE_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <TooN/se2.h>
#include <TooN/se3.h>
#include "KeyFrame.h"
#include "ATANCamera.h"

class SmallBlurryImage
{
 public:
  SmallBlurryImage();
  SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
  void MakeJacs();
  double ZMSSD(SmallBlurryImage &other);
  std::pair<SE2<>,double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);
  static SE3<> SE3fromSE2(SE2<> se2, ATANCamera camera);
  
protected:
  CVD::Image<CVD::byte> mimSmall;     	//把KF缩小了的图像
  CVD::Image<float> mimTemplate;      	//把KF缩小了，并模糊了的图像
  CVD::Image<Vector<2> > mimImageJacs;	//图像的一阶导数
  bool mbMadeJacs;                      //是否计算了Jacabi矩阵
  static CVD::ImageRef mirSize;         //图像的大小
};



#endif









