// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.
// ��ͼ���С����ģ����
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
  CVD::Image<CVD::byte> mimSmall;     	//��KF��С�˵�ͼ��
  CVD::Image<float> mimTemplate;      	//��KF��С�ˣ���ģ���˵�ͼ��
  CVD::Image<Vector<2> > mimImageJacs;	//ͼ���һ�׵���
  bool mbMadeJacs;                      //�Ƿ������Jacabi����
  static CVD::ImageRef mirSize;         //ͼ��Ĵ�С
};



#endif









