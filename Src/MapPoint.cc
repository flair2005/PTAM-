// Copyright 2008 Isis Innovation Limited
#include "MapPoint.h"
#include "KeyFrame.h"
void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *pPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3PlanePoint_C = k.se3CfromW * v3WorldPos;   //得到patch在Source KF相机坐标系里的坐标
  
  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  // 计算出来对应的patch离相机平面的距离(沿着法向量的距离) v3PlanePoint_C也相当于一个向量 乘积的结果为norm(v3PlanePoint_c)*cos(theta)
  // theta为两个向量的夹角。得到的结果为向量v3PlanePoint_C在Patch的Normal Vector方向上的投影的长度
  // a*b = a*b*cos(theta)
  double dCamHeight = fabs(v3PlanePoint_C * v3Normal_NC);

  //从图像平面到patch plane的投影需要用的相似三角形，此处先计算对应的方向向量在法向量上的投影。(具体图像在自己的印象笔记中)
  double dPixelRate = fabs(v3Center_NC * v3Normal_NC);
  double dOneRightRate = fabs(v3OneRightFromCenter_NC * v3Normal_NC);
  double dOneDownRate = fabs(v3OneDownFromCenter_NC * v3Normal_NC);
  
  // Find projections onto plane
  // 把Source Pyramid level中的向量投影到patch plane里面。
  // 得到Patch的中心坐标、右边一个、下面一个在相机平面中的向量
  // 用相似三角形对应边的比相等，求得在摄像机坐标系下三个点的向量(也可以认为是坐标)
  Vector<3> v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  Vector<3> v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Vector<3> v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;
  
  // Find differences of these projections in the world frame
  // 找到这些投影在世界坐标系中的向量
  // 把patch plane里面的向量投影到世界坐标系里面
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  
