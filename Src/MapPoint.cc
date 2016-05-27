// Copyright 2008 Isis Innovation Limited
#include "MapPoint.h"
#include "KeyFrame.h"
void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *pPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3PlanePoint_C = k.se3CfromW * v3WorldPos;   //�õ�patch��Source KF�������ϵ�������
  
  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  // ���������Ӧ��patch�����ƽ��ľ���(���ŷ������ľ���) v3PlanePoint_CҲ�൱��һ������ �˻��Ľ��Ϊnorm(v3PlanePoint_c)*cos(theta)
  // thetaΪ���������ļнǡ��õ��Ľ��Ϊ����v3PlanePoint_C��Patch��Normal Vector�����ϵ�ͶӰ�ĳ���
  // a*b = a*b*cos(theta)
  double dCamHeight = fabs(v3PlanePoint_C * v3Normal_NC);

  //��ͼ��ƽ�浽patch plane��ͶӰ��Ҫ�õ����������Σ��˴��ȼ����Ӧ�ķ��������ڷ������ϵ�ͶӰ��(����ͼ�����Լ���ӡ��ʼ���)
  double dPixelRate = fabs(v3Center_NC * v3Normal_NC);
  double dOneRightRate = fabs(v3OneRightFromCenter_NC * v3Normal_NC);
  double dOneDownRate = fabs(v3OneDownFromCenter_NC * v3Normal_NC);
  
  // Find projections onto plane
  // ��Source Pyramid level�е�����ͶӰ��patch plane���档
  // �õ�Patch���������ꡢ�ұ�һ��������һ�������ƽ���е�����
  // �����������ζ�Ӧ�ߵı���ȣ���������������ϵ�������������(Ҳ������Ϊ������)
  Vector<3> v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  Vector<3> v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Vector<3> v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;
  
  // Find differences of these projections in the world frame
  // �ҵ���ЩͶӰ����������ϵ�е�����
  // ��patch plane���������ͶӰ����������ϵ����
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  
