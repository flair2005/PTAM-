// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"

Map::Map()
{
  Reset();
}

//�����������
void Map::Reset()
{
  for(unsigned int i=0; i<vpPoints.size(); i++)
    delete vpPoints[i];
  vpPoints.clear();
  bGood = false;
  EmptyTrash();
}

//�����е�bad point���ƶ�trash��ȥ
void Map::MoveBadPointsToTrash()
{
  int nBad = 0;
  for(int i = vpPoints.size()-1; i>=0; i--)
    {
      if(vpPoints[i]->bBad)
	{
	  vpPointsTrash.push_back(vpPoints[i]);
	  vpPoints.erase(vpPoints.begin() + i);
	  nBad++;
	}
    };
};

//��trash���
void Map::EmptyTrash()
{
  for(unsigned int i=0; i<vpPointsTrash.size(); i++)
    delete vpPointsTrash[i];
  vpPointsTrash.clear();
};




