// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

Map::Map()
{
  Reset();
}

Map::~Map() { Reset(); }

void Map::Reset()
{
  for(unsigned int i=0; i<vpPoints.size(); i++)
    delete vpPoints[i];
  vpPoints.clear();
  for(auto* k:vpKeyFrames) delete k;
  vpKeyFrames.clear();
#ifdef IPTAM_MOBILE_PAPER
  for(auto* k:vpRetainedKeyFrames) delete k;
  vpRetainedKeyFrames.clear();
#endif
  bGood = false;
  EmptyTrash();
}

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

void Map::EmptyTrash()
{
  for(unsigned int i=0; i<vpPointsTrash.size(); i++)
    delete vpPointsTrash[i];
  vpPointsTrash.clear();
};




