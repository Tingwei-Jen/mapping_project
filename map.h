#pragma once
#ifndef MAP_H
#define MAP_H

#include "frame.h"
#include "mappoint.h"
#include "sign.h"

class Map
{
public:
	Map();
	void AddSign(Sign* sign);
	void AddMappoint(MapPoint* mappoint);

	std::vector<Sign*> GetAllSigns(){ return mSigns; }
	std::vector<MapPoint*> GetAllMappoints(){ return mMappoints; }

private:
	std::vector<Sign*> mSigns;
	std::vector<MapPoint*> mMappoints;

};
#endif //MAP_H