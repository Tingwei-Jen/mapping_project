#include "map.h"

Map::Map()
{
}

void Map::AddSign(Sign* sign)
{
	this->mSigns.push_back(sign);
}

void Map::AddMappoint(MapPoint* mappoint)
{
	this->mMappoints.push_back(mappoint);
}
