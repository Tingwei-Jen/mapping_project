#include "map.h"

Map::Map()
{
}

void Map::AddFrame(Frame* frame)
{
	mFrames.push_back(frame);
}

void Map::AddSeed(Seed* seed)
{
	mSeeds.push_back(seed);
}
