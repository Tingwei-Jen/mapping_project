#pragma once
#ifndef MAP_H
#define MAP_H

#include "frame.h"
#include "seed.h"

class Map
{
public:
	Map();
	void AddFrame(Frame* frame);
	void AddSeed(Seed* seed);

	std::vector<Frame*> GetAllFrames(){ return mFrames; }
	std::vector<Seed*> GetAllSeeds(){ return mSeeds; }

private:
	std::vector<Frame*> mFrames;
	std::vector<Seed*> mSeeds;

};
#endif //MAP_H