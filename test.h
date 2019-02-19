#pragma once
#ifndef TEST_H
#define TEST_H

#include "pcl.h"
#include "mapping.h"
#include "utils.h"

class Test
{
public:
	Test();
	void testMovingObject();
	void testReconstructMonitor();
	void testReconstructMonitorHomography();
	void testReconstructLandmark();


private:
	void getMyntParameter(Mat& K, Mat& distort, int& width, int& height);

};
#endif //TEST_H