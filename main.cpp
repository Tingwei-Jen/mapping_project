#include "test.h"
#include <iostream>

int main()
{
    Test *test = new Test();
    //test->testMovingObject();
    //test->testReconstructMonitor();
    //test->testReconstructMonitorHomography();
    test->testReconstructLandmark();
    return 0;
}