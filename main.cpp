#include "mapping.h"
#include <iostream>

int main()
{
    Mapping *mapping = new Mapping();
   	mapping->CreateMapPoints();
   	mapping->MovingObjectTest();




    return 0;
}