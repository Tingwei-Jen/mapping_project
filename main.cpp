#include "map.h"
#include "mapping.h"
#include "visualize.h"

#include <iostream>
using namespace std;

int main()
{

	Mapping* mapping = new Mapping();
	mapping->CreateMap2();
	Visualize::VisualizeMap(mapping->GetMap());
    return 0;    
}