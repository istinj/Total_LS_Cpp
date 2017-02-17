#include <stdio.h>
#include <iostream>
#include <Eigen/Core>

#include "World.h"

using namespace std;
World* w;

int main(int argc, char const *argv[])
{
	w = new World();
	w->initPoses();
	w->initLandmarks();

	w->generateLandmarkObs();

	for(int i = 0; i < 100; i++){
		w->printMeasurements(i);
		cin.get();
	}
	return 0;
}
