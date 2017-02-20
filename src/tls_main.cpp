#include <stdio.h>
#include <iostream>
#include <Eigen/Core>

#include "World.h"
#include "Solver.h"

using namespace std;

World* w;

int main(int argc, char const *argv[])
{
	w = new World();
	w->initWorld(10.0f, 10.0f, 10.0f);

	return 0;
}
